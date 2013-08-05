#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <libopencm3/lpc43xx/timer.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>

#include "dac.h"
#include "adc.h"
#include "timer.h"
#include "path.h"
#include "event.h"

struct path {
        struct path* next;
        bool queued;
        unsigned int npts;
        uint16_t points[PATH_MAX_POINTS][3];
};

#define N_PATHS 16
static struct path paths[N_PATHS] = {};
static struct path* active_path = NULL;
static unsigned int active_point;
static struct event_t path_done;

struct path* take_path()
{
        for (unsigned int i=0; i<N_PATHS; i++) {
                if (!paths[i].queued) {
                        paths[i].queued = true;
                        return &paths[i];
                }
        }
        return NULL;
}

int enqueue_points(uint16_t* points, unsigned int npts)
{
        if (npts > PATH_MAX_POINTS)
                return -2;
        if (npts == 0)
                return -3;

        struct path* path = take_path();
        if (path == NULL)
                return -1;
        memcpy(&path->points[0][0], points, 3*2*npts);
        path->npts = npts;
        path->next = NULL;

        cm_disable_interrupts();
        if (active_path) {
                struct path* tail = active_path;
                while (tail->next != NULL) tail = tail->next;
                tail->next = path;
        } else {
                active_path = path;
        }
        cm_enable_interrupts();
        return 0;
}

static void put_path(struct path* path)
{
        path->next = NULL;
        path->queued = false;
}

void clear_path()
{
        cm_disable_interrupts();
        active_path = NULL;
        for (unsigned int i=0; i<N_PATHS; i++)
                put_path(&paths[i]);
        cm_enable_interrupts();
}

int start_path(unsigned int freq)
{
        if (path_running())
                return -1;
        if (active_path == NULL)
                return -2;

        init_event(&path_done);
        active_point = 0;
        setup_periodic_timer(TIMER3, freq);
        nvic_enable_irq(NVIC_TIMER3_IRQ);
        TIMER3_MCR |= TIMER_MCR_MR3I;  // interrupt on overflow match
        timer_enable_counter(TIMER3);
        return 0;
}

bool path_running()
{
        return active_path != NULL;
}

static struct dac_update_t updates[3] = {
        {.channel = channel_a},
        {.channel = channel_b},
        {.channel = channel_c}
};

void timer3_isr()
{
        TIMER3_IR = 0xf;  // Clear interrupt
        if (active_point >= active_path->npts) {
                struct path* old = active_path;
                active_path = active_path->next;
                active_point = 0;
                put_path(old);
        }
        if (active_path == NULL) {
                timer_disable_counter(TIMER3);
                event_fire(&path_done);
                return;
        }
        
        adc_manual_trigger();

        active_point++;
        for (unsigned int i=0; i<3; i++)
                updates[i].value = active_path->points[active_point][i];
        set_dac(3, updates);
}
