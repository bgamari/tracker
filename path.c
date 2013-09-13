#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <libopencm3/lpc43xx/timer.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>

#include "adc.h"
#include "feedback.h"
#include "timer.h"
#include "path.h"

struct path {
        struct path* next;
        bool queued;
        unsigned int npts;
        uint16_t points[PATH_MAX_POINTS][3];
};

#define N_PATHS 8
static struct path paths[N_PATHS] = {};
static struct path* active_path = NULL;
static unsigned int active_point;
static bool path_running = false;
static bool sync_trigger = false;

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

        for (unsigned int i=0; i<npts; i++)
                for (unsigned int j=0; j<3; j++)
                        path->points[i][j] = points[3*i+j];
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
        path_running = false;
        for (unsigned int i=0; i<N_PATHS; i++)
                put_path(&paths[i]);
        cm_enable_interrupts();
}

int start_path(unsigned int freq, bool synchronous_trigger)
{
        if (path_running)
                return -1;
        if (active_path == NULL)
                return -2;

        feedback_set_mode(NO_FEEDBACK);
        sync_trigger = synchronous_trigger;
        active_point = 0;
        path_running = true;
        setup_periodic_timer(TIMER1, freq);
        nvic_enable_irq(NVIC_TIMER1_IRQ);
        TIMER1_MCR |= TIMER_MCR_MR3I;  // interrupt on overflow match
        timer_enable_counter(TIMER1);
        if (synchronous_trigger) {
                adc_set_trigger_mode(TRIGGER_MANUAL);
        } else {
                adc_set_trigger_mode(TRIGGER_AUTO);
        }
        return 0;
}

bool is_path_running()
{
        return path_running;
}

static void path_done()
{
        timer_disable_counter(TIMER1);
        path_running = false;
        adc_set_trigger_mode(TRIGGER_OFF);
}

void timer1_isr()
{
        TIMER1_IR = 0xf;  // Clear interrupt
        if (active_point >= active_path->npts) {
                struct path* old = active_path;
                active_path = active_path->next;
                active_point = 0;
                put_path(old);
        }
        if (active_path == NULL) {
                path_done();
                return;
        }
        
        // This will be ignored until synchronous triggering is enabled
        adc_manual_trigger();

        feedback_set_position(active_path->points[active_point]);
        active_point++;
}
