#include <libopencm3/cm3/nvic.h>
#include <libopencm3/lpc43xx/timer.h>

#include "timer.h"
#include "scan.h"
#include "event.h"
#include "dac.h"
#include "adc.h"

static struct event_t scan_done;

static struct raster_scan_t cur_scan;
static int idx_x, idx_y;
static int dir;

void raster_scan(struct raster_scan_t *scan)
{
    init_event(&scan_done);
    cur_scan = *scan;
    idx_x = -cur_scan.size_x / 2;
    idx_y = -cur_scan.size_y / 2;
    dir = 1;

    nvic_enable_irq(NVIC_TIMER3_IRQ);
    setup_periodic_timer(TIMER3, scan->freq);
    TIMER3_MCR |= TIMER_MCR_MR3I;  // interrupt on overflow match
    timer_enable_counter(TIMER3);
    event_wait(&scan_done);
}

void timer3_isr()
{
    TIMER3_IR = 0xf;  // Clear interrupt
    if (idx_y == (int) cur_scan.size_y) {
        timer_disable_counter(TIMER3);
        event_fire(&scan_done);
    }

    if (idx_x == dir * (int) cur_scan.size_x / 2) {
        dir *= -1;
        idx_y++;
    }

    idx_x += dir;
    uint16_t pos_x = cur_scan.center_x;
    pos_x += cur_scan.step_x * idx_x;
    uint16_t pos_y = cur_scan.center_y;
    pos_y += cur_scan.step_y * idx_y;

    adc_manual_trigger();

    struct dac_update_t updates[2];
    updates[0].channel = channel_a;
    updates[0].value = pos_x;
    updates[1].channel = channel_b;
    updates[1].value = pos_y;
    set_dac(2, updates);
}

