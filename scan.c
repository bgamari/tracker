#include <libopencm3/stm32/f4/rcc.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/f4/timer.h>
#include "scan.h"
#include "event.h"
#include "dac.h"

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

    unsigned int prescaler = 1;
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);
    nvic_enable_irq(NVIC_TIM4_IRQ);
    timer_reset(TIM4);
    timer_set_mode(TIM4, 0, 0, TIM_CR1_DIR_UP);
    while (rcc_ppre1_frequency / prescaler / scan->freq > 0xffff)
        prescaler *= 2;
    timer_set_prescaler(TIM4, prescaler);
    u32 period = rcc_ppre1_frequency / prescaler / scan->freq;
    timer_set_period(TIM4, period);
    timer_enable_preload(TIM4);

    timer_enable_oc_output(TIM4, TIM_OC1);
    timer_enable_oc_output(TIM4, TIM_OC4);
    timer_enable_irq(TIM4, TIM_DIER_CC4IE);
    timer_enable_counter(TIM4);
    event_wait(&scan_done);
}

void TIM4_IRQHandler()
{
    timer_clear_flag(TIM4, 0xffffffff);
    if (idx_y == cur_scan.size_y) {
        timer_disable_counter(TIM4);
        event_fire(&scan_done);
    }

    if (idx_x == dir * cur_scan.size_x / 2) {
        dir *= -1;
        idx_y++;
    }

    idx_x += dir;
    uint16_t pos_x = cur_scan.center_x;
    pos_x += cur_scan.step_x * idx_x;
    uint16_t pos_y = cur_scan.center_y;
    pos_y += cur_scan.step_y * idx_y;

    struct dac_update_t updates[2];
    updates[0].channel = channel_a;
    updates[0].value = pos_x;
    updates[1].channel = channel_b;
    updates[1].value = pos_y;
    set_dac(2, updates);
}

