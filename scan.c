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
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    NVIC_EnableIRQ(TIM4_IRQn);
    while (SlowPeripheralClock / prescaler / scan->freq > 0xffff)
        prescaler *= 2;
    TIM4->PSC = prescaler - 1;
    TIM4->ARR = SlowPeripheralClock / prescaler / scan->freq;
    TIM4->CR1 = TIM_CR1_ARPE;
    TIM4->CR2 = 0;
    TIM4->DIER = TIM_DIER_CC4IE;
    TIM4->CCR1 = 0;
    TIM4->CCR4 = TIM4->ARR / 2;
    TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC4E;
    TIM4->CR1 |= TIM_CR1_CEN;
    event_wait(&scan_done);
}

void TIM4_IRQHandler()
{
    if (idx_y == cur_scan.size_y) {
        TIM4->CR1 &= ~TIM_CR1_CEN;
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

