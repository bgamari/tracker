#include "adc_forward.h"
#include "beagle_spi.h"

static uint16_t buf1[1024][4] __attribute__((section (".dma_data"))) = { };
static uint16_t buf2[1024][4] __attribute__((section (".dma_data"))) = { };

void adcfwd_buffer_full(struct adc_t *adc)
{
    uint16_t *buf = adc_get_idle_buffer(adc);
    beagle_spi_dma_tx(1024*4*2, (char *) buf);
}

void adcfwd_start(struct adc_t *adc)
{
    adc->buffer_full_cb = adcfwd_buffer_full;
    adc_dma_start(adc, 1024, &buf1[0][0], &buf2[0][0]);
}

void adcfwd_stop(struct adc_t *adc)
{
    adc_dma_stop(adc);
}

