#pragma once

#include <stdint.h>

void beagle_spi_init();

typedef void (*completion_t)(void *user);

int beagle_spi_dma_tx(uint16_t length, char *buffer);
int beagle_spi_dma_rx(uint16_t length, char *buffer,
                      completion_t rx_cb, void *user_data);

