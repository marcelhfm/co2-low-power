#include "epd_3in7.h"

#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(epd_3in7, LOG_LEVEL_INF);

#define EPD_NODE DT_NODELABEL(epd_3in7)

static const struct spi_dt_spec epd_spi =
    SPI_DT_SPEC_GET(EPD_NODE, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0);

statuc const struct gpio_dt_spec epd_dc_gpio =
    GPIO_DT_SPEC_GET(EPD_NODE, dc_gpios);

statuc const struct gpio_dt_spec epd_reset_gpio =
    GPIO_DT_SPEC_GET(EPD_NODE, reset_gpios);

statuc const struct gpio_dt_spec epd_busy_gpio =
    GPIO_DT_SPEC_GET(EPD_NODE, busy_gpios);

static inline void epd_write_byte(uint8_t data) {
  struct spi_buf tx_buf = {.buf = &data, .len = 1};
  struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
  (void)spi_write_dt(&epd_spi, &tx);
}

static inline void epd_write_buffer(const uint8_t *data, size_t size) {
  struct spi_buf tx_buf = {.buf = (void *)data, .len = size};
  struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
  (void)spi_write_dt(&epd_spi, &tx);
}

static void epd_send_command(uint8_t cmd) {
  // DC pin = 0 => command
  gpio_pin_set_dt(&epd_dc_gpio, 0);
  epd_write_byte(cmd);
}

static void epd_send_data(uint8_t data) {
  // DC pin = 1 => data
  gpio_pin_set_dt(&epd_dc_gpio, 1);
  epd_write_byte(data);
}

static void epd_reset(void) {
  gpio_pin_set_dt(&epd_reset_gpio, 1);
  k_msleep(30);

  gpio_pin_set_dt(&epd_reset_gpio, 0);
  k_msleep(3);

  gpio_pin_set_dt(&epd_reset_gpio, 1);
  k_msleep(30);
}

static void epd_wait_busy(void) {
  LOG_DBG("Waiting for e-paper busy...");
  while (gpio_pin_get_dt(&epd_busy_gpio)) {
    k_msleep(10);
  }
  k_msleep(200);
  LOG_DBG("e-paper is ready.");
}

void epd_3in7_init(void) {
  LOG_INF("Initializing EPD 3in7...");

  if (!device_is_ready(epd_spi.bus)) {
    LOG_ERR("SPI bus %s not ready!", epd_spi.bus->name);
    return;
  }

  // Configure GPIO pins
  gpio_pin_configure_dt(&epd_dc_gpio, GPIO_OUTPUT_INACTIVE);
  gpio_pin_configure_dt(&epd_reset_gpio, GPIO_OUTPUT_INACTIVE);
  gpio_pin_configure_dt(&epd_busy_gpio, GPIO_INPUT);

  // HW reset
  epd_reset();

  epd_send_command(0x12); // SW reset
  k_msleep(300);

  epd_wait_busy();

  LOG_INF("EPD init done!");
}

void epd_3in7_clear(void) {
  LOG_INF("Clearing EPD...");

  // TBD
  epd_wait_busy();
}

void epd_3in7_display(const uint8_t *image_data, size_t image_size) {
  LOG_INF("Writing image data of size %u", image_size);

  // TBD
  epd_wait_busy();
}
