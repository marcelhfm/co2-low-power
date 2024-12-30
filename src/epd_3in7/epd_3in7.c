// ported from
// https://github.com/waveshareteam/Pico_ePaper_Code/blob/main/c/lib/e-Paper/EPD_3in7.c

#include "epd_3in7.h"
#include "zephyr/device.h"

#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

static const uint8_t lut_1Gray_DU[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 1
    0x01, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0A, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 3
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 5
    0x00, 0x00, 0x05, 0x05, 0x00, 0x05, 0x03, 0x05, 0x05, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 9
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x22, 0x22, 0x22, 0x22, 0x22};

LOG_MODULE_REGISTER(epd_3in7, LOG_LEVEL_DBG);

#define SPI2_NODE DT_NODELABEL(spi2)
#define GPIO0_NODE DT_NODELABEL(gpio0)
#define GPIO1_NODE DT_NODELABEL(gpio1)

static const struct device *spi_dev = DEVICE_DT_GET(SPI2_NODE);

static struct spi_config epd_spi_cfg = {
    .frequency = 8000000,
    .operation = (SPI_WORD_SET(8) | SPI_TRANSFER_MSB),
    .slave = 0,
};

#define EPD_DC_PIN 4     // GPIO 0.4
#define EPD_RESET_PIN 29 // GPIO 0.29
#define EPD_BUSY_PIN 5   // GPIO 0.5
#define EPD_CS_PIN 14    // GPIO 1.14

static const struct device *dc_dev = DEVICE_DT_GET(GPIO0_NODE);
static const struct device *reset_dev = DEVICE_DT_GET(GPIO0_NODE);
static const struct device *busy_dev = DEVICE_DT_GET(GPIO0_NODE);
static const struct device *cs_dev = DEVICE_DT_GET(GPIO1_NODE);

/* SPI writes */
static inline void epd_write_byte(uint8_t data) {
  struct spi_buf tx_buf = {.buf = &data, .len = 1};
  struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

  int ret;
  ret = gpio_pin_set(cs_dev, EPD_CS_PIN, 0);
  if (ret < 0) {
    LOG_ERR("Error while setting cs pin to 0: %d", ret);
    return;
  }
  ret = spi_write(spi_dev, &epd_spi_cfg, &tx);
  if (ret < 0) {
    LOG_ERR("Error while writing spi: %d", ret);
    return;
  }
  ret = gpio_pin_set(cs_dev, EPD_CS_PIN, 1);
  if (ret < 0) {
    LOG_ERR("Error while setting cs pin to 1: %d", ret);
  }
}

/* Set DC pin (0=cmd, 1=data). */
static inline void epd_set_dc(bool value) {
  if (gpio_pin_set(dc_dev, EPD_DC_PIN, value) < 0) {
    LOG_ERR("Error while setting DC pin to value %d", value);
  }
}

/* Send command or data over SPI. */
static void epd_send_command(uint8_t cmd) {
  epd_set_dc(0); // DC pin = 0 => command
  epd_write_byte(cmd);
}

static void epd_send_data(uint8_t data) {
  epd_set_dc(1); // DC pin = 1 => data
  epd_write_byte(data);
}

/* Hardware reset pin toggling */
static void epd_reset(void) {
  LOG_DBG("Reset started");
  gpio_pin_set(reset_dev, EPD_RESET_PIN, 1);
  k_msleep(30);

  gpio_pin_set(reset_dev, EPD_RESET_PIN, 0);
  k_msleep(3);

  gpio_pin_set(reset_dev, EPD_RESET_PIN, 1);
  k_msleep(30);
  LOG_DBG("Hardware reset finished");
}

static void epd_wait_busy(void) {
  while (gpio_pin_get(busy_dev, EPD_BUSY_PIN)) {
    k_msleep(10);
  }
  k_msleep(200);
}

void epd_3in7_load_lut(void) {
  uint16_t i;
  epd_send_command(0x32);
  for (i = 0; i < 105; i++) {
    epd_send_data(lut_1Gray_DU[i]);
  }
}

void epd_3in7_1gray_init(void) {
  LOG_DBG("Initializing...");
  epd_wait_busy();

  epd_reset();

  epd_send_command(0x12);
  k_msleep(300);

  epd_send_command(0x46); // Clear and fill RAM // Clear and fill RAM
  epd_send_data(0xF7);
  epd_wait_busy();
  epd_send_command(0x47);
  epd_send_data(0xF7);
  epd_wait_busy();

  epd_send_command(0x01); // setting gaet number
  epd_send_data(0xDF);
  epd_send_data(0x01);
  epd_send_data(0x00);

  epd_send_command(0x03); // set gate voltage
  epd_send_data(0x00);

  epd_send_command(0x04); // set source voltage
  epd_send_data(0x41);
  epd_send_data(0xA8);
  epd_send_data(0x32);

  epd_send_command(0x11); // set data entry sequence
  epd_send_data(0x03);

  epd_send_command(0x3C); // set border
  epd_send_data(0x03);

  epd_send_command(0x0C); // set booster strength
  epd_send_data(0xAE);
  epd_send_data(0xC7);
  epd_send_data(0xC3);
  epd_send_data(0xC0);
  epd_send_data(0xC0);

  epd_send_command(0x18); // set internal sensor on
  epd_send_data(0x80);

  epd_send_command(0x2C); // set vcom value
  epd_send_data(0x44);

  epd_send_command(
      0x37); // set display option, these setting turn on previous function
  epd_send_data(0x00); // can switch 1 gray or 4 gray
  epd_send_data(0xFF);
  epd_send_data(0xFF);
  epd_send_data(0xFF);
  epd_send_data(0xFF);
  epd_send_data(0x4F);
  epd_send_data(0xFF);
  epd_send_data(0xFF);
  epd_send_data(0xFF);
  epd_send_data(0xFF);

  epd_send_command(0x44); // setting X direction start/end position of RAM
  epd_send_data(0x00);
  epd_send_data(0x00);
  epd_send_data(0x17);
  epd_send_data(0x01);

  epd_send_command(0x45); // setting Y direction start/end position of RAM
  epd_send_data(0x00);
  epd_send_data(0x00);
  epd_send_data(0xDF);
  epd_send_data(0x01);

  epd_send_command(0x22); // Display Update Control 2
  epd_send_data(0xCF);
}

void epd_3in7_clear(void) {
  LOG_DBG("Clearing display");
  uint16_t i;
  uint16_t IMAGE_COUNTER = EPD_3IN7_WIDTH * EPD_3IN7_HEIGHT / 8;

  epd_send_command(0x4E);
  epd_send_data(0x00);
  epd_send_data(0x00);
  epd_send_command(0x4F);
  epd_send_data(0x00);
  epd_send_data(0x00);

  epd_send_command(0x24);
  for (i = 0; i < IMAGE_COUNTER; i++) {
    epd_send_data(0xff);
  }

  epd_3in7_load_lut();

  epd_send_command(0x20);
  epd_wait_busy();
}

void epd_3in7_display(const uint8_t *image) {
  LOG_DBG("Displaying image...");
  uint16_t i;
  uint16_t IMAGE_COUNTER = EPD_3IN7_WIDTH * EPD_3IN7_HEIGHT / 8;

  epd_send_command(0x4E);
  epd_send_data(0x00);
  epd_send_data(0x00);
  epd_send_command(0x4F);
  epd_send_data(0x00);
  epd_send_data(0x00);

  epd_send_command(0x24);
  for (i = 0; i < IMAGE_COUNTER; i++) {
    epd_send_data(image[i]);
  }

  epd_3in7_load_lut();
  epd_send_command(0x20);
  epd_wait_busy();
}

void epd_3in7_sleep(void) {
  LOG_DBG("Entering deep sleep");
  epd_send_command(0X10); // deep sleep
  epd_send_data(0x03);
}

void epd_3in7_init(void) {
  LOG_INF("Initializing EPD 3in7...");

  LOG_DBG("Checking if SPI is ready");
  if (!device_is_ready(spi_dev)) {
    LOG_ERR("SPI bus %s not ready!", spi_dev->name);
    return;
  }

  LOG_DBG("Configuring GPIO pins");

  int res;
  res = gpio_pin_configure(dc_dev, EPD_DC_PIN, GPIO_OUTPUT_INACTIVE);
  if (res < 0) {
    LOG_ERR("Error while configuring DC pin: %d", res);
    return;
  }

  res = gpio_pin_configure(reset_dev, EPD_RESET_PIN, GPIO_OUTPUT_INACTIVE);
  if (res < 0) {
    LOG_ERR("Error while configuring RESET pin: %d", res);
    return;
  }

  res = gpio_pin_configure(busy_dev, EPD_BUSY_PIN, GPIO_INPUT);
  if (res < 0) {
    LOG_ERR("Error while configuring BUSY pin: %d", res);
    return;
  }

  gpio_pin_configure(cs_dev, EPD_CS_PIN, GPIO_OUTPUT_INACTIVE);
  if (res < 0) {
    LOG_ERR("Error while configuring CS pin: %d", res);
    return;
  }

  epd_3in7_1gray_init();
}
