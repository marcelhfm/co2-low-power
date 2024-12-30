#include "epd_3in7/epd_3in7.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define READ_CO2_THREAD_STACK_SIZE 1024
#define READ_CO2_THREAD_PRIORITY 7

#define LED2_NODE DT_ALIAS(led2)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED2_NODE, gpios);

static const uint8_t mhz19c_read_cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x79};

void read_co2_thread(void *arg1, void *arg2, void *arg3);
K_THREAD_DEFINE(read_co2_tid, READ_CO2_THREAD_STACK_SIZE, read_co2_thread, NULL,
                NULL, NULL, READ_CO2_THREAD_PRIORITY, 0, 0);

void toggle_led() {
  int ret = gpio_pin_toggle_dt(&led);
  if (ret < 0) {
    LOG_ERR("[LED]: Error while toggling LED\n");
  }
}

void read_co2_thread(void *arg1, void *arg2, void *arg3) {
  LOG_INF("Hello from read co2 thread!\n");
  const struct device *uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

  if (!device_is_ready(uart_dev)) {
    LOG_ERR("Error: UART device not ready.\n");
    return;
  }

  struct uart_config cfg = {
      .baudrate = 9600,
      .parity = UART_CFG_PARITY_NONE,
      .stop_bits = UART_CFG_STOP_BITS_1,
      .data_bits = UART_CFG_DATA_BITS_8,
      .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
  };

  if (uart_configure(uart_dev, &cfg) != 0) {
    LOG_ERR("Error: Unable to configure UART.\n");
    return;
  }

  uint8_t rx_buf[9];
  int co2_ppm = 0;

  while (1) {
    memset(rx_buf, 0, sizeof(rx_buf));

    // send 9-byte read command
    for (int i = 0; i < 9; i++) {
      toggle_led();
      uart_poll_out(uart_dev, mhz19c_read_cmd[i]);
    }

    // poll 9-byte response
    for (int i = 0; i < 9; i++) {
      toggle_led();
      while (uart_poll_in(uart_dev, &rx_buf[i]) < 0) {
        k_sleep(K_MSEC(10));
      }
    }

    // parse response
    if (rx_buf[0] == 0xFF && rx_buf[1] == 0x86) {
      co2_ppm = (rx_buf[2] << 8) | rx_buf[3];
      LOG_INF("CO2 = %d ppm\n", co2_ppm);
    } else {
      LOG_ERR("CO2 Sensor: Invalid response or checksum error.");
      printk("Raw response bytes: ");
      for (int i = 0; i < 9; i++) {
        printk("0x%02X ", rx_buf[i]);
      }
      printk("\n");

      co2_ppm = (rx_buf[2] << 8) | rx_buf[3];
      LOG_DBG("FAILED CHECKSUM CO2 = %d ppm", co2_ppm);
    }

    k_sleep(K_SECONDS(60));
  }
}

int main(void) {
  k_sleep(K_SECONDS(5));
  int ret;

  if (!gpio_is_ready_dt(&led)) {
    return 0;
  }

  ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    return 0;
  }

  LOG_INF("Hello from co2-low-power\n");

  epd_3in7_init();
  epd_3in7_clear();

  static uint8_t test_buffer[EPD_3IN7_WIDTH * EPD_3IN7_HEIGHT / 8];
  memset(test_buffer, 0x00, sizeof(test_buffer));
  epd_3in7_display(test_buffer);

  while (1) {
    k_sleep(K_SECONDS(5));
  }
}
