#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#if CONFIG_CONNECT_WIFI
# include "esp_wifi.h"
#else
# error "Including wifi_connect.h, but CONFIG_CONNECT_WIFI is not set"
#endif

#ifdef CONFIG_CONNECT_WIFI
# define INTERFACE get_example_netif()
#endif

#if !defined (CONFIG_CONNECT_WIFI)
// This is useful for some tests which do not need a network connection
# define INTERFACE NULL
#endif

/**
 * @brief Init Wifi connection
 *
 * Initializes NVS first and then netif. If no errror is set, conitnues with the wifi connection.
 *
 * @return ESP_OK on successful connection
 */
esp_err_t wifi_init(void);

/**
 * @brief Configure Wi-Fi or Ethernet, connect, wait for IP
 *
 * This all-in-one helper function is used in protocols examples to
 * reduce the amount of boilerplate in the example.
 *
 * It is not intended to be used in real world applications.
 * See examples under examples/wifi/getting_started/ and examples/ethernet/
 * for more complete Wi-Fi or Ethernet initialization code.
 *
 * Read "Establishing Wi-Fi or Ethernet Connection" section in
 * examples/protocols/README.md for more information about this function.
 *
 * @return ESP_OK on successful connection
 */
esp_err_t wifi_connect(void);

/**
 * Counterpart to example_connect, de-initializes Wi-Fi or Ethernet
 */
esp_err_t wifi_disconnect(void);

/**
 * @brief Returns esp-netif pointer created by example_connect()
 *
 * @note If multiple interfaces active at once, this API return NULL
 * In that case the get_example_netif_from_desc() should be used
 * to get esp-netif pointer based on interface description
 */
esp_netif_t *get_example_netif(void);

/**
 * @brief Returns esp-netif pointer created by example_connect() described by
 * the supplied desc field
 *
 * @param desc Textual interface of created network interface, for example "sta"
 * indicate default WiFi station, "eth" default Ethernet interface.
 *
 */
esp_netif_t *get_example_netif_from_desc(const char *desc);

#ifdef __cplusplus
}
#endif

