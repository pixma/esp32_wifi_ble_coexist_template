#ifndef WIFI_ACTIVITY_H_
#define WIFI_ACTIVITY_H_

#include "main.h"

#include "esp_err.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_http_server.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* The HTTP server port */
#define ESP_AP_SERVER_PORT 80
#define ESP_SOFTAP_WIFI_RESPONSE_HEADER "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"
#define HTTPD_401 "401 UNAUTHORIZED" /*!< HTTP Response 401 */
#define FILE_BUF_SIZE 512
/* Define buffer limit for reading file */
#define BUFFER_LIMIT (1 * 1024)
#define SPIFFS_MAX_FILE_SIZE (64 * 1024)

#define FILE_PATH_DIR_WEB_APP "/spiffs"
#define FILE_PATH_INDEX_HTML "/spiffs/index.html"
#define FILE_PATH_DASHBOARD_HTML "/spiffs/dashboard.html"
#define FILE_PATH_SIIMPLE_CSS "/spiffs/siimple.min.css"
#define FILE_PATH_SIIMPLE_CSS_ICONS "/spiffs/siimple-icons.css"

#define NO_HTML 0
#define NO_CSS 0

#define ESP_VFS_PATH_MAX 64

#define ESP_AP_ADMIN_USERNAME "admin"
#define ESP_AP_ADMIN_PASSWORD "passkey99*"

#define AUTH_OK_SCRIPT "<html><script>window.onload=function(){window.location.href = \"/dashboard\";}</script></html>"
#define AUTH_FAIL_SCRIPT "<html><script>window.onload=function(){window.location.href = \"/\";}</script></html>"
// WiFi related Macros/Definations starts here
#define DEVICE_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define DEVICE_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define DEVICE_ESP_MAXIMUM_RETRY CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

enum httpRequestedFileType
{
    HTML_FILE,
    CSS_FILE
};

enum requestedHtmlFilePathIndex{
    INDEX_HTML_PATH=1,
    DASHBOARD_HTML_PATH=2
};

enum requestedCSSFilePathIndex{
    INDEX_CSS_PATH=1,
    DASHBOARD_CSS_PATH=2
};

/**
 * @brief
 *
 */
void wifi_init_sta(void);

/**
 * @brief
 *
 * @param req
 * @param fileType
 * @param htmlFilePaths
 * @param cssFilePaths
 * @return esp_err_t
 */
static esp_err_t contentProvider(httpd_req_t *req, enum httpRequestedFileType fileType, int htmlFilePaths, int cssFilePaths);

/**
 * @brief
 *
 * @param arg
 * @param event_base
 * @param event_id
 * @param event_data
 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data);

/**
 * @brief
 *
 * @param arg
 * @param event_base
 * @param event_id
 * @param event_data
 */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data);

/**
 * @brief
 *
 * @param req
 * @return esp_err_t
 */
static esp_err_t dashboard_get_handler(httpd_req_t *req);
/**
 * @brief
 *
 * @param req
 * @return esp_err_t
 */
static esp_err_t auth_post_handler(httpd_req_t *req);

/**
 * @brief
 *
 * @param req
 * @return esp_err_t
 */
static esp_err_t siimple_css_get_handler(httpd_req_t *req);
/* An HTTP GET handler for the root URL */
/**
 * @brief
 *
 * @param req
 * @return esp_err_t
 */
static esp_err_t root_get_handler(httpd_req_t *req);

/**
 * @brief
 *
 * @return int
 */
int switchToAccessPointMode();
/**
 * @brief
 *
 * @return int
 */
int switchToStationMode();
/**
 * @brief
 *
 * @return int
 */
int checkSpiffSystem();
/**
 * @brief
 *
 * @return int
 */
int whichMode();

/**
 * @brief
 *
 * @param ssidName
 * @param passwd
 * @param nodeId
 * @param infraId
 * @param serverIp
 * @return int
 */
int saveCred(const char *ssidName, const char *passwd, const char *nodeId, const char *infraId, const char *serverIp);

/**
 * @brief
 *
 * @param ssidName
 * @param passwd
 * @param nodeId
 * @param infraId
 * @param serverIp
 * @return int
 */
int readCred(char *ssidName, char *passwd, char *nodeId, char *infraId, char *serverIp);

/**
 * @brief
 *
 * @return int
 */
int factoryReset();

#endif