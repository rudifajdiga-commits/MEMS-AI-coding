#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"

#include "lwip/ip_addr.h"

// --- I2C Pins ---
#define PIN_IMU_SDA  12
#define PIN_IMU_SCL  13
#define PIN_IMU_CS   10
#define PIN_IMU_INT1 11

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000 
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// --- LSM6DSV16X Registers ---
#define REG_WHO_AM_I   0x0F
#define REG_CTRL1      0x10
#define REG_CTRL2      0x11
#define REG_CTRL3      0x12
#define REG_CTRL6      0x15
#define REG_CTRL8      0x17
#define REG_STATUS     0x1E
#define REG_OUT_TEMP_L 0x20
#define REG_HAODR_CFG  0x62
#define WHO_AM_I_EXPECTED 0x70

static const char *TAG = "imu_web";
static uint8_t imu_addr = 0;
static httpd_handle_t server = NULL;

// --- Madgwick Filter ---
#define betaDef 0.1f // Standard proportional gain
volatile float beta = 2.5f; // START HIGH for fast initial convergence (fixes startup drift)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
int samples_collected = 0;

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float sampleFreq) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    gx *= 0.0174533f; gy *= 0.0174533f; gz *= 0.0174533f;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;   

        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2; _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
    }

    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

// --- Embedded Files ---
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");
extern const uint8_t FinalBaseMesh_glb_gz_start[] asm("_binary_FinalBaseMesh_glb_gz_start");
extern const uint8_t FinalBaseMesh_glb_gz_end[]   asm("_binary_FinalBaseMesh_glb_gz_end");

static esp_err_t index_html_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
}

static esp_err_t model_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "model/gltf-binary");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    return httpd_resp_send(req, (const char *)FinalBaseMesh_glb_gz_start, FinalBaseMesh_glb_gz_end - FinalBaseMesh_glb_gz_start);
}

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) return ESP_OK; // Handshake
    return ESP_OK;
}

static const httpd_uri_t uri_get = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = index_html_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_model = {
    .uri      = "/FinalBaseMesh.glb",
    .method   = HTTP_GET,
    .handler  = model_handler,
    .user_ctx = NULL
};

static const httpd_uri_t ws_uri = {
    .uri        = "/ws",
    .method     = HTTP_GET,
    .handler    = ws_handler,
    .user_ctx   = NULL,
    .is_websocket = true
};

static void start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7;
    config.uri_match_fn = httpd_uri_match_wildcard;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_get);
        httpd_register_uri_handler(server, &uri_model);
        httpd_register_uri_handler(server, &ws_uri);
        ESP_LOGI(TAG, "Webserver started on port %d", config.server_port);
    }
}

// Struct to hold data for async WS sending
struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
    char str[256];
};

static void ws_async_send(void *arg) {
    struct async_resp_arg *resp_arg = arg;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)resp_arg->str;
    ws_pkt.len = strlen(resp_arg->str);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    httpd_ws_send_frame_async(resp_arg->hd, resp_arg->fd, &ws_pkt);
    free(resp_arg);
}

static void broadcast_ws_data(const char* data) {
    if (!server) return;
    size_t fds = 10; // Max clients
    int client_fds[10];
    if (httpd_get_client_list(server, &fds, client_fds) == ESP_OK) {
        for (int i = 0; i < fds; i++) {
            httpd_ws_client_info_t client_info = httpd_ws_get_fd_info(server, client_fds[i]);
            if (client_info == HTTPD_WS_CLIENT_WEBSOCKET) {
                struct async_resp_arg *resp_arg = malloc(sizeof(struct async_resp_arg));
                if (resp_arg) {
                    resp_arg->hd = server;
                    resp_arg->fd = client_fds[i];
                    strncpy(resp_arg->str, data, sizeof(resp_arg->str)-1);
                    // Queue the async send
                    if (httpd_queue_work(server, ws_async_send, resp_arg) != ESP_OK) {
                        free(resp_arg); // Free if failed to queue
                    }
                }
            }
        }
    }
}

// --- WiFi AP Setup ---
static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    // Set static IP to 192.168.4.1
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(ap_netif, &ip_info);
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    ip_info.gw.addr = ip_info.ip.addr;
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ip_info);
    esp_netif_dhcps_start(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_IMU",
            .ssid_len = strlen("ESP32_IMU"),
            .channel = 1,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP Started. Connect to SSID: ESP32_IMU, Password: 12345678");
    ESP_LOGI(TAG, "Webserver available at http://192.168.4.1");
}

// --- I2C Helpers ---
static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_IMU_SDA,
        .scl_io_num = PIN_IMU_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t write_register(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM, addr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
static esp_err_t read_register(uint8_t addr, uint8_t reg, uint8_t *value) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, value, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
static esp_err_t read_registers(uint8_t addr, uint8_t reg, uint8_t *buffer, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, buffer, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static uint8_t find_imu(void) {
    const uint8_t addrs[] = {0x6B, 0x6A};
    for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
        uint8_t who = 0;
        if (read_register(addrs[i], REG_WHO_AM_I, &who) == ESP_OK && who == WHO_AM_I_EXPECTED) return addrs[i];
    }
    return 0;
}

static bool init_imu(void) {
    imu_addr = find_imu();
    if (imu_addr == 0) return false;

    write_register(imu_addr, REG_CTRL3, 0x01); // SW_RESET
    vTaskDelay(20 / portTICK_PERIOD_MS);

    uint8_t current;
    read_register(imu_addr, REG_CTRL3, &current);
    write_register(imu_addr, REG_CTRL3, (current & ~0x44) | 0x44);

    read_register(imu_addr, REG_HAODR_CFG, &current);
    write_register(imu_addr, REG_HAODR_CFG, (current & ~0x03) | 0x02); // 100Hz

    read_register(imu_addr, REG_CTRL6, &current);
    write_register(imu_addr, REG_CTRL6, (current & ~0x0F) | 0x01); // 250dps

    read_register(imu_addr, REG_CTRL8, &current);
    write_register(imu_addr, REG_CTRL8, (current & ~0x03) | 0x03); // 16g

    read_register(imu_addr, REG_CTRL1, &current);
    write_register(imu_addr, REG_CTRL1, (current & ~0x0F) | 0x06); // Accel 100Hz

    read_register(imu_addr, REG_CTRL2, &current);
    write_register(imu_addr, REG_CTRL2, (current & ~0x0F) | 0x06); // Gyro 100Hz

    return true;
}

void app_main(void) {
    // Initialize NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();
    start_webserver();

    gpio_set_direction(PIN_IMU_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_IMU_CS, 1);
    gpio_set_direction(PIN_IMU_INT1, GPIO_MODE_INPUT);

    vTaskDelay(20 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_master_init());
    
    if (!init_imu()) {
        ESP_LOGE(TAG, "IMU init failed");
        while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Streaming at 100Hz...");

    int64_t last_time = esp_timer_get_time();
    char json_buf[256];

    while (1) {
        uint8_t status = 0;
        if (read_register(imu_addr, REG_STATUS, &status) != ESP_OK) continue;

        if ((status & 0x03) != 0) {
            uint8_t raw[14];
            if (read_registers(imu_addr, REG_OUT_TEMP_L, raw, sizeof(raw)) != ESP_OK) continue;

            int64_t current_time = esp_timer_get_time();
            float dt = (current_time - last_time) / 1000000.0f;
            last_time = current_time;

            int16_t gxRaw = (int16_t)((raw[3] << 8) | raw[2]);
            int16_t gyRaw = (int16_t)((raw[5] << 8) | raw[4]);
            int16_t gzRaw = (int16_t)((raw[7] << 8) | raw[6]);
            int16_t axRaw = (int16_t)((raw[9] << 8) | raw[8]);
            int16_t ayRaw = (int16_t)((raw[11] << 8) | raw[10]);
            int16_t azRaw = (int16_t)((raw[13] << 8) | raw[12]);

            float ax = axRaw * 0.488f / 1000.0f; // 16g
            float ay = ayRaw * 0.488f / 1000.0f;
            float az = azRaw * 0.488f / 1000.0f;

            float gx = gxRaw * 8.75f / 1000.0f; // 250dps
            float gy = gyRaw * 8.75f / 1000.0f;
            float gz = gzRaw * 8.75f / 1000.0f;

            if (dt > 0.0f && dt < 1.0f) {
                MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, 1.0f / dt);
                
                // Drift fix: After ~2 seconds of high-beta stabilization, lower beta to smooth tracking
                samples_collected++;
                if (samples_collected == 200) {
                    beta = betaDef; // Drop to 0.1f for smooth tracking
                    ESP_LOGI(TAG, "Filter stabilized. Beta lowered.");
                }
            }

            float roll  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 57.29578f;
            float pitch = asinf(2.0f * (q0*q2 - q3*q1)) * 57.29578f;
            float yaw   = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 57.29578f;

            // Still print to serial monitor
            printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", roll, pitch, yaw, ax, ay, az);

            // Broadcast to WebSockets
            if (server) {
                snprintf(json_buf, sizeof(json_buf), "{\"r\":%.2f,\"p\":%.2f,\"y\":%.2f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"q0\":%.4f,\"q1\":%.4f,\"q2\":%.4f,\"q3\":%.4f}", 
                         roll, pitch, yaw, ax, ay, az, q0, q1, q2, q3);
                broadcast_ws_data(json_buf);
            }

        } else {
            vTaskDelay(2 / portTICK_PERIOD_MS);
        }
    }
}

