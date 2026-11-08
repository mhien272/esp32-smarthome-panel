
#include "espnow_manager.h"

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <cstring>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace EspNowManager {

// Khai báo trước callback nhận ESP-NOW
static void onReceive(
    const uint8_t* mac,
    const uint8_t* data,
    int len
);

constexpr uint8_t ESPNOW_CHANNEL = 1;

// Danh sách 4 Slave
static DeviceInfo devices[] = {
    {2, {0xA4, 0xCB, 0x8F, 0xF1, 0xF6, 0x04},
        DeviceStatus::IDLE, false},

    {3, {0xA4, 0xCB, 0x8F, 0xF1, 0x31, 0x24},
        DeviceStatus::IDLE, false},

    {5, {0xA4, 0xCB, 0x8F, 0xF1, 0xF6, 0x3C},
        DeviceStatus::IDLE, false},

    {6, {0x98, 0xC3, 0x77, 0xA7, 0xEB, 0xB8},
        DeviceStatus::IDLE, false}
};

struct Response {
    uint8_t mac[6];
    char message[64];
};

static QueueHandle_t responseQueue = nullptr;

constexpr int DEVICE_COUNT =
    sizeof(devices) / sizeof(devices[0]);

static bool initialized = false;

// Tìm thiết bị theo ID
static int findDevice(int id) {
    for (int i = 0; i < DEVICE_COUNT; i++) {
        if (devices[i].id == id) {
            return i;
        }
    }

    return -1;
}

// Khởi tạo ESP-NOW và đăng ký các Slave
bool begin() {
    if (initialized) {
        return true;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println(
            "[ESP-NOW] Wi-Fi not connected yet"
        );
        return false;
    }

    uint8_t channel = WiFi.channel();

    Serial.printf(
        "[ESP-NOW] Current Wi-Fi channel: %u\n",
        channel
    );

    if (channel != ESPNOW_CHANNEL) {
        Serial.println(
            "[ESP-NOW] Channel mismatch with Slaves"
        );
        return false;
    }

    if (esp_now_init() != ESP_OK) {
        return false;
    }

    if (responseQueue == nullptr) {
        responseQueue = xQueueCreate(
            16,
            sizeof(Response)
        );

        if (responseQueue == nullptr) {
            Serial.println("[ESP-NOW] Queue creation failed");
            return false;
        }
    }

    esp_err_t cbResult = esp_now_register_recv_cb(onReceive);

    if (cbResult != ESP_OK) {
        Serial.println("[ESP-NOW] Receive callback failed");
        return false;
    }

    for (int i = 0; i < DEVICE_COUNT; i++) {
        esp_now_peer_info_t peer = {};

        memcpy(peer.peer_addr, devices[i].mac, 6);

        peer.channel = ESPNOW_CHANNEL;
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = false;

        esp_err_t err = esp_now_add_peer(&peer);

        if (err != ESP_OK &&
            err != ESP_ERR_ESPNOW_EXIST) {
            return false;
        }
    }

    initialized = true;
    return true;
}

// Đọc thông tin thiết bị
bool getDeviceInfo(int deviceId, DeviceInfo& info) {
    int index = findDevice(deviceId);

    if (index < 0) {
        return false;
    }

    info = devices[index];
    return true;
}

#define ACK_TIMEOUT_MS 300
#define MAX_ATTEMPTS 3

// Thông tin lệnh đang chờ xác nhận
struct PendingCommand {
    uint32_t sequence = 0;
    bool desiredState = false;
    uint32_t lastSendTime = 0;
    uint8_t attempts = 0;
    bool waiting = false;
};

// Mỗi Slave có lệnh chờ riêng
static PendingCommand pending[DEVICE_COUNT];

// Sequence number của module
static uint32_t sequenceNumber = 0;

// Gửi một lần, dùng lại khi retry
static void sendPendingAttempt(int index) {
    PendingCommand& cmd = pending[index];

    char message[48];

    snprintf(
        message,
        sizeof(message),
        "SET_RELAY,%lu,%d",
        (unsigned long)cmd.sequence,
        cmd.desiredState ? 1 : 0
    );

    cmd.lastSendTime = millis();
    cmd.attempts++;

    esp_err_t err = esp_now_send(
        devices[index].mac,
        reinterpret_cast<const uint8_t*>(message),
        strlen(message) + 1
    );

    Serial.printf(
        "[ESP-NOW] Slave %d | Seq %lu | Attempt %u/%d | %s\n",
        devices[index].id,
        (unsigned long)cmd.sequence,
        (unsigned int)cmd.attempts,
        MAX_ATTEMPTS,
        err == ESP_OK ? "QUEUED" : "SEND ERROR"
    );
}

static void onReceive(
    const uint8_t* mac,
    const uint8_t* data,
    int len
) {
    if (!responseQueue || !mac || !data || len <= 0) {
        return;
    }

    Response response = {};

    memcpy(response.mac, mac, 6);

    int count = len < 64 ? len : 63;
    memcpy(response.message, data, count);
    response.message[count] = '\0';

    // Chuyển phản hồi sang queue, xử lý trong update()
    xQueueSend(responseQueue, &response, 0);
}

static void processResponses() {
    if (!responseQueue) return;

    Response response;

    while (xQueueReceive(
        responseQueue,
        &response,
        0
    ) == pdTRUE) {

        for (int i = 0; i < DEVICE_COUNT; i++) {

            // Kiểm tra đúng MAC của Slave
            if (memcmp(
                response.mac,
                devices[i].mac,
                6
            ) != 0) {
                continue;
            }

            unsigned long seq = 0;
            int state = -1;
            char extra = '\0';

            // Kiểm tra đúng định dạng STATE,seq,state
            if (sscanf(
                response.message,
                "STATE,%lu,%d%c",
                &seq,
                &state,
                &extra
            ) != 2 || (state != 0 && state != 1)) {
                break;
            }

            PendingCommand& cmd = pending[i];

            // Phải có lệnh đang chờ
            if (!cmd.waiting) {
                break;
            }

            // Kiểm tra sequence number
            if (seq != cmd.sequence) {
                break;
            }

            // Kiểm tra trạng thái yêu cầu
            if ((state == 1) != cmd.desiredState) {
                break;
            }

            // Phản hồi hợp lệ
            cmd.waiting = false;

            devices[i].state = (state == 1);
            devices[i].status = DeviceStatus::ONLINE;

            Serial.printf(
                "[ESP-NOW] Slave %d | SUCCESS | Seq %lu | %s | Attempts %u\n",
                devices[i].id,
                seq,
                state ? "ON" : "OFF",
                cmd.attempts
            );

            break;
        }
    }
}

// Hàm public để điều khiển thiết bị
bool sendCommand(int deviceId, bool on) {
    if (!initialized) {
        return false;
    }

    int index = findDevice(deviceId);

    if (index < 0) {
        return false;
    }

    PendingCommand& cmd = pending[index];

    // Không ghi đè lệnh chưa được xác nhận
    if (cmd.waiting) {
        return false;
    }

    sequenceNumber++;

    cmd.sequence = sequenceNumber;
    cmd.desiredState = on;
    cmd.attempts = 0;
    cmd.waiting = true;

    devices[index].status = DeviceStatus::PENDING;

    sendPendingAttempt(index);

    return true;
}

void update() {
    if (!initialized) {
        return;
    }

    // Ưu tiên xử lý ACK trước khi kiểm tra timeout
    processResponses();

    const uint32_t now = millis();

    for (int i = 0; i < DEVICE_COUNT; i++) {
        PendingCommand& cmd = pending[i];

        // Không có lệnh đang chờ
        if (!cmd.waiting) {
            continue;
        }

        // Chưa hết thời gian timeout
        if ((uint32_t)(now - cmd.lastSendTime)
            < ACK_TIMEOUT_MS) {
            continue;
        }

        // Còn lượt gửi: retry cùng sequence number
        if (cmd.attempts < MAX_ATTEMPTS) {
            Serial.printf(
                "[ESP-NOW] Slave %d | ACK TIMEOUT | Retrying...\n",
                devices[i].id
            );

            sendPendingAttempt(i);
        } else {
            // Đã thử đủ số lần nhưng chưa có ACK
            cmd.waiting = false;
            devices[i].status = DeviceStatus::OFFLINE;

            Serial.printf(
                "[ESP-NOW] Slave %d | OFFLINE | No ACK after %u attempts\n",
                devices[i].id,
                (unsigned int)cmd.attempts
            );
        }
    }
}

} // namespace EspNowManager