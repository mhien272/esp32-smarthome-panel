
#pragma once

#include <Arduino.h>

namespace EspNowManager {

enum class DeviceStatus {
    IDLE,
    PENDING,
    ONLINE,
    OFFLINE
};

struct DeviceInfo {
    int id;
    uint8_t mac[6];
    DeviceStatus status;
    bool state;
};

// Khởi tạo ESP-NOW
bool begin();

// Gửi lệnh bật/tắt thiết bị
bool sendCommand(int deviceId, bool on);

// Xử lý phản hồi, timeout và retry
void update();

// Lấy thông tin trạng thái thiết bị
bool getDeviceInfo(int deviceId, DeviceInfo& info);

}
