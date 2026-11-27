#include <Zigbee.h>
#include "zdo/esp_zigbee_zdo_command.h"

static ZigbeeSwitch light(1);
static uint8_t phase;
static unsigned long next_action;

static void leave_response(esp_zb_zdp_status_t status, void *) {
    Serial.printf("LEAVE_RESPONSE status=%d\n", int(status));
}

void setup() {
    Serial.begin(115200);
    light.setManufacturerAndModel("M5Stack", "LeaveProbe");
    Zigbee.addEndpoint(&light);
    Zigbee.setPrimaryChannelMask(1u << 15);
    Zigbee.setRebootOpenNetwork(180);
    if (!Zigbee.begin(ZIGBEE_COORDINATOR)) Serial.println("COORD_START_FAILED");
}

void loop() {
    delay(50);
    if (millis() < next_action || !light.bound()) return;
    if (phase == 0) {
        light.lightOn();
        next_action = millis() + 1500;
    } else if (phase == 1) {
        esp_zb_zdo_mgmt_leave_req_param_t request = {};
        esp_zb_lock_acquire(portMAX_DELAY);
        auto devices = light.getBoundDevices();
        if (devices.empty()) {
            esp_zb_lock_release();
            return;
        }
        auto *device = devices.front();
        memcpy(request.device_address, device->ieee_addr, sizeof(request.device_address));
        request.dst_nwk_addr = device->short_addr;
        request.rejoin = 1;
        esp_zb_zdo_device_leave_req(&request, leave_response, nullptr);
        esp_zb_lock_release();
        Serial.println("LEAVE_REQUEST rejoin=1");
        next_action = millis() + 10000;
    } else if (phase == 2) {
        light.lightOff();
    } else {
        return;
    }
    ++phase;
}
