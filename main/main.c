#include <stdio.h>
#include "tinyusb.h"
#include "device/usbd.h"
#include "tusb_cdc_acm.h"
#include "tusb.h"
#include "esp_err.h"
#include "esp_log.h"

#define TAG "ESP32-S3 Recovery & Diagnostics"
#define LOG(msg) ESP_LOGW(TAG, msg)
#define PID 0x050
#define RELEASE_NUM 0x0200
#define LANG_ENG 0x0409
#define CDC_INTERFACE_NUM 0
// single endpoint for vendor: 9 bytes interface descriptor + 7 bytes OUT endpoint descriptor 
#define TUD_VENDOR_DESC_LEN 16
#define CONFIG_LEN (TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_CONFIG_DESC_LEN)
#define TUD_VENDOR_DESCRIPTOR_CUSTOM(_itfnum,_stridx,_epin,_epsize) \
9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _stridx, \
7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), 10

typedef enum connection_status_t {
    DISCONNECTED,   
    MOUNTED,
    SUSPENDED,
    RESUMED
} connection_status_t;

connection_status_t conn_status;
QueueHandle_t queue;

const char* string_desc[] = {
    (const char[]) { 0x09, 0x04 }, // english only
    "Taylor Systems Engineering",
    "Secondary Module Recovery Device",
    "123", // change serial number later
    "CDC and Vendor configuration",
    "CDC",
    "Vendor"
};

const tusb_desc_device_t device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC, // composite device
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .idVendor = 0x303A,
    .idProduct = PID,
    .bcdDevice = RELEASE_NUM,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t full_speed_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, 3, 4, CONFIG_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(CDC_INTERFACE_NUM, 5, 0x81, 8, 0x02, 0x82, 64),
    TUD_VENDOR_DESCRIPTOR_CUSTOM(2, 6, 0x83, 64),
}; 

/* 
endpoint 0: device control endpoint
CDC: requires 3 endpoint lanes: notification, data in, data out
endpoint 1 CDC notification: interrupt; 
endpoint 2 DATA in: bulk
endpoint 2 DATA out: bulk

endpoint 3 critical messages in: interrupt
*/

const tinyusb_desc_config_t descriptor = {
    .device = &device,
    .string = string_desc,
    .qualifier = NULL, // ESP32 S3 only runs at full speed, not high speed so qualifier is omitted
    .full_speed_config = full_speed_config,
    .high_speed_config = NULL
};

// using internal PHY on ESP32-S3
const tinyusb_phy_config_t phy = {
    .skip_setup = false,
    .self_powered = false,
}; 

uint8_t data[64];

// high level hook -- not critical for operation just keeping track of state
void event_cb(tinyusb_event_t *event, void *arg) {
    switch (event->id) {
        case TINYUSB_EVENT_ATTACHED:
            LOG("Attached: device connected to host; config complete");
            conn_status = MOUNTED;
            break;
        case TINYUSB_EVENT_DETACHED:
            LOG("Detached: device unplugged / disconnected from host");
            conn_status = DISCONNECTED;
            break;
        case TINYUSB_EVENT_SUSPENDED:
            LOG("Suspended: host suspended the bus");
            conn_status = SUSPENDED;
            break;
        case TINYUSB_EVENT_RESUMED:
            LOG("Resumed: host resumed the bus");
            conn_status = RESUMED;
            break;
    }
} 


void usb_init() {
    const tinyusb_config_t config = {
        .descriptor = descriptor,
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .phy = phy,
        .event_cb = event_cb,
        .event_arg = NULL
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&config)); 
}

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event) {
    uint8_t data[64];
    size_t num_bytes_read;
   if (itf == CDC_INTERFACE_NUM && event->type == CDC_EVENT_RX) {
        tinyusb_cdcacm_read(CDC_INTERFACE_NUM, data, sizeof(data), &num_bytes_read); 
        if (num_bytes_read < 64) {
            data[num_bytes_read] = 0;   
        }
        xQueueSend(queue, data, 0);
    } 
}   

void register_callbacks() {
    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = tinyusb_cdc_rx_callback, 
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
}

void freertos_init() {
    queue = xQueueCreate(5, sizeof(data));
}

void app_main(void) {
    freertos_init();
    usb_init();
    register_callbacks();
    while (1) {
        xQueueReceive(queue, data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}