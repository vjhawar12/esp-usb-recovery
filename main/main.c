#include <stdio.h>
#include "tinyusb.h"
#include "device/usbd.h"
#include "tinyusb_cdc_acm.h"
#include "tusb.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"

#define TAG "ESP32-S3 Recovery & Diagnostics"
#define LOG(msg) ESP_LOGW(TAG, msg)
#define PID 0x050
#define RELEASE_NUM 0x0200
#define LANG_ENG 0x0409
#define CDC_INTERFACE_NUM 0
// single endpoint for vendor: 9 bytes interface descriptor + 7 bytes OUT endpoint descriptor 
#undef TUD_VENDOR_DESC_LEN
#define TUD_VENDOR_DESC_LEN 16
#define CONFIG_LEN (TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_CONFIG_DESC_LEN)
#define TUD_VENDOR_DESCRIPTOR_CUSTOM(_itfnum,_stridx,_epin,_epsize) \
9, TUSB_DESC_INTERFACE, _itfnum, 0, 1, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _stridx, \
7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(_epsize), 10

#define UART_TX 4
#define UART_RX 5
#define UART_CTS 6
#define UART_RTS 7
#define UART_DTR 8
#define UART_DSR 9
#define UART_NUM UART_NUM_2

#define MAX_RATE_HZ 60

typedef enum connection_status_t {
    DISCONNECTED,   
    MOUNTED,
    SUSPENDED,
    RESUMED
} connection_status_t;

typedef struct command_t {
    bool stream_on;
    uint8_t rate;
} command_t;

typedef struct payload_t {
    uint8_t data[64];
    size_t length;
} payload_t;

command_t cmd;
connection_status_t conn_status;
QueueHandle_t queue, uart_queue; // event queue for UART controller to transmit to CPU

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
EP1 In: interrupt; 
EP2 In: bulk
EP2 OUT: bulk

Vendor:
EP3 In: interrupt
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

void uart_init() {
    const int uart_buffer_size = (1024 * 2);
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
    const uart_port_t uart_num = UART_NUM;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // disable flow control pins for now, replace with UART_HW_FLOWCTRL_CTS_RTS later
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX, UART_RX, UART_RTS, UART_CTS, UART_DTR, UART_DSR));
}

// when linux computer sends data to esp via cdc interface
// handle things like STREAM ON/OFF, SET RATE, etc
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event) {
    payload_t payload;
    size_t num_bytes_read;
    if (itf == CDC_INTERFACE_NUM && event->type == CDC_EVENT_RX) {
        tinyusb_cdcacm_read(CDC_INTERFACE_NUM, payload.data, sizeof(payload.data) - 1, &num_bytes_read); 
        payload.data[num_bytes_read] = 0; 
        payload.length = num_bytes_read;  
        xQueueSend(queue, &payload, 0);
    } 
}   

void write_to_linux(const uint8_t* in_buffer) {
    size_t size = strnlen((const char*)in_buffer, 256); 
    tinyusb_cdcacm_write_queue(CDC_INTERFACE_NUM, in_buffer, size);
    esp_err_t err = tinyusb_cdcacm_write_flush(CDC_INTERFACE_NUM, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CDC ACM write flush error: %s", esp_err_to_name(err));
    }
}

void write_to_mcu(const char* string, size_t size) {
    uart_write_bytes(UART_NUM, string, strnlen(string, size));
}

/* 
STREAM_ON
STREAM_OFF
SET_RATE <hz>
GET_STATUS
PING
*/
void parse_command(void *pvParams) {
    payload_t payload;
    uint8_t out_buffer[64];
    uint8_t in_buffer[128];
    uint32_t rate;
    while (1) {
        xQueueReceive(queue, &payload, portMAX_DELAY);
        if (!strncmp((const char*)payload.data, "STREAM ON\r\n", 12)) {
            cmd.stream_on = true;
            write_to_mcu( "GET SENSOR DATA\r\n", 18);
        } else if (!strncmp((const char*)payload.data, "STREAM OFF\r\n", 13)) {
            cmd.stream_on = false;
            write_to_mcu("STOP SENSOR DATA\r\n", 18);
        } else if (sscanf((const char*)payload.data, "SET RATE %d\r\n", (int*)&rate) == 1) {
            if (rate < MAX_RATE_HZ) {
                cmd.rate = rate;
                snprintf((char*)out_buffer, sizeof(out_buffer), "SET RATE %d", (int)rate); 
                write_to_mcu((const char*)out_buffer, 12);
            } else {
                write_to_linux((const uint8_t*)"Rate too high\r\n");
            }
        } else if (!strncmp((const char*)payload.data, "GET STATUS\r\n", 13)) {
            write_to_mcu("GET STATUS\r\n", 13); 
            uart_read_bytes(UART_NUM, in_buffer, sizeof(in_buffer), pdMS_TO_TICKS(300)); 
        } else if (!strncmp((const char*)payload.data, "PING", 4)) {
            write_to_mcu("PING\r\n", 7); 
            // expected response: PONG  
            int bytes_read = uart_read_bytes(UART_NUM, in_buffer, sizeof(in_buffer), pdMS_TO_TICKS(300)); 
            if (bytes_read > 0) {
                write_to_linux(in_buffer);
            }
        } else {
            
        }
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
    queue = xQueueCreate(5, sizeof(payload_t));
}

void app_main(void) {
    freertos_init();
    usb_init();
    uart_init();
    register_callbacks();
    xTaskCreate(parse_command, "Parse Command Task", 512, NULL, 5, NULL); 
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}