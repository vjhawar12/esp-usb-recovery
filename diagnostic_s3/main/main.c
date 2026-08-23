#include <stdio.h>
#include "tinyusb.h"
#include "device/usbd.h"
#include "tinyusb_cdc_acm.h"
#include "tusb.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "tusb_config.h"

#define TAG "Diagnostic ESP32-S3"
#define LOG(msg) ESP_LOGW(TAG, msg)
#define PID 0x050
#define RELEASE_NUM 0x0200
#define LANG_ENG 0x0409
#define CDC_INTERFACE_NUM 0
#define VENDOR_INTERFACE_NUM 2
// single endpoint for vendor: 9 bytes interface descriptor + 7 bytes OUT endpoint descriptor 
#undef TUD_VENDOR_DESC_LEN
#define TUD_VENDOR_DESC_LEN 23
#define CONFIG_LEN (TUD_CDC_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_CONFIG_DESC_LEN)

#define EP3_IN 0x83
#define EP3_OUT 0x03

// vendor interface with 2 endpoints, EP3 Bulk Out and EP3 Bulk In
#define TUD_VENDOR_DESCRIPTOR_CUSTOM(_itfnum,_stridx,_epin,_epout,_epsize) \
9, TUSB_DESC_INTERFACE, _itfnum, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, _stridx, \
7, TUSB_DESC_ENDPOINT, _epin, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 10, \
7, TUSB_DESC_ENDPOINT, _epout, TUSB_XFER_BULK, U16_TO_U8S_LE(_epsize), 10

#define UART_TX 4
#define UART_RX 5
#define UART_CTS 6
#define UART_RTS 7
#define UART_DTR 8
#define UART_DSR 9
#define UART_NUM UART_NUM_2

#define MAX_RATE_HZ 60
#define GET_SENSOR_DATA_CMD "GET SENSOR DATA\r\n"
#define PING_CMD "PING\r\n"
#define GET_STATUS_CMD "GET STATUS\r\n"
#define GET_RESET_REASON "GET RESET REASON\r\n"

#define VENDOR_MAX_BUFFER_SIZE 64
#define CDC_MAX_BUFFER_SIZE 64

typedef enum connection_status_t {
    DISCONNECTED,   
    MOUNTED,
    SUSPENDED,
    RESUMED
} connection_status_t;

typedef enum payload_type_t {
    VENDOR,
    CDC
} payload_type_t;

typedef enum vendor_err_t {
    OK,
    TX_FULL,
} vendor_err_t;

typedef struct payload_t {
    union {
        uint8_t cdc_data[CDC_MAX_BUFFER_SIZE + 1];
        uint8_t vendor_data[VENDOR_MAX_BUFFER_SIZE + 1];
    } buffer;
    payload_type_t type;
    size_t length;
} payload_t;

typedef enum mcu_interface_err_t {
    OK,
    IDENTIFY,
    GET_STATUS,
    CAPTURE_STATE,
    GET_FAULT_CONTEXT,
    DIAGNOSE,
    VERIFY_FIRMWARE,
    RESET_TARGET,
    ENTER_RECOVERY,
    RECOVER_TARGET,
    GENERATE_REPORT
} mcu_interface_err_t;

connection_status_t conn_status;
QueueHandle_t cdc_queue, vendor_queue, uart_queue; // event queue for UART controller to transmit to CPU

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

// defines interface numbers, endpoints, etc
uint8_t full_speed_config[] = {
    // config num = 1; num interfaces = 3
    TUD_CONFIG_DESCRIPTOR(1, 3, 4, CONFIG_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(CDC_INTERFACE_NUM, 5, 0x81, 8, 0x02, 0x82, 64),
    TUD_VENDOR_DESCRIPTOR_CUSTOM(VENDOR_INTERFACE_NUM, 6, EP3_IN, EP3_OUT, 64),
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
    //  initialize the entire USB subsystem on the chip.
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
// sends live logs to linux 
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event) {
    if (itf == CDC_INTERFACE_NUM && event->type == CDC_EVENT_RX) {
        payload_t payload;
        size_t num_bytes_read;
        payload.type = CDC;
        tinyusb_cdcacm_read(CDC_INTERFACE_NUM, payload.buffer.cdc_data, sizeof(payload.buffer.cdc_data), &num_bytes_read); 
        payload.buffer.cdc_data[num_bytes_read] = 0; 
        payload.length = num_bytes_read;  
        xQueueSend(cdc_queue, &payload, 0);
    } 
}   

// vendor equivalent of cdc rx callback
/* 
handles
IDENTIFY
GET_STATUS
CAPTURE_STATE
GET_FAULT_CONTEXT
DIAGNOSE
VERIFY_FIRMWARE
RESET_TARGET
ENTER_RECOVERY
RECOVER_TARGET
GENERATE_REPORT
*/
void tud_vendor_rx_cb(uint8_t idx, const uint8_t *buffer, uint32_t bufsize) {
    if (idx == 0) {
        payload_t payload;
        payload.type = VENDOR;
        memcpy(payload.buffer.vendor_data, buffer, bufsize * sizeof(uint8_t));
        payload.buffer.vendor_data[bufsize] = 0; 
        payload.length = bufsize;  
        xQueueSend(vendor_queue, &payload, 0);
    }
}

esp_err_t cdc_write_to_linux(const uint8_t* in_buffer, size_t size) {
    tinyusb_cdcacm_write_queue(CDC_INTERFACE_NUM, in_buffer, size);
    esp_err_t err = tinyusb_cdcacm_write_flush(CDC_INTERFACE_NUM, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CDC ACM write flush error: %s", esp_err_to_name(err));
    }
    return err;
}

vendor_err_t vendor_write_to_linux(const char* buffer) {
    uint32_t bytes_available = tud_vendor_n_write_available(0);
    size_t size = strlen(buffer);
    if (bytes_available >= size) {
        tud_vendor_n_write(0, buffer, size); 
        uint32_t bytes_sent = tud_vendor_write_flush();
        uint8_t out_buffer[64];
        snprintf(out_buffer, "[DATA_SENT] Bytes sent: %lu\r\n", bytes_sent);
        cdc_write_to_linux(out_buffer, strlen(out_buffer));
        return OK;
    } 
    return TX_FULL;
}   

int uart_write_to_mcu(const char* string, size_t size) {
    return uart_write_bytes(UART_NUM, string, strnlen(string, size));
}

int uart_read_from_mcu(uint8_t* buffer, uint8_t max_length, uint8_t max_time_ms) {
    int bytes_read = uart_read_bytes(UART_NUM, buffer, max_length, pdMS_TO_TICKS(max_time_ms)); 
    if (bytes_read == -1) {
        LOG("Error: Timeout");
    } else if (bytes_read == 0) {
        LOG("Error: Reading from empty buffer");
    } 
    return bytes_read;
}

mcu_interface_err_t handle_identify() {
    return vendor_write_to_linux("HANDLE IDENTIFY PLACEHOLDER\r\n") != ESP_OK? IDENTIFY : OK;
}

mcu_interface_err_t handle_get_status() {
    return vendor_write_to_linux("HANDLE GET STATUS PLACEHOLDER\r\n") != ESP_OK? GET_STATUS : OK;
}

mcu_interface_err_t handle_capture_state() {
    return vendor_write_to_linux("HANDLE CAPUTURE STATE PLACEHOLDER\r\n") != ESP_OK? CAPTURE_STATE : OK;
}

mcu_interface_err_t handle_get_fault_context() {
    return vendor_write_to_linux("HANDLE GET FAULT CONTEXT\r\n") != ESP_OK? GET_FAULT_CONTEXT : OK;
}

mcu_interface_err_t handle_diagnose() {
    return vendor_write_to_linux("HANDLE DIAGNOSE PLACEHOLDER\r\n") != ESP_OK? DIAGNOSE : OK;
}   

mcu_interface_err_t handle_verify_firmware() {
    return vendor_write_to_linux("HANDLE VERIFY FIRMWARE PLACEHOLDER\r\n") != ESP_OK? VERIFY_FIRMWARE : OK;
}

mcu_interface_err_t handle_reset_target() {
    return vendor_write_to_linux("HANDLE RESET TARGET PLACEHOLDER\r\n") != ESP_OK? RESET_TARGET : OK;
}

mcu_interface_err_t handle_enter_recovery() {
    return vendor_write_to_linux("HANDLE ENTER RECOVERY PLACEHOLDER\r\n") != ESP_OK? ENTER_RECOVERY : OK;
}

mcu_interface_err_t handle_recover_target() {
    return vendor_write_to_linux("HANDLE RECOVER TARGET PLACEHOLDER\r\n") != ESP_OK? RECOVER_TARGET : OK;
}

mcu_interface_err_t handle_generate_report() {
    return vendor_write_to_linux("HANDLE GENERATE REPORT PLACEHOLDER\r\n") != ESP_OK? GENERATE_REPORT : OK;
}

/*  
+--------------------+-------------------------------------------------------------------+------------------------------------+
| Command            | Technician is asking:                                             | Intrusiveness                      |
+--------------------+-------------------------------------------------------------------+------------------------------------+
| PING               | “Is my diagnostic S3 alive?”                                      | None                               |
| IDENTIFY           | “What target did I connect to?”                                   | None                               |
| GET_STATUS         | “What condition is it in right now?”                              | None                               |
| CAPTURE_STATE      | “Preserve volatile evidence before we touch anything.”            | Low                                |
| GET_FAULT_CONTEXT  | “What evidence exists about why it failed?”                       | Low                                |
| DIAGNOSE           | “Put all the evidence together and classify the failure.”         | Low                                |
| VERIFY_FIRMWARE    | “Does the installed image look intact/valid?”                     | Low                                |
| RESET_TARGET       | “Try a simple hardware reset.”                                    | Destructive to volatile state      |
| ENTER_RECOVERY     | “Bypass the application and enter ROM bootloader/debug recovery.” | Higher                             |
| RECOVER_TARGET     | “Automatically perform the appropriate recovery procedure.”       | Potentially destructive            |
| GENERATE_REPORT    | “Explain what happened and what was done.”                        | None                               |
+--------------------+-------------------------------------------------------------------+------------------------------------+
*/
void parse_vendor_commands(void *pvParams) {
    payload_t payload;
    mcu_interface_err_t err;
    while (1) {
        xQueueReceive(vendor_queue, &payload, portMAX_DELAY);
        if (!strcmp((const char*)payload.buffer.vendor_data, "PING\r\n")) {
            write_to_linux((const uint8_t*)"PONG\r\n", 7);
        } else if (!strcmp((const char*)payload.buffer.vendor_data, "IDENTIFY\r\n")) {
            err = handle_identify();
        } else if (!strcmp((const char*)payload.buffer.vendor_data, "GET STATUS\r\n"))  {
            err = handle_get_status();
        } else if (!strcmp((const char*)payload.buffer.vendor_data, "CAPTURE STATE\r\n")) {
            err = handle_capture_state();
        } else if (!strcmp((const char*)payload.buffer.vendor_data, "GET FAULT CONTEXT\r\n")) { 
            err = handle_get_fault_context();
        } else if (!strcmp((const char*)payload.buffer.vendor_data, "DIAGNOSE\r\n")) { 
            err = handle_diagnose();
        } else if (!strcmp((const char*)payload.buffer.vendor_data, "VERIFY FIRMWARE\r\n")) { 
            err = handle_verify_firmware();
        } else if (!strcmp((const char*)payload.buffer.vendor_data, "RESET TARGET\r\n")) {
            err = handle_reset_target();
        } else if (!strcmp((const char*)payload.buffer.vendor_data, "ENTER RECOVERY\r\n")) { 
            err = handle_enter_recovery();
        } else if (!strcmp((const char*)payload.buffer.vendor_data, "RECOVER TARGET\r\n")) { 
            err = handle_recover_target();
        } else if (!strcmp((const char*)payload.buffer.vendor_data, "GENERATE REPORT\r\n")) { 
            err = handle_generate_report();
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
    cdc_queue = xQueueCreate(5, sizeof(payload_t));
    vendor_queue = xQueueCreate(5, sizeof(payload_t));
}

void app_main(void) {
    freertos_init();
    usb_init();
    uart_init();
    register_callbacks();
    xTaskCreate(parse_vendor_commands, "Parse Vendor Commands Task", 512, NULL, 5, NULL);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}