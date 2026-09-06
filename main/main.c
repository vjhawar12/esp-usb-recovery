#include <stdio.h>
#include "tinyusb.h"
#include "device/usbd.h"
#include "tinyusb_cdc_acm.h"
#include "tusb.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "device/usbd_pvt.h"
#include "device/dcd.h"  

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

#define VENDOR_BULK_IN 0x83
#define VENDOR_BULK_OUT 0x03
#define CDC_NOTIFICATION_IN 0x81
#define CDC_DATA_OUT 0x02
#define CDC_DATA_IN 0x82

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

#define VENDOR_MAX_BUFFER_SIZE 2048
#define MAX_IN_PACKET_SIZE 64
#define MAX_OUT_PACKET_SIZE 64
#define CDC_MAX_BUFFER_SIZE 64

#define VENDOR_INTERFACE_CLASS 0xFF
#define VENDOR_INTERFACE_SUBCLASS 0
#define VENDOR_INTERFACE_PROTOCOL 0

#define BUFFER_POOL_NUM 5

void recovery_vendor_init(void);
bool recovery_vendor_deinit(void);
void recovery_vendor_reset(uint8_t rhport);
uint16_t recovery_vendor_open(uint8_t rhport, const tusb_desc_interface_t *desc_itf, uint16_t max_len);
bool recovery_vendor_control_completed_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request);
bool recovery_vendor_data_completed_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);

// tracks USB connection state for logging
typedef enum connection_status_t {
    DISCONNECTED, // physical usb link is absent    
    CONNECTED, // physical usb link detected, but kernel has not acknowledged yet
    ENUMERATED, // connected and kernel has enumerated (acknowledged) the device a.k.a device mounted
    SUSPENDED, // host has put bus in low power state / cable has temporarily lost host communication
} connection_status_t;

// tracks the type of data being submitted during a USB transaction (a data transfer across a single endpoint)
typedef enum payload_type_t {
    VENDOR,
    CDC
} payload_type_t;

typedef enum vendor_err_t {
    VENDOR_ERR_OK,
    VENDOR_ERR_USB_DESC,
    VENDOR_ERR_PACKET_SIZE_EXCEEDED,
    VENDOR_TX_FULL,
    VENDOR_TX_DCD_ERR,
    VENDOR_RX_DCD_ERR,
    VENDOR_INVALID_CMD,
    VENDOR_NO_BUFFER_FREE,
} vendor_err_t;

typedef enum cdc_err_t {
    CDC_ERR_OK,
    CDC_TX_FULL,
    CDC_ERR_UNKNOWN
} cdc_err_t;

typedef enum mcu_interface_err_t {
    MCU_INTERFACE_ERR_OK,
    IDENTIFY,
    GET_STATUS,
    CAPTURE_STATE,
    GET_FAULT_CONTEXT,
    DIAGNOSE,
    VERIFY_FIRMWARE,
    RESET_TARGET,
    ENTER_RECOVERY,
    RECOVER_TARGET,
    GENERATE_REPORT,
    MCU_INTERFACE_INVALID_CMD
} mcu_interface_err_t;

typedef struct {
  uint8_t itf_num; // interface number
  uint8_t ep_addr_out, ep_addr_in; // addresses of endpoints
  uint16_t max_in_packet_size, max_out_packet_size; // max packet sizes for each out/in packet
  uint8_t rhport; // root hub usb controller port (should be 0 for single usb controller chips)
  bool tx_done; // set when a single low-level USB transaction (device -> host) is completed 
  bool rx_done; // set when a single low-level USB transaction (host -> device) is completed
} recovery_vendor_interface_t; 


/* 
USB RX Path

arm free pool buffer
        ↓
Linux sends OUT transfer
        ↓
USB hardware fills buffer
        ↓
xfer callback
        ↓
queue pointer to completed buffer
        ↓
arm another free buffer
        ↓
command task receives pointer
        ↓
process command
        ↓
release buffer (occupied = false)

the below enums track state of inidividual buffers (buffer_state_t) and the rx engine (rx_state_t)
based on that we can compare expected vs actual state at different points and trigger error handlers 
*/

typedef enum buffer_state_t {
    BUFFER_FREE,
    BUFFER_ARMED,
    BUFFER_QUEUED,
    BUFFERED_PROCESSING,
} buffer_state_t;

typedef enum rx_state_t {
    RX_IDLE,
    RX_ARMED,
    RX_NO_BUFFER_FREE
} rx_state_t; 

rx_state_t rx_state = RX_IDLE;

CFG_TUSB_MEM_ALIGN
static uint8_t in_buff[VENDOR_MAX_BUFFER_SIZE] = {0};  // points to the physical usb packet from device -> host
CFG_TUSB_MEM_ALIGN
typedef struct vendor_payload_t { // buffer + metadata like occupied and length
    buffer_state_t state;
    size_t length;
    uint8_t buff[VENDOR_MAX_BUFFER_SIZE];
} vendor_payload_t;

/* 
Buffers are static so pointers to them persist even once the callback returns, this lets us queue a 4 byte pointer rather than
a potentially very large struct object. 

A buffer pool is used to keep track of which buffer tinyUSB is using so that I don't modify the buffer while TinyUSB is, as that would 
corrupt the USB data.

A buffer is marked occupied when handed to TinyUSB and remains occupied while its pointer is queued/processed by the command task.
 
Ownership:
FREE -> USB RX -> vendor_queue -> command parser -> FREE

Rx (defined as host -> device) needs a buffer pool because we're processing messages from the host in this code and handing the message off
to another task, and a potential issue could be one buffer is being filled up with data while another message arrived from the host and the same buffer
is being reused, hence the buffer pool so a buffer that is free could be used for the incoming data

The TX side is simpler because we're just sending a message to the host and processing it host-side. We only send the next transmission once the current
has finished so we have control over the overlap, unlike RX where we don't know when the host will send and thus need the buffer pool.
*/

static vendor_payload_t out_buff[BUFFER_POOL_NUM] = {0};  // buffer pool, each buffer points to the physical usb packet from host -> device
static recovery_vendor_interface_t vendor_interface; // metadata + done flags for vendor interface 

CFG_TUSB_MEM_ALIGN
typedef struct cdc_payload_t {
     uint8_t buff[CDC_MAX_BUFFER_SIZE];
     size_t length;
} cdc_payload_t;


// defines the callbacks on init, open, reset, etc
static usbd_class_driver_t const recovery_vendor_driver = {
    .name             = "VENDOR",
    .init             = recovery_vendor_init, // called once at system boot to initialize variables
    .deinit           = recovery_vendor_deinit, // called when stack tear down (deep sleep, reboot, etc)
    .reset            = recovery_vendor_reset, // called when USB bus reset triggered
    .open             = recovery_vendor_open, // called once during USB enumeration to arm the DMA transfer
    .control_xfer_cb  = recovery_vendor_control_completed_cb, // called when vendor specific control transfer arrives on EP 0
    .xfer_cb          = recovery_vendor_data_completed_cb, // called everytime a data transfer (bulk/interrupt) completes 
    .xfer_isr         = NULL,
    .sof              = NULL
}; 

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
    TUD_VENDOR_DESCRIPTOR_CUSTOM(VENDOR_INTERFACE_NUM, 6, VENDOR_BULK_IN, VENDOR_BULK_OUT, 64),
}; 

connection_status_t conn_status;
QueueHandle_t cdc_queue, vendor_queue, uart_queue; // event queue for UART controller to transmit to CPU
TaskHandle_t my_task_handle; 

/* 
 ==============================================================================
 USB HARDWARE ENDPOINT LAYOUT & LANE ALLOCATION
 ==============================================================================
 Note: Each physical Endpoint (EP) index contains two independent hardware lanes:
   - OUT Lane (Host -> Device / RX) : Address = 0x0X (Bit 7 = 0)
   - IN  Lane (Device -> Host / TX) : Address = 0x8X (Bit 7 = 1)
 ------------------------------------------------------------------------------

 [EP0] - Default Control Endpoint (Shared Bidirectional Pipe)
   ├─ OUT Lane (0x00) : Control Engine (SETUP & OUT Data Stage)
   └─ IN  Lane (0x80) : Control Engine (IN Data & STATUS Stage)

 ------------------------------------------------------------------------------
 [EP1] - CDC Management / Notification Interface
   ├─ OUT Lane (0x01) : Unused / Unassigned in Hardware
   └─ IN  Lane (0x81) : Interrupt EP1 IN FIFO (Device -> Host Notifications)

 ------------------------------------------------------------------------------
 [EP2] - CDC Data Interface
   ├─ OUT Lane (0x02) : Bulk EP2 OUT Hardware FIFO (Host -> Device RX Data)
   └─ IN  Lane (0x82) : Bulk EP2 IN Hardware FIFO  (Device -> Host TX Data)

 ------------------------------------------------------------------------------
 [EP3] - Vendor Custom Class InteDIAGNOSTIC ESP32-S3]rface
   ├─ OUT Lane (0x03) : Bulk EP3 OUT Hardware FIFO (Host -> Device RX Stream)
   └─ IN  Lane (0x83) : Bulk EP3 IN Hardware FIFO  (Device -> Host TX Stream)

 ==============================================================================
 HARDWARE SUMMARY FOR CONSUMED LANES:
   - EP0 : 1 Bidirectional Pipe (0x00 / 0x80) (itf 0)
   - EP1 : 1 IN Lane (0x81) (itf 1)
   - EP2 : 1 OUT Lane (0x02) + 1 IN Lane (0x82) (itf 1)
   - EP3 : 1 OUT Lane (0x03) + 1 IN Lane (0x83) (itf 2)
 ==============================================================================
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

static int buff_count = 0;

static void recovery_vendor_init(void) {
    memset((&vendor_interface), 0, (sizeof(vendor_interface)));
}

static bool recovery_vendor_deinit(void) {
    memset(in_buff, 0, sizeof(in_buff));
    memset(out_buff, 0, sizeof(out_buff)); 
    return true;
}

static void recovery_vendor_reset(uint8_t rhport) {
    (void) rhport;
    memset((&vendor_interface), 0, (sizeof(vendor_interface)));
    memset(in_buff, 0, sizeof(in_buff));
    memset(out_buff, 0, sizeof(out_buff)); 
}

// prepares the USB host to receive #vendor_interface.max_out_packet_size of data from device (rx transaction) 
// Find an unowned RX buffer before arming the OUT endpoint.
// Never reuse an occupied buffer because the command task may still be
// reading it through a queued pointer.
vendor_err_t arm_rx(uint8_t rhport) {
    for (int tries = 0; tries < BUFFER_POOL_NUM; tries++) {
        buff_count = (buff_count + 1) % BUFFER_POOL_NUM;
        if (out_buff[buff_count].state == BUFFER_FREE) {
            if (!usbd_edpt_xfer(vendor_interface.rhport, vendor_interface.ep_addr_out, out_buff[buff_count].buff, vendor_interface.max_out_packet_size)) {
                rx_state = RX_IDLE;
                return VENDOR_RX_DCD_ERR;
            }
            out_buff[buff_count].state = BUFFER_ARMED; // buffer is sending data via the queue so this buffer cannot be reused until the command parser confirms it receives data
            rx_state = RX_ARMED;
            return VENDOR_ERR_OK;
        }
    }
    rx_state = RX_NO_BUFFER_FREE;
    return VENDOR_NO_BUFFER_FREE; 
}

// prepares the USB host to send #total_bytes bytes of data to device (tx transaction)
vendor_err_t arm_tx(uint8_t rhport, uint16_t total_bytes) {
    TU_ASSERT(usbd_edpt_xfer(vendor_interface.rhport, vendor_interface.ep_addr_in, in_buff, total_bytes), VENDOR_TX_DCD_ERR);
    return VENDOR_ERR_OK;
}

void handle_dcd_error() {

}

void handle_max_retries_exceeded() {

}

/* 
Claim the interface
Identify/validate Recovery v1
Walk the descriptors belonging to that interface
Find the endpoints
Validate the endpoint topology
Open the endpoints in TinyUSB
Save the live interface state
Arm the OUT endpoint
Return how many descriptor bytes were consumed
*/
uint16_t recovery_vendor_open(uint8_t rhport, const tusb_desc_interface_t *desc_itf, uint16_t max_len) {
    // validate interface class, subclass, protocol
    TU_ASSERT(desc_itf->bInterfaceClass == VENDOR_INTERFACE_CLASS, VENDOR_ERR_USB_DESC); 
    TU_ASSERT(desc_itf->bInterfaceSubClass == VENDOR_INTERFACE_SUBCLASS, VENDOR_ERR_USB_DESC); 
    TU_ASSERT(desc_itf->bInterfaceProtocol == VENDOR_INTERFACE_PROTOCOL, VENDOR_ERR_USB_DESC);
    // validate interface topology: 1 interface with 1 bulk in, 1 bulk out, no alternate settings
    TU_ASSERT(desc_itf->bNumEndpoints == 2, VENDOR_ERR_USB_DESC); 
    TU_ASSERT(desc_itf->bAlternateSetting == 0, VENDOR_ERR_USB_DESC); 
    const uint8_t* desc_end = (const uint8_t*)desc_itf + max_len; // end of descriptor region available to this open() call
    const uint8_t* p_desc = tu_desc_next(desc_itf); // point to endpoint descriptor
    int bulk_endpoint_in_count = 0;
    int bulk_endpoint_out_count = 0;
    const tusb_desc_endpoint_t* desc_ep;
    const tusb_desc_endpoint_t* desc_in_ep;
    const tusb_desc_endpoint_t* desc_out_ep; 
    uint16_t in_size = 0;
    uint16_t out_size = 0;
    while (tu_desc_in_bounds(p_desc, desc_end)) { // looping per descriptor (interface, endpoint 1, endpoint 2)
        const uint8_t desc_type = tu_desc_type(p_desc); // get descriptor type
        desc_ep = (const tusb_desc_endpoint_t*) p_desc;
        if (desc_type == TUSB_DESC_INTERFACE || desc_type == TUSB_DESC_INTERFACE_ASSOCIATION) {
            break; // end of this interface
        } else if (desc_type == TUSB_DESC_ENDPOINT) {
            if (tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN) {
                bulk_endpoint_in_count++;
                TU_ASSERT(bulk_endpoint_in_count == 1, VENDOR_ERR_USB_DESC);
                desc_in_ep = desc_ep; 
                TU_ASSERT(desc_in_ep->bmAttributes.xfer == TUSB_XFER_BULK, VENDOR_ERR_USB_DESC);
                in_size = tu_edpt_packet_size(desc_in_ep);
                TU_ASSERT(in_size <= MAX_IN_PACKET_SIZE, VENDOR_ERR_PACKET_SIZE_EXCEEDED);
            } else {
                bulk_endpoint_out_count++;
                TU_ASSERT(bulk_endpoint_out_count == 1, VENDOR_ERR_USB_DESC);
                desc_out_ep = desc_ep;
                TU_ASSERT(desc_out_ep->bmAttributes.xfer == TUSB_XFER_BULK, VENDOR_ERR_USB_DESC);
                out_size = tu_edpt_packet_size(desc_out_ep);
                TU_ASSERT(out_size <= MAX_OUT_PACKET_SIZE, VENDOR_ERR_PACKET_SIZE_EXCEEDED);
            }
        }
        p_desc = tu_desc_next(p_desc);
    }
    TU_ASSERT(bulk_endpoint_in_count == 1, VENDOR_ERR_USB_DESC);
    TU_ASSERT(bulk_endpoint_out_count == 1, VENDOR_ERR_USB_DESC);
    vendor_interface.itf_num = desc_itf->bInterfaceNumber;
    /* 
        each usb controller would have 1-2 buses (for 3.0 its 2 buses) and each bus has a root hub attached to it 
        i.e a rhport. individual devices attach on a roothub port

        rhport variable identifies a physical root hub

        an rhport is fixed throughout the duration of the firmware, assuming you don't unplug and replug the device into
        a different port while the application is running 
    */
    vendor_interface.rhport = rhport;
    vendor_interface.ep_addr_in  = desc_in_ep->bEndpointAddress;
    vendor_interface.max_in_packet_size = in_size;
    vendor_interface.ep_addr_out = desc_out_ep->bEndpointAddress;
    vendor_interface.max_out_packet_size = out_size;
    TU_ASSERT(usbd_edpt_open(vendor_interface.rhport, desc_in_ep), VENDOR_ERR_USB_DESC);
    TU_ASSERT(usbd_edpt_open(vendor_interface.rhport, desc_out_ep), VENDOR_ERR_USB_DESC);
    // arming the USB peripheral so it can accept incoming packets
    vendor_err_t armed = arm_rx(vendor_interface.rhport);
    switch (armed) {
        case VENDOR_ERR_OK:
            break;
        case VENDOR_NO_BUFFER_FREE:
            break;
        case VENDOR_RX_DCD_ERR:
            handle_dcd_error();
            break;
        default:
            break;
    }
    return (uint16_t)((uintptr_t)p_desc - (uintptr_t)desc_itf);
}

bool recovery_vendor_control_completed_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
    (void) rhport; (void) stage; (void) request;
    return false;
}

// runs when data (interrupt/bulk) transfer finishes on an endpoint. This could be OUT/RX or IN/TX.  
bool recovery_vendor_data_completed_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
    if (result != XFER_RESULT_SUCCESS) {
        return false;
    }
    TU_VERIFY(ep_addr == vendor_interface.ep_addr_in || ep_addr == vendor_interface.ep_addr_out, 0);
    if (ep_addr == vendor_interface.ep_addr_in) {
        // mark TX transfer done
        vendor_interface.tx_done = true;
        xTaskNotifyGive(my_task_handle); 
    } else {
        // mark RX transfer done
        vendor_interface.rx_done = true;
        if (xferred_bytes > VENDOR_MAX_BUFFER_SIZE - 1) return false;
        // queue a pointer to a vendor_payload_t which has the buffer rather than the full object
        vendor_payload_t *payload = &out_buff[buff_count]; // passing in address of the thing we want to send
        payload->length = xferred_bytes;
        payload->buff[xferred_bytes] = 0;
        // Queue only the pointer value, not the 2 KB vendor_payload_t.
        // out_buff[] is static, so the pointed-to object remains valid after this
        // callback returns. The parser releases the buffer by clearing occupied.
        TU_VERIFY(xQueueSend(vendor_queue, &payload, 0) == pdTRUE);
        payload->state = BUFFER_QUEUED;
        // rearming the USB peripheral so it can accept the next packet
        vendor_err_t armed = arm_rx(vendor_interface.rhport);
        switch (armed) {
            case VENDOR_ERR_OK:
                break;
            case VENDOR_NO_BUFFER_FREE:
                break;
            case VENDOR_RX_DCD_ERR:
                handle_dcd_error();
                break;
            default:
                break;
        }
    }
    return true;
}

usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count) {
    *driver_count = 1;
    return &recovery_vendor_driver;
}

void tud_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr) {
    switch (eventid) {
        case DCD_EVENT_UNPLUGGED:
            conn_status = DISCONNECTED;
            break;
        case DCD_EVENT_BUS_RESET:
            conn_status = CONNECTED;
            break;
        case DCD_EVENT_SUSPEND:
            conn_status = SUSPENDED;
            break;
    }
}

/* 
ensure tinyusb.c has the following: 
__attribute__((weak)) void custom_tud_mount_cb() {} 
and tud_mount_cb calls custom_tud_mount_cb at the end
*/

void custom_tud_mount_cb() {
    conn_status = ENUMERATED;
}

static void usb_init() {
    const tinyusb_config_t config = {
        .descriptor = descriptor,
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .phy = phy,
        .event_cb = NULL,
        .event_arg = NULL
    };
    //  initialize the entire USB subsystem on the chip.
    ESP_ERROR_CHECK(tinyusb_driver_install(&config)); 

    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = NULL, 
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
}

static void uart_init() {
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

cdc_err_t cdc_write_string(char* buffer) {
    size_t size = strlen(buffer);
    uint32_t bytes_available = tud_cdc_write_available(); 
    if (bytes_available < size) {
        return CDC_TX_FULL;
    }
    size_t bytes_written = tinyusb_cdcacm_write_queue(CDC_INTERFACE_NUM, (uint8_t*)buffer, size);
    tinyusb_cdcacm_write_flush(CDC_INTERFACE_NUM, 0);
    if (bytes_written != size) {
        return CDC_ERR_UNKNOWN;
    }
    return CDC_ERR_OK;
}   

cdc_err_t cdc_write_bytes(uint8_t* buffer, size_t size) {
    uint32_t bytes_available = tud_cdc_write_available(); 
    if (bytes_available < size) {
        return CDC_TX_FULL;
    }
    size_t bytes_written = tinyusb_cdcacm_write_queue(CDC_INTERFACE_NUM, (uint8_t*)buffer, size);
    if (bytes_written != size) {
        return CDC_ERR_UNKNOWN;
    }
    return CDC_ERR_OK;
}

// diagnostic ESP32-S3 (device) -> Linux host
// clear tx_done flag, fill buffer, arm hardware, wait for tx_done flag to be set by callback
// to write a string a cast can be used
vendor_err_t vendor_write(uint8_t* buffer, uint32_t ms) {
     // claim endpoint before submiting transfer
    if (!usbd_edpt_claim(0, VENDOR_BULK_IN)) {
        return VENDOR_TX_FULL;
    }
    // clearing tx_done flag
    vendor_interface.tx_done = false;
    // filling buffer
    size_t size = sizeof(buffer);
    memcpy(in_buff, buffer, size);
    // arming hardware
    vendor_err_t armed = arm_tx(0, size);
    switch (armed) {
        case VENDOR_ERR_OK:
            break;
        case VENDOR_TX_DCD_ERR:
            handle_dcd_error();
            break;
        default:
            break;
    }
    // waiting for tx_done to be set
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms)); 
    // logging via CDC
    char in_buffer[64];
    snprintf(in_buffer, sizeof(in_buffer), "[DATA_SENT] %u Bytes sent\r\n", size);
    cdc_write_string(in_buffer);   
    return VENDOR_ERR_OK;
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
    return vendor_write((uint8_t*)"HANDLE IDENTIFY PLACEHOLDER\r\n", pdMS_TO_TICKS(30)) != VENDOR_ERR_OK? IDENTIFY : MCU_INTERFACE_ERR_OK;
}

mcu_interface_err_t handle_get_status() {
    return vendor_write((uint8_t*)"HANDLE GET STATUS PLACEHOLDER\r\n", pdMS_TO_TICKS(30)) != VENDOR_ERR_OK? GET_STATUS : MCU_INTERFACE_ERR_OK;
}

mcu_interface_err_t handle_capture_state() {
    return vendor_write((uint8_t*)"HANDLE CAPTURE STATE PLACEHOLDER\r\n", pdMS_TO_TICKS(30)) != VENDOR_ERR_OK? CAPTURE_STATE : MCU_INTERFACE_ERR_OK;
}

mcu_interface_err_t handle_get_fault_context() {
    return vendor_write((uint8_t*)"HANDLE GET FAULT CONTEXT\r\n", pdMS_TO_TICKS(30)) != VENDOR_ERR_OK? GET_FAULT_CONTEXT : MCU_INTERFACE_ERR_OK;
}

mcu_interface_err_t handle_diagnose() {
    return vendor_write((uint8_t*)"HANDLE DIAGNOSE PLACEHOLDER\r\n", pdMS_TO_TICKS(30)) != VENDOR_ERR_OK? DIAGNOSE : MCU_INTERFACE_ERR_OK;
}   

mcu_interface_err_t handle_verify_firmware() {
    return vendor_write((uint8_t*)"HANDLE VERIFY FIRMWARE PLACEHOLDER\r\n", pdMS_TO_TICKS(30)) != VENDOR_ERR_OK? VERIFY_FIRMWARE : MCU_INTERFACE_ERR_OK;
}

mcu_interface_err_t handle_reset_target() {
    return vendor_write((uint8_t*)"HANDLE RESET TARGET PLACEHOLDER\r\n", pdMS_TO_TICKS(30)) != VENDOR_ERR_OK? RESET_TARGET : MCU_INTERFACE_ERR_OK;
}

mcu_interface_err_t handle_enter_recovery() {
    return vendor_write((uint8_t*)"HANDLE ENTER RECOVERY PLACEHOLDER\r\n", pdMS_TO_TICKS(30)) != VENDOR_ERR_OK? ENTER_RECOVERY : MCU_INTERFACE_ERR_OK;
}

mcu_interface_err_t handle_recover_target() {
    return vendor_write((uint8_t*)"HANDLE RECOVER TARGET PLACEHOLDER\r\n", pdMS_TO_TICKS(30)) != VENDOR_ERR_OK? RECOVER_TARGET : MCU_INTERFACE_ERR_OK;
}

mcu_interface_err_t handle_generate_report() {
    return vendor_write((uint8_t*)"HANDLE GENERATE REPORT PLACEHOLDER\r\n", pdMS_TO_TICKS(30)) != VENDOR_ERR_OK? GENERATE_REPORT : MCU_INTERFACE_ERR_OK;
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
static void parse_vendor_commands(void *pvParams) {
    vendor_payload_t *payload;
    mcu_interface_err_t err;
    while (1) {
        xQueueReceive(vendor_queue, &payload, portMAX_DELAY);
        payload->state = BUFFERED_PROCESSING;
        if (!strcmp((const char*)payload->buff, "PING\r\n")) {
            cdc_write_string(TAG "PONG\r\n");
        } else if (!strcmp((const char*)payload->buff, "IDENTIFY\r\n")) {
            err = handle_identify();
        } else if (!strcmp((const char*)payload->buff, "GET STATUS\r\n"))  {
            err = handle_get_status();
        } else if (!strcmp((const char*)payload->buff, "CAPTURE STATE\r\n")) {
            err = handle_capture_state();
        } else if (!strcmp((const char*)payload->buff, "GET FAULT CONTEXT\r\n")) { 
            err = handle_get_fault_context();
        } else if (!strcmp((const char*)payload->buff, "DIAGNOSE\r\n")) { 
            err = handle_diagnose();
        } else if (!strcmp((const char*)payload->buff, "VERIFY FIRMWARE\r\n")) { 
            err = handle_verify_firmware();
        } else if (!strcmp((const char*)payload->buff, "RESET TARGET\r\n")) {
            err = handle_reset_target();
        } else if (!strcmp((const char*)payload->buff, "ENTER RECOVERY\r\n")) { 
            err = handle_enter_recovery();
        } else if (!strcmp((const char*)payload->buff, "RECOVER TARGET\r\n")) { 
            err = handle_recover_target();
        } else if (!strcmp((const char*)payload->buff, "GENERATE REPORT\r\n")) { 
            err = handle_generate_report();
        } else {
            err = MCU_INTERFACE_INVALID_CMD;
        }
        if (err != MCU_INTERFACE_ERR_OK) {

        }
        // Command processing is complete; USB may now reuse this pool buffer.
	    payload->state = BUFFER_FREE;
        if (RX_NO_BUFFER_FREE) {
            arm_rx(vendor_interface.rhport);
        }
    }       
}

static void freertos_init() {
    cdc_queue = xQueueCreate(5, sizeof(cdc_payload_t)); 
    // Queue stores vendor_payload_t pointers only; payload bytes remain in out_buff[].
    vendor_queue = xQueueCreate(5, sizeof(vendor_payload_t*)); // number of bytes to send: sizeof(vendor_payload_t*) bytes
}

void app_main(void) {
    freertos_init();
    usb_init();
    uart_init();
    xTaskCreate(parse_vendor_commands, "Parse Vendor Commands Task", 512, NULL, 5, &my_task_handle);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
