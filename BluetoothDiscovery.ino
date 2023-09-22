/*
 * ESP32 code for Bluetooth testing (using Arduino IDE)
 * This code is designed to discover an A2DP bluetooth device, pair with it, and then connect. 
 * So far, this code is discovering devices. 
 * 
 * When working with the ESP-IDF (or similar SDKs), the steps to establish an SCO connection might look like this:
 * 
 * 1) Initialize the Bluetooth stack.
 * 2 Register necessary callbacks (for events like connection status changes).
 * 3) Start device discovery to find the target device.
 * 4) Pair with the target device.
 * 5) Once paired, establish an ACL connection.
 *      - ACL (Asynchronous Connection-Less): 
 *      - This is used for the transmission of data and control information. 
 *      - It is typically used for non-time-sensitive data transmission, such as file transfers or general data packets. 
 *      - ACL channels can operate in a point-to-multipoint configuration, meaning one master device can communicate with multiple slave devices.
 * 6) Request an SCO connection using the appropriate API.
 *      - SCO (Synchronous Connection-Oriented): 
 *      - This is primarily used for voice communication. 
 *      - SCO channels are time-bound and operate in a point-to-point configuration.
 */

#ifdef ARDUINO
  #include "Arduino.h"
  #include "esp_spi_flash.h" // deprecated. use spi_flash_mmap.h (For spi_flash_get_chip_size())
#elif defined(ESP_PLATFORM)
  #include "spi_flash_mmap.h"
#endif

extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/timers.h"
    #include "nvs.h"
    #include "nvs_flash.h"
    #include "esp_system.h"
    #include "esp_log.h"
    #include "esp_bt.h"
    #include "esp_bt_main.h"
    #include "esp_bt_device.h"
    #include "esp_gap_bt_api.h"
    #include "esp_a2dp_api.h"
    #include "esp_avrc_api.h"
    //======BLE Inlcudes
    #include "esp_gap_ble_api.h"
    #include "esp_bt_defs.h"
}

#include <map>
#include <vector>
#include <string.h>
#include <set>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>


#define MAX_RETRY_COUNT 3

bool BLE_SCAN_ON = false;

bool device_found = false;

bool isDiscoveryComplete = false;

// a lookup table for the Bluetooth earbuds we want to discover, pair with and connect to.
std::map<std::string, std::string> targetDeviceNames = {
    // {"74:74:46:ED:07:6B", "Pixel Buds A Series"},
    {"58:FC:C6:6C:0A:03", "TOZO Home"},
    {"54:B7:E5:8C:07:71", "TOZO Mazda"},
};

std::map<std::string, esp_bd_addr_t> discoveredDevices;       // BDA string -> Name

std::map<std::string, std::string> bleDiscoveredDevices;    // BLE discovered devices: BDA string -> Name

struct BleAdvertisementData {
    std::string deviceName = "";
    int8_t txPowerLevel = INT8_MIN; // Set default to minimum int8_t value to indicate "not set"
    uint16_t appearance = UINT16_MAX; // Use a default that's unlikely in real data to indicate "not set"
    std::vector<uint8_t> manufacturerData;
    std::vector<uint8_t> serviceUUIDs;
};

static TimerHandle_t scanTimer = NULL;
const int BLE_SCAN_DURATION = 10000; // milliseconds

// ============= function prototypes ===============
void commonInitialization(void);
void Initialize_Stack(void);
void Start_Discovery(void);
void app_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void a2d_sink_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
const char* getEspBtStatusDescription(esp_bt_status_t status);
void pair_with_device(const std::string& bdaStr, esp_bd_addr_t bda);
void pair_with_all_discovered_devices(void);
const char* gap_event_to_string(esp_bt_gap_cb_event_t event);
void initialize_once(void);
static void stop_ble_scan_callback(TimerHandle_t xTimer);
void ble_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void ble_scan_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
void start_ble_scan(void);
void compareAndReportMatches(void);
BleAdvertisementData extractBleAdvFields(const uint8_t* ble_adv, uint8_t adv_length);
void printBleAdvertisementData(const uint8_t* ble_adv, uint8_t adv_length, const std::string& address);
std::string mapToString(const std::map<std::string, std::string>& deviceMap);
void print_ESP32_info(void);
std::string bdaToString(const esp_bd_addr_t bda);
void setup(void);
void loop(void);
// ============= END function prototypes ===============

void Initialize_Stack(void) {
    const char* TAG = "INIT STACK";

    esp_err_t ret;

    // Initialize NVS — needed for BT settings storage
    nvs_flash_init();

    // Initialize the BT system
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret =esp_bt_controller_init(&bt_cfg);

    if (ret != ESP_OK) {        
        ESP_LOGE(TAG, "Initialize controller failed: %s", esp_err_to_name(ret));
        print_ESP32_info();
        return;
    }
    ESP_LOGI(TAG, "Success initializing BT controller");

    // Enable the bluetooth controllerin a certain mode. 
    // Mode choices are:
        // ESP_BT_MODE_IDLE: No Bluetooth functionality is enabled.
        // ESP_BT_MODE_BLE: Only BLE functionality is enabled.
        // ESP_BT_MODE_CLASSIC_BT: Only classic Bluetooth functionality is enabled.
        // ESP_BT_MODE_BTDM: Both BLE and classic Bluetooth are enabled.

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "BT Controller Enable FAILED with error: %s", esp_err_to_name(ret));
      print_ESP32_info();
      return;
    } 
    ESP_LOGI(TAG, "ESP_BT_MODE_BTDM SUCCEEDED");

    #ifdef ARDUINO
        ret = esp_bluedroid_init();     //deprecated in IDF. use esp_bluedroid_init_with_cfg()
    #elif defined(ESP_PLATFORM)
        esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
        ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    #endif

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Initialize bluedroid failed: %s", esp_err_to_name(ret));
        print_ESP32_info();
        return;
    } 
    ESP_LOGI(TAG, "Initialize bluedroid SUCCEEDED: %s", esp_err_to_name(ret));

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "Enable bluedroid failed: %s", esp_err_to_name(ret));
        print_ESP32_info();
        return;
    } 
    ESP_LOGI(TAG, "Enable bluedroid SUCCEEDED: %s", esp_err_to_name(ret));

    // Initialize A2DP sink
    if (esp_a2d_register_callback(&a2d_sink_callback) != ESP_OK) {
        ESP_LOGE(TAG, "%s register a2dp sink callback failed\n", __func__);
        return;
    }

    if (esp_a2d_sink_init() != ESP_OK) {
        ESP_LOGE(TAG, "%s a2dp sink initialize failed\n", __func__);
        return;
    }

    // Set ESP32 device name
    esp_bt_dev_set_device_name("ESP32_WROOM_32D");

}

void Start_Discovery(void) {
    // Register GAP callback
    ESP_LOGI("Start_Discovery", "Starting discovery...");
    esp_bt_gap_register_callback(app_gap_callback);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    //scan for 30 seconds with no limit on the number of devices discovered
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 30, 0);
}

void connect_to_source(esp_bd_addr_t remote_bda) {
    esp_a2d_sink_connect(remote_bda);
}

/*  
 * GAP stands for "Generic Access Profile." It's one of the core bluetooth protocols. 
 * GAP Handles:
 *  1) Discovery: GAP manages the discovery process, i.e., how devices advertise themselves and how they scan for other devices. This includes setting the device to be discoverable or non-discoverable, setting the device name, and defining the scan modes.
 *  2) Connection: Once two devices have discovered each other, GAP also handles how they establish a connection. This includes security features such as pairing, bonding, and the use of passkeys.
 *  3) Roles: GAP defines roles such as "Central" and "Peripheral." In the case of Bluetooth Low Energy (BLE), a Central device (like a smartphone) scans and connects to Peripheral devices (like a heart rate monitor).
 */
void app_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    // for debugging - print all events right before the switch statement
    ESP_LOGI("app_gap_callback", "...........................> Received GAP event: %s", gap_event_to_string(event));

    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT: {
            std::string bda_str = bdaToString(param->disc_res.bda);
            auto it = targetDeviceNames.find(bda_str);
            if (it != targetDeviceNames.end() && discoveredDevices.find(bda_str) == discoveredDevices.end()) {
                ESP_LOGI("app_gap_callback", "----------- Found target device: %s (BDA: %s)", it->second.c_str(), bda_str.c_str());
                memcpy(discoveredDevices[bda_str], param->disc_res.bda, sizeof(esp_bd_addr_t));
                esp_bt_gap_ssp_confirm_reply(param->disc_res.bda, true);
            }

            if (discoveredDevices.size() == targetDeviceNames.size()) {
                ESP_LOGI("app_gap_callback", "----------- Found all target devices. Stopping discovery...");
                esp_bt_gap_cancel_discovery();
                isDiscoveryComplete = true;
            }
            break;
        }

        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            std::string bda_str = bdaToString(param->auth_cmpl.bda);
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI("app_gap_callback", "Authentication success: %s", param->auth_cmpl.device_name);
                ESP_LOGI("app_gap_callback", "BDA: %s", bda_str.c_str());
            } else {
                ESP_LOGE("app_gap_callback", "Authentication failed, status: %d", param->auth_cmpl.stat);
                ESP_LOGE("app_gap_callback", "BDA: %s", bda_str.c_str());
            }
            break;
        }
        

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:{
            ESP_LOGI("app_gap_callback", "Discovery state changed.");
            break;
        }

        case ESP_BT_GAP_RMT_SRVCS_EVT: {
            std::string bda_str = bdaToString(param->rmt_srvcs.bda);
            ESP_LOGI("app_gap_callback", "Remote services for BDA: %s", bda_str.c_str());
            break; 
        }
       

        case ESP_BT_GAP_PIN_REQ_EVT: {
            std::string bda_str = bdaToString(param->pin_req.bda);
            ESP_LOGI("app_gap_callback", "PIN code request for BDA: %s", bda_str.c_str());
            const char* known_pin = "1234";
            esp_bt_pin_code_t pin_code;
            strncpy((char*)pin_code, known_pin, ESP_BT_PIN_CODE_LEN);
            esp_bt_gap_pin_reply(param->pin_req.bda, true, ESP_BT_PIN_CODE_LEN, pin_code);
            break;
        }
        
        case ESP_BT_GAP_CFM_REQ_EVT: {
            std::string bda_str = bdaToString(param->cfm_req.bda);
            ESP_LOGI("app_gap_callback", "Confirm request for passkey: %" PRIu32 ", for BDA: %s", param->cfm_req.num_val, bda_str.c_str());
            // ESP_LOGI("app_gap_callback", "Confirm request for passkey: %u, for BDA: %s", param->cfm_req.num_val, bda_str.c_str());
            if (param->cfm_req.num_val == 123456) {
                esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            }
            break;
        }

        case ESP_BT_GAP_KEY_NOTIF_EVT: {
            std::string bda_str = bdaToString(param->key_notif.bda);
            ESP_LOGI("app_gap_callback", "Passkey notification: %" PRIu32 ", for BDA: %s", param->key_notif.passkey, bda_str.c_str());
            break;
        }
    
        case ESP_BT_GAP_KEY_REQ_EVT: {
            std::string bda_str = bdaToString(param->key_req.bda);
            ESP_LOGI("app_gap_callback", "Passkey request for BDA: %s", bda_str.c_str());
            break;
        }

        case ESP_BT_GAP_READ_RSSI_DELTA_EVT: {
            std::string bda_str = bdaToString(param->read_rssi_delta.bda);
            ESP_LOGI("app_gap_callback", "RSSI delta event for BDA: %s", bda_str.c_str());
            break;
        }

        case ESP_BT_GAP_SET_AFH_CHANNELS_EVT:{
            ESP_LOGI("app_gap_callback", "Set AFH channels event triggered.");
            break;
        }

        case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:{
            ESP_LOGI("app_gap_callback", "Config EIR data event triggered.");
            break;
        }

        case ESP_BT_GAP_MODE_CHG_EVT: {
            std::string bda_str = bdaToString(param->mode_chg.bda);
            ESP_LOGI("app_gap_callback", "ESP_BT_GAP_MODE_CHG_EVT: Mode change event triggered.");
            ESP_LOGI("app_gap_callback", "Device BDA: %s", bda_str.c_str());
            ESP_LOGI("app_gap_callback", "New mode: %d", param->mode_chg.mode);
            break;
        }

        case ESP_BT_GAP_READ_REMOTE_NAME_EVT:{
            ESP_LOGI("app_gap_callback", "Received GAP event: ESP_BT_GAP_READ_REMOTE_NAME_EVT");
            ESP_LOGI("app_gap_callback", "Remote name read event triggered.");
            break;
        }

        case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT:{
            ESP_LOGI("app_gap_callback", "Received GAP event: ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT");
            ESP_LOGI("app_gap_callback", "Bonded device removal complete.");
            break;
        }

        case ESP_BT_GAP_QOS_CMPL_EVT:{
            ESP_LOGI("app_gap_callback", "Received GAP event: ESP_BT_GAP_QOS_CMPL_EVT");
            ESP_LOGI("app_gap_callback", "Quality of Service (QoS) complete.");
            break;
        }

        case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:{
            ESP_LOGI("app_gap_callback", "Received GAP event: ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT");

            // Checking only if 'param' is not NULL
            if (param) {
                std::string bda_string = bdaToString(param->acl_conn_cmpl_stat.bda);
                
                const char* device_name = "Unknown Device Name";
                // Look for the BDA in the map
                if (targetDeviceNames.count(bda_string)) {
                    device_name = targetDeviceNames[bda_string].c_str();
                }
                
                const char* statusDescription = getEspBtStatusDescription(param->acl_conn_cmpl_stat.stat);
                
                if (param->acl_conn_cmpl_stat.stat == ESP_BT_STATUS_SUCCESS) {
                    ESP_LOGI("app_gap_callback", "CONNECTION SUCCESS: connected to device: %s, %s", device_name, bda_string.c_str());
                } else {
                    ESP_LOGI("app_gap_callback", "CONNECTION FAILED: device: %s (%s), status: %s", device_name, bda_string.c_str(), statusDescription);
                }

            } else {
                ESP_LOGE("app_gap_callback", "Could not retrieve BDA from event.");
            }
            break;
        }

        case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:{
            ESP_LOGI("app_gap_callback", "Received GAP event: ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT");
            ESP_LOGI("app_gap_callback", "ACL disconnection complete status.");
            break;
        }

        default: {
            ESP_LOGW("app_gap_callback", "UNKNOWN_EVENT (Value: %d)", event);
        }
    }
}

void a2d_sink_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
    const char* TAG = "A2D SINK CALLBACK";

    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            ESP_LOGI(TAG, "A2DP Event: Connection State Changed");
            // Handle connection state changes here
            break;

        case ESP_A2D_AUDIO_STATE_EVT:
            ESP_LOGI(TAG, "A2DP Event: Audio Stream Transmission State Changed");
            // Handle audio stream transmission state changes here
            break;

        case ESP_A2D_AUDIO_CFG_EVT:
            ESP_LOGI(TAG, "A2DP Event: Audio Codec is Configured (A2DP SINK)");
            // Handle audio codec configuration (for A2DP SINK) here
            break;

        case ESP_A2D_MEDIA_CTRL_ACK_EVT:
            ESP_LOGI(TAG, "A2DP Event: Acknowledge for Media Control Commands");
            // Handle acknowledgment for media control commands here
            break;

        case ESP_A2D_PROF_STATE_EVT:
            ESP_LOGI(TAG, "A2DP Event: A2DP Init & Deinit Complete");
            // Handle A2DP initialization and deinitialization completion here
            break;

        // case ESP_A2D_SNK_PSC_CFG_EVT:
            // ESP_LOGI(TAG, "A2DP Event: Protocol Service Capabilities Configured (A2DP SINK)");
        //     // Handle protocol service capabilities configuration (for A2DP SINK) here
        //     break;

        // case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
            // ESP_LOGI(TAG, "A2DP Event: A2DP Sink Set Delay Report Value Complete");
        //     // Handle A2DP sink set delay report value completion here
        //     break;

        // case ESP_A2D_SNK_GET_DELAY_VALUE_EVT:
            // ESP_LOGI(TAG, "A2DP Event: A2DP Sink Get Delay Report Value Complete");
        //     // Handle A2DP sink get delay report value completion here
        //     break;

        // case ESP_A2D_REPORT_SNK_DELAY_VALUE_EVT:
            // ESP_LOGI(TAG, "A2DP Event: Report Delay Value (A2DP SRC)");
        //     // Handle delay value report (for A2DP SRC) here
        //     break;

        default:
            ESP_LOGI(TAG, "A2DP Event: Unhandled event %d", event);
            break;
    }
}

const char* getEspBtStatusDescription(esp_bt_status_t status) {
    switch (status) {
        case ESP_BT_STATUS_SUCCESS:       return "Success";
        case ESP_BT_STATUS_FAIL:          return "Fail";
        case ESP_BT_STATUS_NOT_READY:     return "Not Ready";
        case ESP_BT_STATUS_NOMEM:         return "No Memory";
        case ESP_BT_STATUS_BUSY:          return "Busy";
        case ESP_BT_STATUS_DONE:          return "Done";
        case ESP_BT_STATUS_UNSUPPORTED:   return "Unsupported";
        case ESP_BT_STATUS_PARM_INVALID:  return "Parameter Invalid";
        case ESP_BT_STATUS_UNHANDLED:     return "Unhandled";
        case ESP_BT_STATUS_AUTH_FAILURE:  return "Authentication Failure";
        case ESP_BT_STATUS_RMT_DEV_DOWN:  return "Remote Device Down";
        case ESP_BT_STATUS_AUTH_REJECTED: return "Authentication Rejected";
        case ESP_BT_STATUS_INVALID_STATIC_RAND_ADDR: return "Invalid Static Random Address";
        case ESP_BT_STATUS_PENDING:       return "Pending";
        case ESP_BT_STATUS_UNACCEPT_CONN_INTERVAL: return "Unaccepted Connection Interval";
        case ESP_BT_STATUS_PARAM_OUT_OF_RANGE: return "Parameter Out Of Range";
        case ESP_BT_STATUS_TIMEOUT:       return "Timeout";
        case ESP_BT_STATUS_PEER_LE_DATA_LEN_UNSUPPORTED: return "Peer LE Data Length Unsupported";
        case ESP_BT_STATUS_CONTROL_LE_DATA_LEN_UNSUPPORTED: return "Control LE Data Length Unsupported";
        case ESP_BT_STATUS_ERR_ILLEGAL_PARAMETER_FMT: return "Error Illegal Parameter Format";
        case ESP_BT_STATUS_MEMORY_FULL:   return "Memory Full";
        case ESP_BT_STATUS_EIR_TOO_LARGE: return "EIR Too Large";

        // For the HCI-related statuses
        case ESP_BT_STATUS_HCI_SUCCESS:   return "HCI Success (Host Controller Interface)";
        //... (Continue in this fashion for all the ESP_BT_STATUS_HCI_* error messages)
        case ESP_BT_STATUS_HCI_MAC_CONNECTION_FAILED: return "HCI MAC Connection Failed (Host Controller Interface)";

        default:
            static char unknownStatusMessage[50];
            snprintf(unknownStatusMessage, sizeof(unknownStatusMessage), "Unknown Status (Code: %d)", status);
            return unknownStatusMessage;
    }
}


//====================================== PAIRING AND CONNECTING ==============================

/* 
 * Attempts to pair with the BT device using SSP Just Works protocol. 
 * If that fails, it tries the Legacy pairing protocol. 
 */
void pair_with_device(const std::string& bdaStr, esp_bd_addr_t bda) {
    const char* TAG = "PAIR_WITH_DEVICE";

    // You can retrieve the device name from targetDeviceNames using bdaStr if needed
    auto it = targetDeviceNames.find(bdaStr);
    if (it != targetDeviceNames.end()) {
        ESP_LOGI(TAG, "Attempting to pair with device: Name: %s, BDA: %s", it->second.c_str(), bdaStr.c_str());
    } else {
        ESP_LOGI(TAG, "Attempting to pair with device: BDA: %s (Name not found in target devices)", bdaStr.c_str());
    }

    // First, try SSP "Just Works" pairing
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(uint8_t));

    // Try to connect, which should trigger pairing
    // Assuming you have a function like esp_a2d_sink_connect(bda) for A2DP
    ESP_LOGI(TAG, "about to call esp_a2d_sink_connect(bda)");
    esp_err_t sspResult = esp_a2d_sink_connect(bda); 

    if (sspResult == ESP_OK) {
        ESP_LOGI(TAG, "PAIRING SSP SUCCESS using SSP.");
        return;
    } else {
        ESP_LOGI(TAG, "PAIRING SSP FAILED. Trying Legacy Pairing...");

        // Setup for legacy pairing. Note: You may need to provide a PIN code for legacy pairing.
        esp_bt_pin_type_t pinType = ESP_BT_PIN_TYPE_FIXED;
        esp_bt_pin_code_t pinCode = { '1', '2', '3', '4' }; // This is an example PIN code; adjust as needed.
        esp_bt_gap_set_pin(pinType, sizeof(pinCode), pinCode);

        // Try to connect again, which should trigger legacy pairing
        esp_err_t legacyResult = esp_a2d_sink_connect(bda);

        if (legacyResult == ESP_OK) {
            ESP_LOGI(TAG, "Connection (and thus pairing) initiated using Legacy Pairing.");
        } else {
            ESP_LOGI(TAG, "Connection using Legacy Pairing also failed. Check device compatibility and distance.");
        }
    }
}

void pair_with_all_discovered_devices(void) {
    const char* TAG = "PAIR_ALL_DEVICES";

    for (auto& pair : discoveredDevices) {
        ESP_LOGI(TAG, "Attempting to pair with device: BDA: %s", pair.first.c_str());  // Logging the BDA string for clarity
        pair_with_device(pair.first, pair.second); // Here, pair.first is the BDA string and pair.second is the actual BDA (esp_bd_addr_t)
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2-second delay for clarity and to avoid too rapid consecutive actions
    }
    ESP_LOGI(TAG, "DONE Attempting to pair.");
}

//====================================== END PAIRING AND CONNECTING ==============================

//helper method for debugging -- converts numeric events types to readable strings
const char* gap_event_to_string(esp_bt_gap_cb_event_t event) {
    switch (event) {
       case ESP_BT_GAP_DISC_RES_EVT:
            return "ESP_BT_GAP_DISC_RES_EVT";
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            return "ESP_BT_GAP_AUTH_CMPL_EVT";
        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            return "ESP_BT_GAP_DISC_STATE_CHANGED_EVT";
        case ESP_BT_GAP_RMT_SRVCS_EVT:
            return "ESP_BT_GAP_RMT_SRVCS_EVT";
        case ESP_BT_GAP_RMT_SRVC_REC_EVT:
            return "ESP_BT_GAP_RMT_SRVC_REC_EVT";
        case ESP_BT_GAP_PIN_REQ_EVT:
            return "ESP_BT_GAP_PIN_REQ_EVT";
        case ESP_BT_GAP_CFM_REQ_EVT:
            return "ESP_BT_GAP_CFM_REQ_EVT";
        case ESP_BT_GAP_KEY_NOTIF_EVT:
            return "ESP_BT_GAP_KEY_NOTIF_EVT";
        case ESP_BT_GAP_KEY_REQ_EVT:
            return "ESP_BT_GAP_KEY_REQ_EVT";
        case ESP_BT_GAP_READ_RSSI_DELTA_EVT:
            return "ESP_BT_GAP_READ_RSSI_DELTA_EVT";
        case ESP_BT_GAP_SET_AFH_CHANNELS_EVT:
            return "ESP_BT_GAP_SET_AFH_CHANNELS_EVT";
        case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
            return "ESP_BT_GAP_CONFIG_EIR_DATA_EVT";
        case ESP_BT_GAP_MODE_CHG_EVT:
            return "ESP_BT_GAP_MODE_CHG_EVT";
        case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
            return "ESP_BT_GAP_READ_REMOTE_NAME_EVT - Remote name read event";

        case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT:
            return "ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT - Bonded device removal complete event";

        case ESP_BT_GAP_QOS_CMPL_EVT:
            return "ESP_BT_GAP_QOS_CMPL_EVT - Quality of Service (QoS) complete event";

        case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
            return "ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT - ACL connection complete status event";

        case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
            return "ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT - ACL disconnection complete status event";
                    
      default:
          const char* TAG = "gap_event_to_string()";
          static char unknownEventStr[50];
          snprintf(unknownEventStr, sizeof(unknownEventStr), "UNKNOWN_EVENT (Value: %d)", event);
          ESP_LOGW(TAG, "%s", unknownEventStr);  // Using for "warning" since an unknown event might be unexpected
          return unknownEventStr;
    }
}

void initialize_once(void) {
    const char* TAG = "INIT_ONCE";
    static bool initialized = false; // This will be set to true once the function runs.

    if (initialized) {
        ESP_LOGE(TAG, "Erroneous attempt to initialize more than once. Returning.");
        return; // Exit if initialization has already been done.
    }

    // Place your one-time initialization code here.
    Initialize_Stack();

    if (BLE_SCAN_ON){
      // Register BLE GAP callback, if you're keeping this for now.
      esp_err_t ret = esp_ble_gap_register_callback(ble_gap_callback);
      if (ret != ESP_OK) {
          ESP_LOGE(TAG, "Failed to register BLE GAP callback. Error: %d", ret);
      } else {
          ESP_LOGI(TAG, "Register BLE gap callback SUCCEEDED");
      }

      start_ble_scan();
    }

    // Mark initialization as done.
    initialized = true;
    ESP_LOGI(TAG, "Initialization complete.");
}



//====================================== START BLE CODE ==============================

static void stop_ble_scan_callback(TimerHandle_t xTimer) {
  esp_ble_gap_stop_scanning();

  if (scanTimer != NULL) {
    xTimerStop(scanTimer, 0);  // Stop the timer if it's still running
    xTimerDelete(scanTimer, 0);  // Delete the timer to free up the resources
    scanTimer = NULL;  // Reset the timer handle
  }
}

void ble_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    const char* TAG = "BLE_GAP_CB";

    switch(event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "BLE Scan parameters set complete.");
            break;

        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if(param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "BLE Scan started successfully.");
            } else {
                ESP_LOGE(TAG, "BLE Scan start failed.");
            }
            break;

        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {

                std::string bda_std_string = bdaToString(param->scan_rst.bda);

                if (bleDiscoveredDevices.find(bda_std_string) == bleDiscoveredDevices.end()) {
                    // New device discovered
                    bleDiscoveredDevices[bda_std_string] = bda_std_string;

                    // Print the advertisement data using the utility function
                    printBleAdvertisementData(param->scan_rst.ble_adv, param->scan_rst.adv_data_len, bda_std_string);
                    
                    // Additional logic for target devices if needed
                    // ...
                }
            }
            break;

        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            // Assuming this is when the scanning completes. Adjust as necessary.
            if (param->scan_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "BLE scan completed successfully");
            } else {
                ESP_LOGE(TAG, "BLE scan failed to stop scanning");
            }
            for (const auto& entry : bleDiscoveredDevices) {
                ESP_LOGI(TAG, "Found BLE device. BDA: %s, Name: %s", entry.first.c_str(), entry.second.c_str());
            }
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            ESP_LOGI(TAG, "BLE Advertise start complete.");
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(TAG, "BLE Connection parameters updated.");
            break;

        // Add cases for other events if needed.

        default:
            ESP_LOGW(TAG, "Unhandled BLE GAP event %d.", event);
            break;
    }
}

void ble_scan_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    const char* TAG = "BLE_SCAN_CB";

    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        // Start scanning when scan parameters are set
        esp_ble_gap_start_scanning(10);  // scan for 10 seconds
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if(param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "BLE Scan started successfully.");
        } else {
            ESP_LOGE(TAG, "BLE Scan start failed.");
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {

            std::string bda_std_string = bdaToString(param->scan_rst.bda);

            if (bleDiscoveredDevices.find(bda_std_string) == bleDiscoveredDevices.end()) {
                // New device discovered
                bleDiscoveredDevices[bda_std_string] = bda_std_string;  // Store the BDA string as both key and value

                ESP_LOGI(TAG, "Found BLE device. Address: %s, Name: %s", bda_std_string.c_str(), param->scan_rst.ble_adv[2] == '\0' ? "Unknown" : (char *) &param->scan_rst.ble_adv[2]);
                
                // Check if this is one of the target devices
                if (targetDeviceNames.find(bda_std_string) != targetDeviceNames.end()) {
                    ESP_LOGI(TAG, "---> This is a target device: %s", targetDeviceNames[bda_std_string].c_str());
                }
            }
        }
        break;

    default:
        ESP_LOGW(TAG, "Unhandled BLE Scan event %d.", event);
        break;
    }
}

void start_ble_scan(void) {
    const char* TAG = "BLE_SCAN_START";

    // Create the ble scan timer which stops BLE scanning after BLE_SCAN_DURATION
    if (scanTimer == NULL) {
        scanTimer = xTimerCreate("scanTimer", pdMS_TO_TICKS(BLE_SCAN_DURATION), pdFALSE, (void *)0, stop_ble_scan_callback);
        if (scanTimer == NULL) {
            ESP_LOGE(TAG, "Failed to create BLE scan timer. BLE scanning aborted");
            return;
        }
    }

    // Start the timer and check for any errors
    if (xTimerStart(scanTimer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start BLE scan timer. BLE scanning aborted");
        return;
    }

    esp_ble_gap_register_callback(ble_scan_callback);
    
    static esp_ble_scan_params_t ble_scan_params = {
        .scan_type              = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval          = 0x50,
        .scan_window            = 0x30,
        .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE  // Using the correct enum value
    };

    esp_ble_gap_set_scan_params(&ble_scan_params);
}


void compareAndReportMatches(void) {
    const char* TAG = "MATCH_REPORT";

    ESP_LOGI(TAG, "Comparing BLE discovered devices with target BT devices...");
    bool matchesFound = false;

    for (const auto& bleDevice : bleDiscoveredDevices) {
        if (targetDeviceNames.find(bleDevice.first) != targetDeviceNames.end()) {
            matchesFound = true;
            ESP_LOGI(TAG, "Match found: BDA: %s, Name (from BT): %s, Name (from BLE): %s", bleDevice.first.c_str(), targetDeviceNames[bleDevice.first].c_str(), bleDevice.second.c_str());
        }
    }

    if (!matchesFound) {
        ESP_LOGW(TAG, "BLE Discovery Found NO Devices that match the Target Devices");
    }
}

BleAdvertisementData extractBleAdvFields(const uint8_t* ble_adv, uint8_t adv_length) {
    const char* TAG = "BLE ADVERTISEMENT DATA";

    BleAdvertisementData advData;
    uint8_t index = 0;

    while (index < adv_length) {
        if (index + 1 > adv_length) break;  // Boundary check for length byte

        uint8_t length = ble_adv[index++];
        if (length == 0 || index + length > adv_length) break; // End of advertisement data or boundary check

        uint8_t type = ble_adv[index];

        switch(type) {
            case 0x09: // Complete Local Name
                advData.deviceName.assign(reinterpret_cast<const char*>(ble_adv + index + 1), length - 1);
                break;
            case 0x19: // Appearance
                if (length >= 3) {
                    advData.appearance = (ble_adv[index + 2] << 8) | ble_adv[index + 1];
                } else {
                    ESP_LOGW(TAG, "Appearance data length is less than expected.");
                }
                break;
            case 0x0A: // Tx Power Level
                advData.txPowerLevel = static_cast<int8_t>(ble_adv[index + 1]);
                break;
            case 0xFF: // Manufacturer Specific Data
                advData.manufacturerData.assign(ble_adv + index + 1, ble_adv + index + 1 + length - 1);
                break;
            case 0x02: // Incomplete List of 16-bit Service Class UUIDs
            case 0x03: // Complete List of 16-bit Service Class UUIDs
                // You can expand on this as needed
                advData.serviceUUIDs.assign(ble_adv + index + 1, ble_adv + index + 1 + length - 1);
                break;
            // ... Add more cases as needed
            default:
                ESP_LOGI(TAG, "Unhandled BLE advertisement data type: %d", type);
                break;
        }
        
        index += length;
    }

    return advData;
}

void printBleAdvertisementData(const uint8_t* ble_adv, uint8_t adv_length, const std::string& address) {
    const char* TAG = "PRINT BLE ADV DATA";

    BleAdvertisementData advData = extractBleAdvFields(ble_adv, adv_length);

    ESP_LOGI(TAG, "Found BLE device. Address: %s", address.c_str());

    // Print device name if available
    ESP_LOGI(TAG, "Device name length: %d", advData.deviceName.length());
    if (!advData.deviceName.empty()) {
        ESP_LOGI(TAG, "Device Name: %s", advData.deviceName.c_str());
    } else {
        ESP_LOGI(TAG, "Device Name: Unknown");
    }

    // Print Appearance if available
    ESP_LOGI(TAG, "Appearance: %d", advData.appearance);
    if (advData.appearance != UINT16_MAX) {
        ESP_LOGI(TAG, "Valid Appearance: %d", advData.appearance);
    }

    // Print Tx Power Level if available
    ESP_LOGI(TAG, "Tx Power Level: %d dBm", advData.txPowerLevel);
    if (advData.txPowerLevel != INT8_MIN) {
        ESP_LOGI(TAG, "Valid Tx Power Level: %d dBm", advData.txPowerLevel);
    }

    // Print Manufacturer Specific Data if available
    if (!advData.manufacturerData.empty()) {
        std::string manufacturerDataStr = "Manufacturer Data: ";
        for (uint8_t byte : advData.manufacturerData) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", byte);
            manufacturerDataStr += buf;
        }
        ESP_LOGI(TAG, "%s", manufacturerDataStr.c_str());
    }

    // Print Service UUIDs if available
    if (!advData.serviceUUIDs.empty()) {
        std::string serviceUUIDsStr = "Service UUIDs: ";
        for (uint8_t byte : advData.serviceUUIDs) {
            char buf[4];
            snprintf(buf, sizeof(buf), "%02X ", byte);
            serviceUUIDsStr += buf;
        }
        ESP_LOGI(TAG, "%s", serviceUUIDsStr.c_str());
    }
}


//====================================== END BLE CODE ==============================


//====================================== UTILITIES ==============================

std::string mapToString(const std::map<std::string, std::string>& deviceMap) {
    std::stringstream ss;
    
    for (const auto& entry : deviceMap) {
        ss << "Key: " << entry.first << ", Value: " << entry.second << "\n";
    }
    
    return ss.str();
}


void print_ESP32_info(void) {
    const char* TAG = "ESP32 INFO";
    // Print out general ESP32 information
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    
    // esp_chip_info_t chip_info;
    // esp_chip_info(&chip_info);
    // ESP_LOGI(TAG, "Chip Model: %d", chip_info.model);
    // ESP_LOGI(TAG, "Chip Cores: %d", chip_info.cores);
    // ESP_LOGI(TAG, "Chip Revision: %d", chip_info.revision);
    
    // ESP_LOGI(TAG, "Flash Chip Size: %u bytes", spi_flash_get_chip_size());

    // Print Bluetooth Controller Information
    esp_bt_controller_status_t status = esp_bt_controller_get_status();
    switch (status) {
        case ESP_BT_CONTROLLER_STATUS_IDLE:
            ESP_LOGI(TAG, "BT Controller Status: IDLE");
            break;
        case ESP_BT_CONTROLLER_STATUS_INITED:
            ESP_LOGI(TAG, "BT Controller Status: INITED");
            break;
        case ESP_BT_CONTROLLER_STATUS_ENABLED:
            ESP_LOGI(TAG, "BT Controller Status: ENABLED");
            break;
        case ESP_BT_CONTROLLER_STATUS_NUM:
            ESP_LOGI(TAG, "BT Controller Status: UNKNOWN");
            break;
    }
}


// Convert esp_bd_addr_t (Bluetooth Device Address) to its string representation
std::string bdaToString(const esp_bd_addr_t bda) {
    char bda_str[18];
    sprintf(bda_str, "%02X:%02X:%02X:%02X:%02X:%02X", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return std::string(bda_str);
}

//====================================== END UTILITIES ==============================

void commonInitialization(void){
        const char* TAG = "APP_MAIN";

        // Ensure one-time initialization
        initialize_once();

        // Start Discovery once
        Start_Discovery();

        // Main loop
        while(1) {
            if (isDiscoveryComplete) {
                ESP_LOGI(TAG, "Discovery complete. Initiating pairing...");
                pair_with_all_discovered_devices();
                isDiscoveryComplete = false;  
            }
            vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for a second before checking again
        }

        compareAndReportMatches();
}

#ifdef ARDUINO
  void setup() {
      commonInitialization();
  }

  void loop() {
      // Arduino-specific continuous operation

  }
#elif defined(ESP_PLATFORM)
  //wrapped in extern, because this is a .cpp file
  extern "C" {
      void app_main(void) {
          commonInitialization();
      }
  }
#endif
