/*
 * ESP32 code for Bluetooth testing (using Arduino IDE)
 * This code is designed to discover an A2DP bluetooth device, pair with it, and then connect. 
 * So far, this code is discovering devices. 
 */

#include <Arduino.h>

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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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


#define TAG "ESP32_PAIRING"
#define TARGET_DEVICE_NAME "TargetDevice"
bool device_found = false;

bool isDiscoveryComplete = false;

// a lookup table for the Bluetooth earbuds we want to discover, pair with and connect to.
std::map<std::string, std::string> targetDeviceNames = {
    // {"74:74:46:ED:07:6B", "Pixel Buds A Series"},
    {"58:FC:C6:6C:0A:03", "TOZO Home"},
    {"54:B7:E5:8C:07:71", "TOZO Mazda"}
    //REMINDER: no trailing comma
};

std::map<std::string, std::string> discoveredDevices;  // Other discovered devices (if any): BDA -> Name

std::map<std::string, std::string> bleDiscoveredDevices;  // BLE discovered devices: BDA -> Name

std::map<std::string, std::string> bleDeviceNames;


struct BleAdvertisementData {
    std::string deviceName = "";
    int8_t txPowerLevel = INT8_MIN; // Set default to minimum int8_t value to indicate "not set"
    uint16_t appearance = UINT16_MAX; // Use a default that's unlikely in real data to indicate "not set"
    std::vector<uint8_t> manufacturerData;
    std::vector<uint8_t> serviceUUIDs;
};

void Initialize_Stack() {
    esp_err_t ret;

    // Start Serial communication for debugging
    Serial.begin(115200);

    // Initialize NVS — needed for BT settings storage
    nvs_flash_init();

    // Initialize the BT system
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        Serial.printf("Initialize_Stack initialize controller failed: %s\n", esp_err_to_name(ret));
        print_ESP32_info();
        return;
    }
    Serial.println("Success initializing BT controller");

    //enable bluetooth controller (options are: ESP_BT_MODE_IDLE, ESP_BT_MODE_BLE, ESP_BT_MODE_CLASSIC_BT, ESP_BT_MODE_BTDM), note ESP_BT_MODE_BTDM means both BT and BLE
    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret != ESP_OK) {
      Serial.printf("BT Controller Enable FAILED with error: %s\n", esp_err_to_name(ret));
      print_ESP32_info();
      return;
    } 
    Serial.println("ESP_BT_MODE_BTDM SUCCEEDED");

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        Serial.printf("Initialize_Stack initialize bluedroid failed: %s\n", esp_err_to_name(ret));
        print_ESP32_info();
        return;
    } 
    Serial.printf("Initialize_Stack initialize bluedroid SUCCEEDED: %s\n", esp_err_to_name(ret));

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        Serial.printf("Initialize_Stack enable bluedroid failed: %s\n", esp_err_to_name(ret));
        print_ESP32_info();
        return;
    } 
    Serial.printf("Initialize_Stack enable bluedroid SUCCEEDED: %s\n", esp_err_to_name(ret));

    //register A2DP callback:
    esp_a2d_register_callback(a2dp_callback);
    
    // Set device name
    esp_bt_dev_set_device_name("ESP32_Device");
}

// connect to an A2DP device
void connect_a2dp(esp_bd_addr_t remote_bda) {
    esp_a2d_sink_connect(remote_bda);
}

void Start_Discovery() {
    // Register GAP callback
    Serial.println("Starting discovery...");
    esp_bt_gap_register_callback(app_gap_callback);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 30, 0);
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
    Serial.printf(".....................................................................> Received GAP event: %s\n", gap_event_to_string(event));

    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT: {
            std::string bda_str = bdaToString(param->disc_res.bda);

            // Print every discovered device for debugging
            // Serial.printf("Discovered device BDA: %s\n", bda_str.c_str());

            // Now we'll check to see if its one of the devices we want to connect to
            auto it = targetDeviceNames.find(bda_str);
            if (it != targetDeviceNames.end() && discoveredDevices.find(bda_str) == discoveredDevices.end()) {
                Serial.printf("------------------ Found target device: %s (BDA: %s)\n", it->second.c_str(), bda_str.c_str());

                // Store the BDA and its logical name in the discoveredDevices map
                discoveredDevices[bda_str] = it->second;
            }

            // If we've discovered all devices in our map, stop discovery
            if (discoveredDevices.size() == targetDeviceNames.size()) {
                Serial.println("------------------ Found all target devices. Stopping discovery...");
                esp_bt_gap_cancel_discovery();
                isDiscoveryComplete = true;
            }

            break;
        }

        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            std::string bda_str = bdaToString(param->auth_cmpl.bda);
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                Serial.printf("Authentication success: %s\n", param->auth_cmpl.device_name);
                Serial.printf("BDA: %s\n", bda_str.c_str());
            } else {
                Serial.printf("Authentication failed, status: %d\n", param->auth_cmpl.stat);
                Serial.printf("BDA: %s\n", bda_str.c_str());
            }
            break;
        }

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            Serial.println("Discovery state changed.");
            break;

        case ESP_BT_GAP_RMT_SRVCS_EVT: {
            std::string bda_str = bdaToString(param->rmt_srvcs.bda);
            Serial.printf("Remote services for BDA: %s\n", bda_str.c_str());
            break;
        }

        case ESP_BT_GAP_RMT_SRVC_REC_EVT:
            Serial.println("Received remote service record.");
            break;

        case ESP_BT_GAP_PIN_REQ_EVT: {
            std::string bda_str = bdaToString(param->pin_req.bda);
            Serial.printf("PIN code request for BDA: %s\n", bda_str.c_str());
            break;
        }

        case ESP_BT_GAP_CFM_REQ_EVT: {
            std::string bda_str = bdaToString(param->cfm_req.bda);
            Serial.printf("Confirm request for passkey: %d, for BDA: %s\n", param->cfm_req.num_val, bda_str.c_str());
            break;
        }

        case ESP_BT_GAP_KEY_NOTIF_EVT: {
            std::string bda_str = bdaToString(param->key_notif.bda);
            Serial.printf("Passkey notification: %d, for BDA: %s\n", param->key_notif.passkey, bda_str.c_str());
            break;
        }

        case ESP_BT_GAP_KEY_REQ_EVT: {
            std::string bda_str = bdaToString(param->key_req.bda);
            Serial.printf("Passkey request for BDA: %s\n", bda_str.c_str());
            break;
        }

        case ESP_BT_GAP_READ_RSSI_DELTA_EVT: {
            std::string bda_str = bdaToString(param->read_rssi_delta.bda);
            Serial.printf("RSSI delta event for BDA: %s\n", bda_str.c_str());
            break;
        }
                         
        case ESP_BT_GAP_SET_AFH_CHANNELS_EVT:
            // Handle the set AFH channels event here
            Serial.println("Set AFH channels event triggered.");
            break;

        case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
            // Handle the config EIR data event here
            Serial.println("Config EIR data event triggered.");
            break;

        case ESP_BT_GAP_MODE_CHG_EVT: {
            std::string bda_str = bdaToString(param->mode_chg.bda);
            Serial.println("ESP_BT_GAP_MODE_CHG_EVT: Mode change event triggered.");
            Serial.printf("Device BDA: %s\n", bda_str.c_str());
            Serial.printf("New mode: %d\n", param->mode_chg.mode);
            break;
        }

        //------added
        case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
            Serial.println("Received GAP event: ESP_BT_GAP_READ_REMOTE_NAME_EVT");
            Serial.println("Remote name read event triggered.");
            break;

        case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT:
            Serial.println("Received GAP event: ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT");
            Serial.println("Bonded device removal complete.");
            break;

        case ESP_BT_GAP_QOS_CMPL_EVT:
            Serial.println("Received GAP event: ESP_BT_GAP_QOS_CMPL_EVT");
            Serial.println("Quality of Service (QoS) complete.");
            break;

        case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
            Serial.println("Received GAP event: ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT");
            Serial.println("ACL connection complete status.");
            break;

        case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
            Serial.println("Received GAP event: ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT");
            Serial.println("ACL disconnection complete status.");
            break;
        //------end added

        // ... (other cases remain unchanged)

        default: 
            static char unknownEventStr[50];
            snprintf(unknownEventStr, sizeof(unknownEventStr), "UNKNOWN_EVENT (Value: %d)", event);
    }
}

void a2dp_callback(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *a2d) {
    switch (event) {
        case ESP_A2D_CONNECTION_STATE_EVT:
            if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
                Serial.println("A2DP connected");
            } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
                Serial.println("A2DP disconnected");
            }
            break;
        // ... You can handle more A2DP-specific events here ...
    }
}

void pair_with_device(const std::string& bdaStr, const std::string& deviceName) {

    Serial.printf("Attempting to pair with device: Name: %s, BDA: %s\n", deviceName.c_str(), bdaStr.c_str());

    esp_bd_addr_t bda;
    stringToBda(bdaStr, bda);

    // Set ESP32 to use SSP mode
    // esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    // esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO; // Set to DisplayYesNo for devices with display & input capabilities
    // esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
}

void pair_with_all_discovered_devices() {
    for (auto& pair : discoveredDevices) {
        pair_with_device(pair.first, pair.second); // Here, pair.first is the BDA string and pair.second is the device name string
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2-second delay for clarity and to avoid too rapid consecutive actions
    }
    Serial.println("DONE Attempting to pair.");
}

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
            static char unknownEventStr[50];
            snprintf(unknownEventStr, sizeof(unknownEventStr), "UNKNOWN_EVENT (Value: %d)", event);
            return unknownEventStr;
    }
}

void app_main() {
    Initialize_Stack();
    Start_Discovery();

//    while(1) { // Forever loop to try and pair with the devices
        if (isDiscoveryComplete) {
            Serial.println("Discovery complete. Initiating pairing...");
            pair_with_all_discovered_devices();
            isDiscoveryComplete = false;  // Reset the flag to avoid re-pairing in this example
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for a second before checking again
//    }

    //register BLE callback
    esp_err_t ret = esp_ble_gap_register_callback(ble_gap_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register BLE GAP callback. Error: %d", ret);
    }
    Serial.println("Register BLE gap callback SUCCEEDED");

    start_ble_scan();
    compareAndReportMatches();

}


//====================================== START BLE CODE ==============================

void ble_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch(event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            Serial.println("BLE Scan parameters set complete.");
            break;

        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if(param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                Serial.println("BLE Scan started successfully.");
            } else {
                Serial.println("BLE Scan start failed.");
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
                Serial.println("BLE scan completed successfully");
            } else {
                Serial.println("BLE scan failed to stop scanning");
            }
            for (const auto& entry : bleDiscoveredDevices) {
                Serial.printf("Found BLE device. BDA: %s, Name: %s\n", entry.first.c_str(), entry.second.c_str());
            }
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            Serial.println("BLE Advertise start complete.");
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            Serial.println("BLE Connection parameters updated.");
            break;

        // Add cases for other events if needed.

        default:
            Serial.printf("Unhandled BLE GAP event %d.\n", event);
            break;
    }
}

void ble_scan_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        // Start scanning when scan parameters are set
        esp_ble_gap_start_scanning(10);  // scan for 10 seconds
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        // Scan start complete event
        if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            Serial.println("BLE Device Scanning...");
        } else {
            Serial.println("Failed to start BLE scanning");
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT:
        if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {

            std::string bda_std_string = bdaToString(param->scan_rst.bda);

            if (bleDiscoveredDevices.find(bda_std_string) == bleDiscoveredDevices.end()) {
                // New device discovered
                bleDiscoveredDevices[bda_std_string] = bda_std_string;  // Store the BDA string as both key and value

                Serial.printf("Found BLE device. Address: %s, Name: %s\n", 
                              bda_std_string.c_str(), 
                              param->scan_rst.ble_adv[2] == '\0' ? "Unknown" : (char *) &param->scan_rst.ble_adv[2]);
                
                // Check if this is one of the target devices
                if (targetDeviceNames.find(bda_std_string) != targetDeviceNames.end()) {
                    Serial.printf("---> This is a target device: %s\n", targetDeviceNames[bda_std_string].c_str());
                }
            }
        }
        break;

    default:
        break;
    }
}

void start_ble_scan() {
    esp_ble_gap_register_callback(ble_scan_callback);
    
    static esp_ble_scan_params_t ble_scan_params = {
        .scan_type              = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval          = 0x50,
        .scan_window            = 0x30,
    };
    esp_ble_gap_set_scan_params(&ble_scan_params);
}

void compareAndReportMatches() {
    Serial.println("Comparing BLE discovered devices with target BT devices...");
    bool matchesFound = false;

    for (const auto& bleDevice : bleDiscoveredDevices) {
        if (targetDeviceNames.find(bleDevice.first) != targetDeviceNames.end()) {
            matchesFound = true;
            Serial.printf("Match found: BDA: %s, Name (from BT): %s, Name (from BLE): %s\n", 
                          bleDevice.first.c_str(),
                          targetDeviceNames[bleDevice.first].c_str(), 
                          bleDevice.second.c_str());
        }
    }

    if (!matchesFound) {
        Serial.println("BLE Discovery Found NO Devices that match the Target Devices");
    }
}

BleAdvertisementData extractBleAdvFields(const uint8_t* ble_adv, uint8_t adv_length) {
    BleAdvertisementData advData;
    uint8_t index = 0;
    
    while (index < adv_length) {
        uint8_t length = ble_adv[index++];
        if (length == 0) break; // End of advertisement data
        
        uint8_t type = ble_adv[index];
        
        switch(type) {
            case 0x09: // Complete Local Name
                advData.deviceName.assign(reinterpret_cast<const char*>(ble_adv + index + 1), length - 1);
                break;
            case 0x19: // Appearance
                advData.appearance = (ble_adv[index + 2] << 8) | ble_adv[index + 1];
                break;
            case 0x0A: // Tx Power Level
                advData.txPowerLevel = ble_adv[index + 1];
                break;
            case 0xFF: // Manufacturer Specific Data
                advData.manufacturerData.assign(ble_adv + index + 1, ble_adv + index + length);
                break;
            case 0x02: // Incomplete List of 16-bit Service Class UUIDs
            case 0x03: // Complete List of 16-bit Service Class UUIDs
                // You can expand on this as needed
                advData.serviceUUIDs.assign(ble_adv + index + 1, ble_adv + index + length);
                break;
            // ... Add more cases as needed
        }
        
        index += length;
    }

    return advData;
}

void printBleAdvertisementData(const uint8_t* ble_adv, uint8_t adv_length, const std::string& address) {
    BleAdvertisementData advData = extractBleAdvFields(ble_adv, adv_length);

    Serial.printf("Found BLE device. Address: %s\n", address.c_str());

    // Print device name if available
    Serial.printf("Device name length: %d\n", advData.deviceName.length());
    if (!advData.deviceName.empty()) {
        Serial.printf("Device Name: %s\n", advData.deviceName.c_str());
    } else {
        Serial.println("Device Name: Unknown");
    }

    // Print Appearance if available
    Serial.printf("Appearance: %d\n", advData.appearance);
    if (advData.appearance != UINT16_MAX) {
        Serial.printf("Valid Appearance: %d\n", advData.appearance);
    }

    // Print Tx Power Level if available
    Serial.printf("Tx Power Level: %d dBm\n", advData.txPowerLevel);
    if (advData.txPowerLevel != INT8_MIN) {
        Serial.printf("Valid Tx Power Level: %d dBm\n", advData.txPowerLevel);
    }

    // Print Manufacturer Specific Data if available
    if (!advData.manufacturerData.empty()) {
        Serial.print("Manufacturer Data: ");
        for (uint8_t byte : advData.manufacturerData) {
            Serial.printf("%02X ", byte);
        }
        Serial.println();
    }

    // Print Service UUIDs if available
    if (!advData.serviceUUIDs.empty()) {
        Serial.print("Service UUIDs: ");
        for (uint8_t byte : advData.serviceUUIDs) {
            Serial.printf("%02X ", byte);
        }
        Serial.println();
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


void print_ESP32_info(){
    // Print out general ESP32 information
    Serial.println();
    Serial.printf("ESP-IDF Version: %s\n", esp_get_idf_version());
    
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    Serial.printf("Chip Model: %d\n", chip_info.model);
    Serial.printf("Chip Cores: %d\n", chip_info.cores);
    Serial.printf("Chip Revision: %d\n", chip_info.revision);
    
    Serial.printf("Flash Chip Size: %u bytes\n", spi_flash_get_chip_size());

    // Print Bluetooth Controller Information
    esp_bt_controller_status_t status = esp_bt_controller_get_status();
    switch (status) {
        case ESP_BT_CONTROLLER_STATUS_IDLE:
            Serial.println("BT Controller Status: IDLE");
            break;
        case ESP_BT_CONTROLLER_STATUS_INITED:
            Serial.println("BT Controller Status: INITED");
            break;
        case ESP_BT_CONTROLLER_STATUS_ENABLED:
            Serial.println("BT Controller Status: ENABLED");
            break;
        case ESP_BT_CONTROLLER_STATUS_NUM:
            Serial.println("BT Controller Status: UNKNOWN");
            break;
    }

    return;
}

// Convert esp_bd_addr_t (Bluetooth Device Address) to its string representation
std::string bdaToString(const esp_bd_addr_t bda) {
    char bda_str[18];
    sprintf(bda_str, "%02X:%02X:%02X:%02X:%02X:%02X", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    return std::string(bda_str);
}

// Convert string representation of BDA to esp_bd_addr_t
void stringToBda(const std::string& bda_string, esp_bd_addr_t& bda) {
    sscanf(bda_string.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x", &bda[0], &bda[1], &bda[2], &bda[3], &bda[4], &bda[5]);
}

//====================================== END UTILITIES ==============================


void setup() {
  app_main();
}

void loop() {
    // Code to be run repeatedly here
}

