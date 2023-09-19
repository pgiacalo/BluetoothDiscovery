/*
 * ESP32 code for Bluetooth testing (using Arduino IDE)
 * This code is designed to discover an A2DP bluetooth device, pair with it, and then connect. 
 * So far, this code is discovering devices. 
 */

#include <Arduino.h>

#include <map>
#include <string.h>
#include <set>

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
std::map<std::string, std::string> deviceNames = {
    // {"74:74:46:ED:07:6B", "Pixel Buds A Series"},
    {"58:FC:C6:6C:0A:03", "TOZO Home"},
    {"54:B7:E5:8C:07:71", "TOZO Mazda"}
    //REMINDER: no trailing comma
};

std::map<std::string, esp_bd_addr_t> discoveredDevices;

//Bluetooth Device Address (with is the MAC (Media Access Control))
esp_bd_addr_t target_device_bda;
bool target_device_found = false;

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
            char bda_str[18];
            sprintf(bda_str, "%02X:%02X:%02X:%02X:%02X:%02X", 
                param->disc_res.bda[0], param->disc_res.bda[1], param->disc_res.bda[2], 
                param->disc_res.bda[3], param->disc_res.bda[4], param->disc_res.bda[5]);

            // Print every discovered device for debugging
            // Serial.printf("Discovered device BDA: %s\n", bda_str);

            //now we'll check to see if its one of the devices we want to connect to
            auto it = deviceNames.find(bda_str);
            if (it != deviceNames.end() && discoveredDevices.find(bda_str) == discoveredDevices.end()) {
                Serial.printf("------------------ Found target device: %s (BDA: %s)\n", it->second.c_str(), bda_str);
                
                // Store the BDA and its logical name in the discoveredDevices map
                memcpy(discoveredDevices[bda_str], param->disc_res.bda, sizeof(esp_bd_addr_t));
                
            }

            // If we've discovered all devices in our map, stop discovery
            if (discoveredDevices.size() == deviceNames.size()) {
                Serial.println("------------------ Found all target devices. Stopping discovery...");
                esp_bt_gap_cancel_discovery();
                isDiscoveryComplete = true;
            }

            break;
        }

        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                Serial.printf("Authentication success: %s\n", param->auth_cmpl.device_name);
                Serial.printf("BDA: %02X:%02X:%02X:%02X:%02X:%02X\n",
                              param->auth_cmpl.bda[0], param->auth_cmpl.bda[1], param->auth_cmpl.bda[2],
                              param->auth_cmpl.bda[3], param->auth_cmpl.bda[4], param->auth_cmpl.bda[5]);
            } else {
                Serial.printf("Authentication failed, status: %d\n", param->auth_cmpl.stat);
                Serial.printf("BDA: %02X:%02X:%02X:%02X:%02X:%02X\n",
                              param->auth_cmpl.bda[0], param->auth_cmpl.bda[1], param->auth_cmpl.bda[2],
                              param->auth_cmpl.bda[3], param->auth_cmpl.bda[4], param->auth_cmpl.bda[5]);
            }
            break;
        }        

        case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
            Serial.println("Discovery state changed.");
            break;

        case ESP_BT_GAP_RMT_SRVCS_EVT:
            Serial.printf("Remote services for BDA: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          param->rmt_srvcs.bda[0], param->rmt_srvcs.bda[1], param->rmt_srvcs.bda[2],
                          param->rmt_srvcs.bda[3], param->rmt_srvcs.bda[4], param->rmt_srvcs.bda[5]);
            break;

        case ESP_BT_GAP_RMT_SRVC_REC_EVT:
            Serial.println("Received remote service record.");
            break;

        case ESP_BT_GAP_PIN_REQ_EVT:
            Serial.printf("PIN code request for BDA: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          param->pin_req.bda[0], param->pin_req.bda[1], param->pin_req.bda[2],
                          param->pin_req.bda[3], param->pin_req.bda[4], param->pin_req.bda[5]);
            break;

        case ESP_BT_GAP_CFM_REQ_EVT:
            Serial.printf("Confirm request for passkey: %d, for BDA: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          param->cfm_req.num_val,
                          param->cfm_req.bda[0], param->cfm_req.bda[1], param->cfm_req.bda[2],
                          param->cfm_req.bda[3], param->cfm_req.bda[4], param->cfm_req.bda[5]);
            break;

        case ESP_BT_GAP_KEY_NOTIF_EVT:
            Serial.printf("Passkey notification: %d, for BDA: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          param->key_notif.passkey,
                          param->key_notif.bda[0], param->key_notif.bda[1], param->key_notif.bda[2],
                          param->key_notif.bda[3], param->key_notif.bda[4], param->key_notif.bda[5]);
            break;

        case ESP_BT_GAP_KEY_REQ_EVT:
            Serial.printf("Passkey request for BDA: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          param->key_req.bda[0], param->key_req.bda[1], param->key_req.bda[2],
                          param->key_req.bda[3], param->key_req.bda[4], param->key_req.bda[5]);
            break;

        case ESP_BT_GAP_READ_RSSI_DELTA_EVT:
            /* Commented out due to compilation issue
            Serial.printf("RSSI delta for BDA: %02X:%02X:%02X:%02X:%02X:%02X, RSSI: %d\n",
                          param->read_rssi_delta.bda[0], param->read_rssi_delta.bda[1], param->read_rssi_delta.bda[2],
                          param->read_rssi_delta.bda[3], param->read_rssi_delta.bda[4], param->read_rssi_delta.bda[5],
                          param->read_rssi_delta.rssi);
            */
            Serial.printf("RSSI delta event for BDA: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          param->read_rssi_delta.bda[0], param->read_rssi_delta.bda[1], param->read_rssi_delta.bda[2],
                          param->read_rssi_delta.bda[3], param->read_rssi_delta.bda[4], param->read_rssi_delta.bda[5]);
            break;

        // case ESP_BT_GAP_LINK_STATUS_EVT:
        //     Serial.printf("Link status changed for BDA: %02X:%02X:%02X:%02X:%02X:%02X, status: %d\n",
        //                   param->link_status.bda[0], param->link_status.bda[1], param->link_status.bda[2],
                         
        case ESP_BT_GAP_SET_AFH_CHANNELS_EVT:
            // Handle the set AFH channels event here
            Serial.println("Set AFH channels event triggered.");
            break;

        case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
            // Handle the config EIR data event here
            Serial.println("Config EIR data event triggered.");
            break;

        case ESP_BT_GAP_MODE_CHG_EVT:
            Serial.println("ESP_BT_GAP_MODE_CHG_EVT: Mode change event triggered.");
            Serial.printf("Device BDA: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                          param->mode_chg.bda[0], param->mode_chg.bda[1], param->mode_chg.bda[2], 
                          param->mode_chg.bda[3], param->mode_chg.bda[4], param->mode_chg.bda[5]);
            Serial.printf("New mode: %d\n", param->mode_chg.mode);
            break;

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

void pair_with_device(const uint8_t* bda, const std::string& deviceName) {
    // Logging device details
    Serial.printf("Attempting to pair with device: %s (BDA: %02X:%02X:%02X:%02X:%02X:%02X)\n", 
                  deviceName.c_str(), bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

    // Set ESP32 to use SSP mode
    // esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    // esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO; // Set to DisplayYesNo for devices with display & input capabilities
    // esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));


}


// void pair_with_device(const uint8_t* bda, const std::string& deviceName) {
//     // Declare the pin_code here
//     esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};

//     // Your pairing code here...
//     Serial.printf("Attempting to pair with device: %s (BDA: %02X:%02X:%02X:%02X:%02X:%02X)\n", 
//                   deviceName.c_str(), bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    
//     // Set the PIN
//     esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin_code);
    
//     // Initiate pairing
//     esp_err_t ret = esp_bt_gap_pin_reply(const_cast<uint8_t*>(bda), true, 4, pin_code);
//     if (ret != ESP_OK) {
//         Serial.printf("Error in pairing with BDA: %02X:%02X:%02X:%02X:%02X:%02X. Error code: %s\n", 
//                       bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], esp_err_to_name(ret));
//     }
// }

void pair_with_all_discovered_devices() {
    for (auto& pair : discoveredDevices) {
        pair_with_device(const_cast<uint8_t*>(pair.second), deviceNames[pair.first]);
        // pair_with_device(const_cast<uint8_t*>(pair.second), pair.first);
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
            if(param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                Serial.print("BLE Device found, address: ");
                for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
                    Serial.printf("%02x:", param->scan_rst.bda[i]);
                }
                Serial.println();
            }
            break;

        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if(param->scan_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                Serial.println("BLE Scan stopped successfully.");
            } else {
                Serial.println("BLE Scan stop failed.");
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
            // Found a device
            Serial.printf("Found BLE device. Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          param->scan_rst.bda[0], param->scan_rst.bda[1], param->scan_rst.bda[2], 
                          param->scan_rst.bda[3], param->scan_rst.bda[4], param->scan_rst.bda[5]);
        }
        break;

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            Serial.println("Scan completed successfully");
        } else {
            Serial.println("Failed to stop scanning");
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

//====================================== END BLE CODE ==============================

void setup() {
  app_main();
}

void loop() {
    // Code to be run repeatedly here
}

