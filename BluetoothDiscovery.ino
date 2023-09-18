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

#define TAG "ESP32_PAIRING"
#define TARGET_DEVICE_NAME "TargetDevice"
bool device_found = false;

// a lookup table for the Bluetooth earbuds we want to discover, pair with and connect to.
std::map<std::string, std::string> deviceNames = {
    // {"74:74:46:ED:07:6B", "Pixel Buds A Series"},
    {"58:FC:C6:6C:0A:03", "TOZO Home"},
    {"54:B7:E5:8C:07:71", "TOZO Mazda"}
    //REMINDER: no trailing comma!
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

void pair_with_device(esp_bt_gap_cb_param_t *param) {
    Serial.println("Pairing with target device...");
    
    esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
    esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin_code);
    esp_bt_gap_pin_reply(param->disc_res.bda, true, 4, pin_code);
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
 * GAP stands for "Generic Access Profile." It's one of the core protocols 
 * GAP Handles:
 *  1) Discovery: GAP manages the discovery process, i.e., how devices advertise themselves and how they scan for other devices. This includes setting the device to be discoverable or non-discoverable, setting the device name, and defining the scan modes.
 *  2) Connection: Once two devices have discovered each other, GAP also handles how they establish a connection. This includes security features such as pairing, bonding, and the use of passkeys.
 *  3) Roles: GAP defines roles such as "Central" and "Peripheral." In the case of Bluetooth Low Energy (BLE), a Central device (like a smartphone) scans and connects to Peripheral devices (like a heart rate monitor).
 */
void app_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    switch (event) {
        case ESP_BT_GAP_DISC_RES_EVT: {
            char bda_str[18];
            sprintf(bda_str, "%02X:%02X:%02X:%02X:%02X:%02X", 
                param->disc_res.bda[0], param->disc_res.bda[1], param->disc_res.bda[2], 
                param->disc_res.bda[3], param->disc_res.bda[4], param->disc_res.bda[5]);

            auto it = deviceNames.find(bda_str);
            if (it != deviceNames.end() && discoveredDevices.find(bda_str) == discoveredDevices.end()) {
                Serial.printf("Found target device: %s (BDA: %s)\n", it->second.c_str(), bda_str);
                
                // Store the BDA and its logical name in the discoveredDevices map
                memcpy(discoveredDevices[bda_str], param->disc_res.bda, sizeof(esp_bd_addr_t));
                
                // Optional: Set pin if you want to pair immediately after discovery.
                // esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
                // esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin_code);
                // esp_bt_gap_pin_reply(param->disc_res.bda, true, 4, pin_code);
            }

            // If we've discovered all devices in our map, stop discovery
            if (discoveredDevices.size() == deviceNames.size()) {
                Serial.println("Found all target devices. Stopping discovery...");
                esp_bt_gap_cancel_discovery();
            }
            break;
        }
        // ... (other cases remain unchanged)
    }
}


// void app_gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
//     switch (event) {
//         case ESP_BT_GAP_DISC_RES_EVT: {
//             char bda_str[18];
//             sprintf(bda_str, "%02X:%02X:%02X:%02X:%02X:%02X", 
//                 param->disc_res.bda[0], param->disc_res.bda[1], param->disc_res.bda[2], 
//                 param->disc_res.bda[3], param->disc_res.bda[4], param->disc_res.bda[5]);

//             // Checking if the discovered BDA exists in our predefined set
//             auto it = deviceNames.find(bda_str);
//             if (it != deviceNames.end()) {
//                 Serial.printf("Found target device: %s (BDA: %s)\n", it->second.c_str(), bda_str);
                
//                 // Storing the BDA of the discovered target device
//                 memcpy(target_device_bda, param->disc_res.bda, sizeof(esp_bd_addr_t));

//                 esp_bt_gap_cancel_discovery();

//                 esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
//                 esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin_code);
//                 esp_bt_gap_pin_reply(param->disc_res.bda, true, 4, pin_code);
//             }
//             break;
//         }
//         // ... (other cases remain unchanged)
//     }
// }


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

void app_main() {
    Initialize_Stack();
    Serial.println("Stack Initialization Complete");
    Start_Discovery();
}

void setup() {
  app_main();
}

void loop() {
    // Code to be run repeatedly here
}