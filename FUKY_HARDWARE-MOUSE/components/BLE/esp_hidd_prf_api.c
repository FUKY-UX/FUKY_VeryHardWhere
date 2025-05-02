/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "esp_hidd_prf_api.h"
#include "hid_dev.h"
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

// HID mouse input report length
#define HID_MOUSE_IN_RPT_LEN        5

// HID mouse input report length
#define HID_IMU_IN_RPT_LEN          14

esp_err_t esp_hidd_register_callbacks(esp_hidd_event_cb_t callbacks)
{
    esp_err_t hidd_status;

    if(callbacks != NULL) {
   	    hidd_le_env.hidd_cb = callbacks;
    } else {
        return ESP_FAIL;
    }

    if((hidd_status = hidd_register_cb()) != ESP_OK) {
        return hidd_status;
    }
    esp_err_t RET = esp_ble_gatts_app_register(IMU_APP_ID);
    if( RET != ESP_OK) {
        ESP_LOGE("奇怪情况", "他可能不支持自定义的UUID");
        return RET;
    }

    
    if((hidd_status = esp_ble_gatts_app_register(HIDD_APP_ID)) != ESP_OK) {
        return hidd_status;
    }
    //esp_ble_gatts_app_register(DIS_APP_ID);

    return hidd_status;
}

esp_err_t esp_hidd_profile_init(void)
{
     if (hidd_le_env.enabled) {
        ESP_LOGE(HID_LE_PRF_TAG, "HID device profile already initialized");
        return ESP_FAIL;
    }
    // Reset the hid device target environment
    memset(&hidd_le_env, 0, sizeof(hidd_le_env_t));
    hidd_le_env.enabled = true;
    return ESP_OK;
}

uint16_t esp_hidd_get_version(void)
{
	return HIDD_VERSION;
}

void esp_hidd_send_mouse_value(uint16_t conn_id, uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y)
{
    uint8_t buffer[HID_MOUSE_IN_RPT_LEN];

    buffer[0] = mouse_button;   // Buttons
    buffer[1] = mickeys_x;           // X
    buffer[2] = mickeys_y;           // Y
    buffer[3] = 0;           // Wheel
    buffer[4] = 0;           // AC Pan

    hid_dev_send_report(hidd_le_env.gatt_if, conn_id,
                        HID_RPT_ID_MOUSE_IN, HID_REPORT_TYPE_INPUT, HID_MOUSE_IN_RPT_LEN, buffer);
    return;
}

void esp_hidd_send_imu_value(uint16_t conn_id,int16_t lin_accel_x,int16_t lin_accel_y,int16_t lin_accel_z,
    int16_t quat_i,int16_t quat_j,int16_t quat_k,int16_t quat_w)
{
    uint8_t buffer[HID_IMU_IN_RPT_LEN];

    // 逐个填充 buffer，按照小端序 (Little Endian)
    buffer[0]  = lin_accel_x & 0xFF;
    buffer[1]  = (lin_accel_x >> 8) & 0xFF;
    buffer[2]  = lin_accel_y & 0xFF;
    buffer[3]  = (lin_accel_y >> 8) & 0xFF;
    buffer[4]  = lin_accel_z & 0xFF;
    buffer[5]  = (lin_accel_z >> 8) & 0xFF;

    buffer[6]  = quat_i & 0xFF;
    buffer[7]  = (quat_i >> 8) & 0xFF;
    buffer[8]  = quat_j & 0xFF;
    buffer[9]  = (quat_j >> 8) & 0xFF;
    buffer[10] = quat_k & 0xFF;
    buffer[11] = (quat_k >> 8) & 0xFF;
    buffer[12] = quat_w & 0xFF;
    buffer[13] = (quat_w >> 8) & 0xFF;

    esp_ble_gatts_send_indicate(imu_env.IMU_gatt_if, conn_id,imu_env.IMU_att_handle, sizeof(buffer), buffer, false);
}