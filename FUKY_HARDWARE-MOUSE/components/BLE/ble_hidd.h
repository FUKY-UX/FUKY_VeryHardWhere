#ifndef BLE_HIDD_H
#define BLE_HIDD_H

#ifdef __cplusplus
extern "C" {
#endif


void BLE_HID_Init(void);

void send_mouse_value(uint8_t mouse_button, int8_t mickeys_x, int8_t mickeys_y);

void SendIMUData(int16_t AccX,int16_t AccY,int16_t AccZ,int16_t QuatI,int16_t QuatJ,int16_t QuatK,int16_t QuatW);
#ifdef __cplusplus
}
#endif

#endif // BLE_HIDD_H
