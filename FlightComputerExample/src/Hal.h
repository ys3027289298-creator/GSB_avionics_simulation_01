#pragma once

struct SensorData {
    float pressure_pa;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float lat, lon, altitude_gps;
};



float HAL_ReadPressure();

float HAL_ReadAccelX();
float HAL_ReadAccelY();
float HAL_ReadAccelZ();

float HAL_ReadGyroX();
float HAL_ReadGyroY();
float HAL_ReadGyroZ();

float HAL_ReadLat();
float HAL_ReadLon();
float HAL_ReadAlt();

void HAL_Init();
void HAL_Update();
void HAL_SendEvent(const char* event_name, float value);
void HAL_FirePyro1(float current_altitude);
void HAL_FirePyro2(float current_altitude);
