/*
 * hal_sim.cpp — Simulation HAL for firmware SITL/HIL testing
 *
 * Drop this file into your project alongside Hal.h.
 * Implement the two platform serial functions below for your hardware.
 *
 * Packet format (server → firmware):
 *   pressure,ax,ay,az,gx,gy,gz,lat,lon,alt\n
 *
 * Event format (firmware → server):
 *   EVENT,<name>,<value>\n
 */

#include "Hal.h"
#include <stdio.h>
#include <string.h>

/* ---------------------------------------------------------------
 * PLATFORM SERIAL INTERFACE
 * Implement these two functions for your hardware.
 * ---------------------------------------------------------------
 *
 * Write len bytes from buf to serial port.
 * Return number of bytes written.
 *
 *   int hal_serial_write(const char* buf, int len);
 *
 * Read up to len bytes into buf from serial port.
 * Should block until at least one byte is available.
 * Return number of bytes read, or 0 on timeout.
 *
 *   int hal_serial_read(char* buf, int len);
 *
 * Example — ESP32 Arduino:
 *
 *   #include <HardwareSerial.h>
 *   int hal_serial_write(const char* buf, int len) {
 *       return Serial.write((const uint8_t*)buf, len);
 *   }
 *   int hal_serial_read(char* buf, int len) {
 *       int i = 0;
 *       while (i < len) {
 *           while (!Serial.available());
 *           buf[i++] = Serial.read();
 *           if (buf[i-1] == '\n') break;
 *       }
 *       return i;
 *   }
 *
 * Example — STM32 HAL:
 *
 *   int hal_serial_write(const char* buf, int len) {
 *       HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY);
 *       return len;
 *   }
 *   int hal_serial_read(char* buf, int len) {
 *       int i = 0;
 *       uint8_t c;
 *       while (i < len) {
 *           HAL_UART_Receive(&huart2, &c, 1, HAL_MAX_DELAY);
 *           buf[i++] = c;
 *           if (c == '\n') break;
 *       }
 *       return i;
 *   }
 * --------------------------------------------------------------- */

extern int hal_serial_write(const char* buf, int len);
extern int hal_serial_read(char* buf, int len);


/* --------------------------------------------------------------- */

static SensorData latest = {};

static int read_line(char* out, int max_len) {
    int i = 0;
    while (i < max_len - 1) {
        char c = 0;
        int n = hal_serial_read(&c, 1);
        if (n <= 0) break;
        out[i++] = c;
        if (c == '\n') break;
    }
    out[i] = '\0';
    return i;
}

void HAL_Init() {
    /* Serial port should already be initialised by the time HAL_Init
     * is called. If your platform needs explicit init, do it before
     * calling HAL_Init(), or add it here. */
}

void HAL_Update() {
    char buf[256] = {};
    int n = read_line(buf, sizeof(buf));
    if (n <= 0) return;

    sscanf(buf, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
        &latest.pressure_pa,
        &latest.accel_x,    &latest.accel_y,    &latest.accel_z,
        &latest.gyro_x,     &latest.gyro_y,     &latest.gyro_z,
        &latest.lat,        &latest.lon,
        &latest.altitude_gps
    );
}

void HAL_SendEvent(const char* event_name, float value) {
    char buf[128];
    int len = snprintf(buf, sizeof(buf), "EVENT,%s,%.2f\n", event_name, value);
    hal_serial_write(buf, len);
}

void HAL_FirePyro1(float current_altitude) {
    HAL_SendEvent("PYRO1", current_altitude);
}

void HAL_FirePyro2(float current_altitude) {
    HAL_SendEvent("PYRO2", current_altitude);
}

float HAL_ReadPressure() { return latest.pressure_pa;    }
float HAL_ReadAccelX()   { return latest.accel_x;        }
float HAL_ReadAccelY()   { return latest.accel_y;        }
float HAL_ReadAccelZ()   { return latest.accel_z;        }
float HAL_ReadGyroX()    { return latest.gyro_x;         }
float HAL_ReadGyroY()    { return latest.gyro_y;         }
float HAL_ReadGyroZ()    { return latest.gyro_z;         }
float HAL_ReadLat()      { return latest.lat;            }
float HAL_ReadLon()      { return latest.lon;            }
float HAL_ReadAlt()      { return latest.altitude_gps;   }
