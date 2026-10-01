#include "Hal.h"
#include <stdio.h>
#include <string.h>
#include <winsock2.h>
#include <ws2tcpip.h>

static SensorData latest = {};
static SOCKET sock = INVALID_SOCKET;

void HAL_Init() {
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2,2), &wsaData);

    sock = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(9000);
    inet_pton(AF_INET, "127.0.0.1", &server.sin_addr);

    connect(sock, (struct sockaddr*)&server, sizeof(server));
    printf("[HAL] Connected to simulator\n");
}

void HAL_Update() {
    char buf[256] = {};
    int n = recv(sock, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return;

    sscanf(buf, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
        &latest.pressure_pa,
        &latest.accel_x, &latest.accel_y, &latest.accel_z,
        &latest.gyro_x, &latest.gyro_y, &latest.gyro_z,
        &latest.lat, &latest.lon, &latest.altitude_gps
    );
}

void HAL_SendEvent(const char* event_name, float value) {
    char buf[128];
    snprintf(buf, sizeof(buf), "EVENT,%s,%.2f\n", event_name, value);
    send(sock, buf, strlen(buf), 0);
}

void HAL_FirePyro1(float current_altitude) { HAL_SendEvent("PYRO1", current_altitude); }
void HAL_FirePyro2(float current_altitude) { HAL_SendEvent("PYRO2", current_altitude); }

float HAL_ReadPressure() { return latest.pressure_pa;  }
float HAL_ReadAccelX()   { return latest.accel_x;      }
float HAL_ReadAccelY()   { return latest.accel_y;      }
float HAL_ReadAccelZ()   { return latest.accel_z;      }
float HAL_ReadGyroX()    { return latest.gyro_x;       }
float HAL_ReadGyroY()    { return latest.gyro_y;       }
float HAL_ReadGyroZ()    { return latest.gyro_z;       }
float HAL_ReadLat()      { return latest.lat;          }
float HAL_ReadLon()      { return latest.lon;          }
float HAL_ReadAlt()      { return latest.altitude_gps; }
