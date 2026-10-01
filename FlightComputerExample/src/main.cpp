#include <stdio.h>
#include <Hal.h>
#include <kalman.h>

KalmanState kf;
bool apogee = false;

void setup() {
  // CONNECT TO SIMULATOR
  HAL_Init();
  printf("[main] HAL initialised, waiting for data...\n");
}

void loop() {  
  // PULLS LATEST PACKET FROM SIMULATOR
  HAL_Update();

  // RUN YOUR AVIONICS CODE HERE

  // ALTITUDE DATA
  float altitude_from_baro = HAL_ReadAlt();

  // EXAMPLE KALMAN FILTER
  kalman_predict(kf);
  kalman_update(kf, altitude_from_baro);


  //printf("RAW alt: %.2f | KF alt: %.2f | KF vel: %.2f | triggered: %d\n",
  //  altitude_from_baro, kf.altitude, kf.velocity, kf.apogee_triggered);


  if (check_apogee(kf)) {
    printf(">>> APOGEE DETECTED at altitude %.2f m\n", kf.altitude);
    HAL_SendEvent("APOGEE", kf.altitude);

    HAL_FirePyro1(kf.altitude);
    printf(">>> PYRO 1 FIRED at altiude %.2f m\n", kf.altitude);
    apogee = true;
  }

  if (apogee && check_main(kf)) {
    HAL_FirePyro2(kf.altitude);
    printf(">>> PYRO 2 FIRED at altiude %.2f m\n", kf.altitude);
    apogee = false;
  }


}

