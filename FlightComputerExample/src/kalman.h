#pragma once

struct KalmanState {
    float altitude = 0;
    float velocity = 0;
    float accel = 0;
    

    // Covariance matrix (3x3), flattened row-major
    float P[3][3] = {
        {0.1, 0, 0},
        {0, 0.1, 0},
        {0, 0, 0.1}
    };

    bool apogee_triggered = false;
    bool main_triggered = false;
};

const float dt = 0.1f;

// Process noise
const float Q[3][3] = {
    {0.001f, 0, 0},
    {0, 0.001f, 0},
    {0, 0, 1.0f}
};

const float R = 0.5f;  // measurement noise

inline void kalman_predict(KalmanState &k) {
    // State transition: x = F*x  (F is constant-acceleration model)
    float new_altitude = k.altitude + k.velocity * dt + 0.5f * k.accel * dt * dt;
    float new_velocity  = k.velocity + k.accel * dt;
    float new_accel     = k.accel;

    k.altitude = new_altitude;
    k.velocity = new_velocity;
    k.accel    = new_accel;

    // Covariance update: P = F*P*F' + Q
    // (manually expanded for a 3x3 constant-acceleration F matrix)
    float F[3][3] = {
        {1, dt, 0.5f * dt * dt},
        {0, 1,  dt},
        {0, 0,  1}
    };

    float FP[3][3] = {};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int m = 0; m < 3; m++)
                FP[i][j] += F[i][m] * k.P[m][j];

    float FPFt[3][3] = {};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int m = 0; m < 3; m++)
                FPFt[i][j] += FP[i][m] * F[j][m];  // F transposed

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            k.P[i][j] = FPFt[i][j] + Q[i][j];
}

inline void kalman_update(KalmanState &k, float z) {
    // H = [1 0 0] -> we only measure altitude
    float y = z - k.altitude;                 // innovation
    float S = k.P[0][0] + R;                   // innovation covariance
    float K0 = k.P[0][0] / S;                  // Kalman gain
    float K1 = k.P[1][0] / S;
    float K2 = k.P[2][0] / S;

    k.altitude += K0 * y;
    k.velocity += K1 * y;
    k.accel    += K2 * y;

    // P = (I - K*H) * P  — since H picks out row 0 only, this simplifies to:
    float P00 = k.P[0][0], P01 = k.P[0][1], P02 = k.P[0][2];
    for (int j = 0; j < 3; j++) {
        k.P[0][j] -= K0 * (j == 0 ? P00 : (j == 1 ? P01 : P02));
        k.P[1][j] -= K1 * (j == 0 ? P00 : (j == 1 ? P01 : P02));
        k.P[2][j] -= K2 * (j == 0 ? P00 : (j == 1 ? P01 : P02));
    }
}

// Returns true the moment apogee is detected (only fires once)
inline bool check_apogee(KalmanState &k) {
    if (!k.apogee_triggered && k.velocity <= 0 && k.altitude > 5) {
        k.apogee_triggered = true;
        return true;
    }
    return false;
}

inline bool check_main(KalmanState &k) {
    if (k.altitude < 100){
        k.main_triggered = true;
        return true;
    }
    return false;
}