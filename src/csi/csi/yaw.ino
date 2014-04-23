
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 accelgyro;
int16_t ax, ay, az; //define 3-axis accelerometer variables
int16_t gx, gy, gz; //define 3-axis gyroscop variables
signed long yaw=0; //define angle parameters

void init_imu() {
    Wire.begin();
    accelgyro.initialize();
    accelgyro.setRate(0);//set the sampling rate to be 1khz
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);//set gyro to be +-250
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);
    delay(1000);
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    yaw = atan2(ay, ax)/PI*180;
    last_time = micros();
}

void control_R2() { 
    int straight_value = pwm_out[R2];
    
    int delta = map(yaw, -200, 200, -15, 15);
    
    // PUT THE TILT CORRECTION HERE.
    // corrected_value starts out as the value that makes the robot go
    // straight. yaw is the angle from the IMU. Return the desired value.
    
    if (delta < 0) {
        delta = 0 - delta;
        int sum = straight_value + delta;
        int dif = 0;
        if (sum > 255) {
            dif = sum - 255;
            dif = dif * 3;
            sum = 255;
        }
        analogWrite(R2, straight_value - dif);
        analogWrite(L2, sum);
    }
    else {
        int sum = straight_value + delta;
        int dif = 0;
        if (sum > 255) {
            dif = sum - 255;
            dif = dif * 3;
            sum = 255;
        }
        analogWrite(L2, straight_value - dif); 
        analogWrite(R2, sum);
    }
}

signed long get_yaw() { return yaw; };

void update_yaw() {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    // 100,000ths of a second
    signed long sdT = dT / 256;
   
    // 100ths of a dregree * 100
    signed long accYaw = long(atan2(ay,ax)*(5730.0*2.0)); // 1/100 ths of a degree * 10
    // 100ths of a degree * dT
    // 100*gz/65.5 (100ths of a degree/s) * dT/100,000 (s) * dT
    signed long  gyro_dYaw = (sdT*sdT*gz); // 1/100ths of a degree * dT
    
    //comlementary filter
    yaw = ((dT*yaw) - gyro_dYaw + accYaw) / (dT+5);
}


