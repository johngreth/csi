// Read 3-axis accelerations and angular velocities from MPU6050 to control the orientation of CSI robot w
//complimentary filter is applied in this case
// 4/07/2014 by Tianyu Zhao <jack11241008@gmail.com>
//
/* ============================================
This code uses I2C and MPU6050 library from Jeff Rowberg
// updated on 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define acc_scale_factor 16384 //16384 LSB/g
#define gyro_scale_factor 65.5  //131 LSB/(degree/s)


// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board) (we use this setup)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az; //define 3-axis accelerometer variables
int16_t gx, gy, gz; //define 3-axis gyroscop variables
double  yaw=0; //define angle parameters
double dT=0, time=0; //define time parameters

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    
    //Initialize MPU6050
    //Power on and prepare for general usage.This will activate the device and take it out of 
    //sleep mode (which must be done after start-up). This function also sets both the accelerometer and the gyroscope
    //to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
    //the clock source to use the X Gyro for reference, which is slightly better than
    //the default internal clock source.
    accelgyro.initialize();
    accelgyro.setRate(0);//set the sampling rate to be 1khz
    accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_500);//set gyro to be +-250
    accelgyro.setDLPFMode(MPU6050_DLPF_BW_42);
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    
/*
    // use the code below to change accel/gyro offset values
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    delay(1000);
    accelgyro.setXGyroOffset(125);
    accelgyro.setYGyroOffset(-2);
    accelgyro.setZGyroOffset(8);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */
    delay(1000); //delay to wait the setup to be ready
    //Measure initial yaw angle from accelerometer
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    yaw = atan2(ay, ax)/PI*180;
    time = millis();
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //calculate time passed after last measurement, and convert to second
    dT = double(millis()-time)/1000;
    time = millis();
   
    double accYaw = atan2(ay,ax)/PI*180;
    double gyroz = -(double)gz/gyro_scale_factor; //convert the gyroscope data to angular velocity and invert
    double gyro_dYaw = gyroz*dT;
    
    //comlementary filter
    double acc_gyro_weight = dT/(dT+0.001);
    yaw = acc_gyro_weight*(yaw+gyro_dYaw)+(1-acc_gyro_weight)*accYaw;
    Serial.println(yaw,5);
    
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

       /* // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);*/
        



}
