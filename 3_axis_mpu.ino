#define OUTPUT_READABLE_YAWPITCHROLL
#include "helper_3dmath.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#define INTERRUPT_PIN 2  
#define LED_PIN 13 
#define TO_DEG 57.29577951308232087679815481410517033f
#define T_OUT 20 
#define P_OUT 50 
#define FK 0.1 
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Servo servo1, servo2, servo3;
MPU6050 mpu;
MPU6050 accelgyro;
float angle_ax, angle_gx, angle_cpl;
float angle_ay, angle_gy;
int dt = 0;
long int t_next, p_next;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[1];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw;              // The Yaw variable



volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

float clamp(float v, float minv, float maxv){
    if( v>maxv )
        return maxv;
    else if( v<minv )
        return minv;
    return v;
}

void setup() {
    servo1.attach(10);
    servo2.attach(9);
    servo3.attach(8);
    servo1.write(0);
    servo2.write(0);
    servo3.write(0);
  // put your setup code here, to run once:
      // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately


    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-7);
    mpu.setYGyroOffset(18);
    mpu.setZGyroOffset(124);
    mpu.setZAccelOffset(5637); // 1688 factory default for my test chip

// make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
long int t = millis();
    // each T_TO ms calculating an angle
    if( t_next < t ){
        int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
        float ay,gx,ax,gy;

        t_next = t + T_OUT;
        // getting raw data from MPU6050 sensors
        accelgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

        // convert raw gyroscope data to в degrees/sec units
        gx = gx_raw / 131.0;
        gy = gy_raw / 131.0;
        // convert raw accelerometer data to G units
        ay = ay_raw / 16384.0;
        ay = clamp(ay, -1.0, 1.0);

        ax = ax_raw / 16384.0;
        ax = clamp(ax, -1.0, 1.0);

        // calculating an inclination angle from accelerometer
        angle_ax = 90 - TO_DEG*acos(ay);
        angle_ay = 90 - TO_DEG*acos(ax);
        // calculating the inclination angle from gyroscope
        angle_gx = angle_gx + gx * T_OUT/1000.0;
        angle_gy = angle_gy + gy * T_OUT/1000.0;
        // correcting the inclination angle from accelerometer
        angle_gx = angle_gx*(1-FK) + angle_ax*FK;
        angle_gy = angle_gy*(1-FK) + angle_ay*FK;
    }

            Serial.print("Yaw:\t"); 
            yaw = ypr[0] * 180/M_PI;
            Serial.print(yaw);
            servo3.write(map(yaw, -90, 90, 0, 180));
            Serial.print("\tRoll:\t");
            Serial.print(angle_gx);
            servo2.write(map(angle_gx, -90, 90, 180, 0));
            Serial.print("\tPitch:\t");
            Serial.println(angle_gy);
            servo1.write(map(angle_gy, -90, 90, 180, 0));
            
        #endif
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }

}
