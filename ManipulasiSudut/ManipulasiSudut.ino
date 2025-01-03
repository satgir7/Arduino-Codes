#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
float sudutRobot;
float sudutIMU;

int penambahanIMU = 0;
int pengaliIMU = 0;
void setup() {
  Serial.begin(9600);
  panggilMPU6050();
}

void loop() {

  bacaSudutMPU6050();
  if(penambahanIMU == 1 && sudutIMU <= -90){
    pengaliIMU ++;
    penambahanIMU = 0;
  }else if(penambahanIMU == -1 && sudutIMU >= 90){
    pengaliIMU --;
    penambahanIMU = 0; 
  }
  if(sudutIMU >= 90){
    penambahanIMU = 1;
  }else if(sudutIMU <= -90){
    penambahanIMU =  -1;
  }
  sudutRobot = map(sudutIMU, -180, 180, (-180+(pengaliIMU*360)), (180+(pengaliIMU*360)));
  Serial.println(String(sudutIMU)+","+String(sudutRobot));
}

void panggilMPU6050(){
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(93);
  mpu.setYGyroOffset(-60);
  mpu.setZGyroOffset(117);
  mpu.setXAccelOffset(-840);
  mpu.setYAccelOffset(-1046);
  mpu.setZAccelOffset(1390);
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
}
//----------------------  FUNGSI BACA SUDUT MPU6050 --------------------------------
void bacaSudutMPU6050(){
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    sudutIMU = ypr[0] * 180 / M_PI;
//    Serial.print("ypr\t");
//    Serial.print(ypr[0] * 180 / M_PI);
//    Serial.print("\t");
//    Serial.print(ypr[1] * 180 / M_PI);
//    Serial.print("\t");
//    Serial.println(ypr[2] * 180 / M_PI);
//  Serial.println(sudutIMU);
  }
}
