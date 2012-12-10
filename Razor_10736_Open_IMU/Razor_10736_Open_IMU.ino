#include <Wire.h>

#define ToRad(x) ((x) * 0.01745329252)  // *pi/180
#define ToDeg(x) ((x) * 57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
#define betaDef		0.1f
#define betaDef2 1.6f

#define ADXL435_ADDR 0x53
#define ITG3200_ADDR 0x68
#define HMC5883_ADDR 0x1E
//accel defines
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define FIFO_CTL 0x38
#define DATAX0 0x32
//gyro defines
#define POWER_MGMT 0x3E
#define SRD 0x15
#define DLPF 0x16
#define GYRO_XOUT_H 0x1D
#define GYRO_TO_RAD 0.001214142f //(14.375^-1 * pi/180)
#define GYRO_TO_DEG 0.069565217f
//mag defines
#define CONFIG_REG_A 0x00
#define CONFIG_REG_B 0x01
#define MODE_REG 0x02
#define MAG_X_MSB 0x03

int16_t accX,accY,accZ;
int16_t gyroX,gyroY,gyroZ;
int16_t magX,magY,magZ;
float scaledGyroX,scaledGyroY,scaledGyroZ;
int16_t offsetX,offsetY,offsetZ;
int32_t gyroSumX,gyroSumY,gyroSumZ;

float smoothX,smoothY,smoothZ;
byte lsb,msb;

//Wire does not like 0x00
uint8_t HEX_ZERO = 0x00;

long timer, printTimer;
float G_Dt;
uint32_t loopCount = 0;

float q0;
float q1;
float q2;
float q3;
float beta = betaDef;
float beta2 = betaDef2;
float magnitude;

float pitch,roll,yaw;

void setup(){
  Serial.begin(115200);
  Serial.println("Start");
  Wire.begin(); 
  TWBR = ((F_CPU / 400000) - 16) / 2;
  Init();
  timer = millis();
}

void loop(){
  if (millis() - timer >= 3){
    G_Dt = (millis() - timer)/1000.0;
    timer=millis();

    if (loopCount == 3){
      GetMag();
      GetGyro();
      GetAcc();
      AHRSupdate(&G_Dt);
      loopCount = 0;

    }
    else{
      GetGyro();
      GetAcc();
      IMUupdate(&G_Dt);
      loopCount++;
    } 
  }
  
  if (millis() - printTimer > 50){
    printTimer = millis();
    GetEuler();
    Serial.print("$YPR");
    Serial.print(",");
    Serial.print(yaw);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(roll);
    Serial.println("*");
  }  
}


void Init(){
  Wire.beginTransmission(HMC5883_ADDR);
  Wire.write(MODE_REG);
  Wire.write(HEX_ZERO);
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883_ADDR);
  Wire.write(0x18);
  Wire.endTransmission();

  Wire.beginTransmission(ADXL435_ADDR);
  Wire.write(BW_RATE);
  Wire.write(0x0C);
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ADXL435_ADDR);
  Wire.write(POWER_CTL);
  Wire.write(0x08);
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ADXL435_ADDR);
  Wire.write(DATA_FORMAT);
  Wire.write(0x0B);
  Wire.endTransmission();

  Wire.beginTransmission(ITG3200_ADDR);
  Wire.write(POWER_MGMT);
  Wire.write(0x80);
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ITG3200_ADDR);
  Wire.write(DLPF);
  Wire.write(0x18);
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ITG3200_ADDR);
  Wire.write(SRD);
  Wire.write(HEX_ZERO);
  Wire.endTransmission();
  delay(5);
  Wire.beginTransmission(ITG3200_ADDR);
  Wire.write(POWER_MGMT);
  Wire.write(0x03);
  Wire.endTransmission();

  gyroSumX = 0;
  gyroSumY = 0;
  gyroSumZ = 0;
  for (int i =0; i<512; i++){
    GetGyro();//take samples to let the internal LPFs work
    GetAcc();
    delay(1);
  }
  for (int i =0; i<512; i++){
    GetGyro();
    GetAcc();
    gyroSumX += gyroX;
    gyroSumY += gyroY;
    gyroSumZ += gyroZ;
    delay(1);
  }
  offsetX = gyroSumX >> 9;
  offsetY = gyroSumY >> 9;
  offsetZ = gyroSumZ >> 9;

  GetMag();
  GetAcc();
  //calculate the initial quaternion 
  //these are rough values. This calibration works a lot better if the device is kept as flat as possible
  //find the initial pitch and roll
  pitch = ToDeg(fastAtan2(smoothX,sqrt(smoothY * smoothY + smoothZ * smoothZ)));
  roll = ToDeg(fastAtan2(-1*smoothY,sqrt(smoothX * smoothX + smoothZ * smoothZ)));

  if (smoothZ > 0){
    if (smoothX > 0){
      pitch = 180.0 - pitch;
    }
    else{
      pitch = -180.0 - pitch;
    }
    if (smoothY > 0){
      roll = -180.0 - roll;
    }
    else{
      roll = 180.0 - roll;
    }
  }  


  //tilt compensate the compass
  float xMag = (magX * cos(ToRad(pitch))) + (magZ * sin(ToRad(pitch)));
  float yMag = -1 * ((magX * sin(ToRad(roll))  * sin(ToRad(pitch))) + (magY * cos(ToRad(roll))) - (magZ * sin(ToRad(roll)) * cos(ToRad(pitch))));
  yaw = ToDeg(fastAtan2(yMag,xMag));

  if (yaw < 0){
    yaw += 360;
  }  
  //calculate the rotation matrix
  float cosPitch = cos(ToRad(pitch));
  float sinPitch = sin(ToRad(pitch));

  float cosRoll = cos(ToRad(roll));
  float sinRoll = sin(ToRad(roll));

  float cosYaw = cos(ToRad(yaw));
  float sinYaw = sin(ToRad(yaw));

  //need the transpose of the rotation matrix
  float r11 = cosPitch * cosYaw;
  float r21 = cosPitch * sinYaw;
  float r31 = -1.0 * sinPitch;

  float r12 = -1.0 * (cosRoll * sinYaw) + (sinRoll * sinPitch * cosYaw);
  float r22 = (cosRoll * cosYaw) + (sinRoll * sinPitch * sinYaw);
  float r32 = sinRoll * cosPitch;

  float r13 = (sinRoll * sinYaw) + (cosRoll * sinPitch * cosYaw);
  float r23 = -1.0 * (sinRoll * cosYaw) + (cosRoll * sinPitch * sinYaw);
  float r33 = cosRoll * cosPitch;



  //convert to quaternion
  q0 = 0.5 * sqrt(1 + r11 + r22 + r33);
  q1 = (r32 - r23)/(4 * q0);
  q2 = (r13 - r31)/(4 * q0);
  q3 = (r21 - r12)/(4 * q0); 

  float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

}
void GetMag(){
  Wire.beginTransmission(HMC5883_ADDR);
  Wire.write(MAG_X_MSB);
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883_ADDR);
  Wire.requestFrom(HMC5883_ADDR,6);

  msb = Wire.read();//X register
  lsb = Wire.read();
  magY = (((msb << 8) | lsb));

  msb = Wire.read();//Z register
  lsb = Wire.read();
  magZ = -1 * (((msb << 8) | lsb));

  msb = Wire.read();//Y register
  lsb = Wire.read();
  magX = (((msb << 8) | lsb));    
}
void GetAcc(){
  Wire.beginTransmission(ADXL435_ADDR);
  Wire.write(DATAX0);
  Wire.endTransmission();
  Wire.beginTransmission(ADXL435_ADDR);
  Wire.requestFrom(ADXL435_ADDR,6);
  lsb = Wire.read();
  msb = Wire.read();
  accY = -1 * ((msb << 8) | lsb);
  lsb = Wire.read();
  msb = Wire.read();
  accX = -1 * ((msb << 8) | lsb);
  lsb = Wire.read();
  msb = Wire.read();
  accZ = (msb << 8) | lsb;
  Smoothing(&accX,&smoothX);
  Smoothing(&accY,&smoothY);
  Smoothing(&accZ,&smoothZ);
}

void GetGyro(){
  Wire.beginTransmission(ITG3200_ADDR);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission();
  Wire.beginTransmission(ITG3200_ADDR);
  Wire.requestFrom(ITG3200_ADDR,6);

  msb = Wire.read();
  lsb = Wire.read();
  gyroY = (((msb << 8) | lsb) - offsetX);

  msb = Wire.read();
  lsb = Wire.read();
  gyroX = (((msb << 8) | lsb) - offsetY);

  msb = Wire.read();
  lsb = Wire.read();
  gyroZ = -1 * (((msb << 8) | lsb) - offsetZ);  

  scaledGyroX = GYRO_TO_RAD * gyroX;
  scaledGyroY = GYRO_TO_RAD * gyroY;
  scaledGyroZ = GYRO_TO_RAD * gyroZ;
}

void GetEuler(void){
  roll = ToDeg(fastAtan2(2 * (q0 * q1 + q2 * q3),1 - 2 * (q1 * q1 + q2 * q2)));
  pitch = ToDeg(asin(2 * (q0 * q2 - q3 * q1)));
  yaw = ToDeg(fastAtan2(2 * (q0 * q3 + q1 * q2) , 1 - 2* (q2 * q2 + q3 * q3)));
  if (yaw < 0){
    yaw +=360;
  }

}
void GYROupdate(float *dt){
  static float qDot1, qDot2, qDot3, qDot4;
  static float recipNorm;
  static float gx;
  static float gy;
  static float gz;

  gx = scaledGyroX;
  gy = scaledGyroY;
  gz = scaledGyroZ;

  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * *dt;
  q1 += qDot2 * *dt;
  q2 += qDot3 * *dt;
  q3 += qDot4 * *dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}
void IMUupdate(float *dt) {
  static float gx;
  static float gy;
  static float gz;
  static float ax;
  static float ay;
  static float az;

  static float recipNorm;
  static float s0, s1, s2, s3;
  static float qDot1, qDot2, qDot3, qDot4;
  static float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  gx = scaledGyroX;
  gy = scaledGyroY;
  gz = scaledGyroZ;

  ax = accX;
  ay = accY;
  az = accZ;
  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  magnitude = sqrt(ax * ax + ay * ay + az * az);
  if ((magnitude > 384) || (magnitude < 128)){
    ax = 0;
    ay = 0;
    az = 0;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * *dt;
  q1 += qDot2 * *dt;
  q2 += qDot3 * *dt;
  q3 += qDot4 * *dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void AHRSupdate(float *dt) {
  static float gx;
  static float gy;
  static float gz;
  static float ax;
  static float ay;
  static float az;
  static float mx;
  static float my;
  static float mz;


  static float recipNorm;
  static float s0, s1, s2, s3;
  static float qDot1, qDot2, qDot3, qDot4;
  static float hx, hy;
  static float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  gx = scaledGyroX;
  gy = scaledGyroY;
  gz = scaledGyroZ;

  ax = accX;
  ay = accY;
  az = accZ;

  mx = magX;
  my = magY;
  mz = magZ;
  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  magnitude = sqrt(ax * ax + ay * ay + az * az);

  if ((magnitude > 384) || (magnitude < 128)){
    ax = 0;
    ay = 0;
    az = 0;
  }

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {


    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;
    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;



    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta2 * s0;
    qDot2 -= beta2 * s1;
    qDot3 -= beta2 * s2;
    qDot4 -= beta2 * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * *dt;
  q1 += qDot2 * *dt;
  q2 += qDot3 * *dt;
  q3 += qDot4 * *dt;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}


void Smoothing(int16_t *raw, float *smooth){
  *smooth = (*raw * (0.15)) + (*smooth * 0.85);
}
float fastAtan2( float y, float x)
{
  static float atan;
  static float z;
  if ( x == 0.0f )
  {
    if ( y > 0.0f ) return PIBY2_FLOAT;
    if ( y == 0.0f ) return 0.0f;
    return -PIBY2_FLOAT;
  }
  //atan;
  z = y / x;
  if ( fabs( z ) < 1.0f )
  {
    atan = z/(1.0f + 0.28f*z*z);
    if ( x < 0.0f )
    {
      if ( y < 0.0f ) return atan - PI_FLOAT;
      return atan + PI_FLOAT;
    }
  }
  else
  {
    atan = PIBY2_FLOAT - z/(z*z + 0.28f);
    if ( y < 0.0f ) return atan - PI_FLOAT;
  }
  return atan;
}

float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}

















