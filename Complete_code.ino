
#include <SimpleFOC.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Ticker.h>
#include <math.h>

//Pins
#define PWM1_PIN   4
#define MIN_US_1   5
#define MAX_US_1   960

#define PWM2_PIN   15
#define MIN_US_2   5
#define MAX_US_2   960

//Motor
#define PP 7


const int U1=25, V1=26, W1=27, EN1=33;
const int U2=18, V2=19, W2=23, EN2=13;


int   SIGN_PITCH = +1;
int   SIGN_ROLL  = -1;
float Tilt_deg   = 15.0f;

//IMU
Adafruit_MPU6050 mpu;
Ticker ticker;


struct _1_ekf_filter { float LastP, Now_P, out, Kg, Q, R; };
typedef struct { int16_t accX, accY, accZ, gyroX, gyroY, gyroZ; } _st_Mpu;
typedef struct { float   roll, pitch, yaw; } _st_AngE;

volatile _st_Mpu  MpuRaw;
volatile _st_AngE Angle;


typedef volatile struct { float q0, q1, q2, q3; } Quaternion;
Quaternion NumQ = {1, 0, 0, 0};
volatile struct { float x, y, z; } GyroIntegError = {0,0,0};


#define squa(Sq) (((float)Sq) * ((float)Sq))
const float RtA     = 57.2957795f;  
const float Gyro_G  = 0.03051756f * 2; 
const float Gyro_Gr = 0.0005326f  * 2; 

int16_t raw_accel[3], raw_gyro[3];
float   MpuOffset[6] = {349.00f, -10.00f, -82.00f, -18.00f, -24.00f, 11.00f};


void readImuFiltered();
void getRawData(int16_t* accel, int16_t* gyro);
void mpuInit();
void kalman1(struct _1_ekf_filter *ekf, float input);
float Q_rsqrt(float number);
void GetAngle(const _st_Mpu *p, _st_AngE *a, float dt);


void onTick() {
  const float dt = 0.003f;

  readImuFiltered();

  _st_Mpu local;
  noInterrupts();
  local = MpuRaw;
  interrupts();

  _st_AngE temp;
  GetAngle(&local, &temp, dt);

  noInterrupts();
  Angle = temp;
  interrupts();
}

//SimpleFOC
MagneticSensorPWM s1(PWM1_PIN, MIN_US_1, MAX_US_1);
MagneticSensorPWM s2(PWM2_PIN, MIN_US_2, MAX_US_2);
void IRAM_ATTR isr_pwm1(){ s1.handlePWM(); }
void IRAM_ATTR isr_pwm2(){ s2.handlePWM(); }

BLDCMotor      m1(PP), m2(PP);
BLDCDriver3PWM d1(U1, V1, W1, EN1), d2(U2, V2, W2, EN2);

//Motor Config
void setupMotor(BLDCMotor& m) {
  m.controller        = MotionControlType::angle;
  m.torque_controller = TorqueControlType::voltage;

  m.P_angle.P = 30.0f;
  m.P_angle.I = 0.04f;

  m.PID_velocity.P = 1.7f;
  m.PID_velocity.I = 0.04f;
  m.PID_velocity.D = 0.0f;
  m.LPF_velocity.Tf = 0.010f;

  m.velocity_limit       = 110.0f;
  m.voltage_limit        = 10.0f;
  m.voltage_sensor_align = 6.0f;
  m.PID_velocity.output_ramp = 1500.0f;

  m.init();
}

void setup() {
  Wire.begin(21, 22);
  Serial.begin(115200);
  while (!Serial) {}

  mpuInit();
  ticker.attach_ms(3, onTick);

  pinMode(PWM1_PIN, INPUT);
  pinMode(PWM2_PIN, INPUT);
  s1.init(); s1.enableInterrupt(isr_pwm1);
  s2.init(); s2.enableInterrupt(isr_pwm2);

  // Drivers
  d1.voltage_power_supply = 11.0f;
  d1.voltage_limit        = 10.0f;
  d1.pwm_frequency        = 25000;
  d1.init(); d1.enable();

  d2.voltage_power_supply = 11.0f;
  d2.voltage_limit        = 10.0f;
  d2.pwm_frequency        = 25000;
  d2.init(); d2.enable();

  m1.linkSensor(&s1); m1.linkDriver(&d1);
  m2.linkSensor(&s2); m2.linkDriver(&d2);

  m1.sensor_direction = Direction::CW;
  m2.sensor_direction = Direction::CW;

  setupMotor(m1);
  setupMotor(m2);
  m1.initFOC();
  m2.initFOC();
}

static inline float clampf(float x,float a,float b){
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

void loop() {
  _st_AngE a;
  noInterrupts();
  a = Angle;
  interrupts();

  float pitch_cmd = (a.pitch * (float)M_PI / 180.0f) * SIGN_PITCH;
  float roll_cmd  = (a.roll  * (float)M_PI / 180.0f) * SIGN_ROLL;

  float tilt = Tilt_deg * (float)M_PI / 180.0f;
  float c = cosf(tilt), s = sinf(tilt);

  float pitch_out = pitch_cmd * c - roll_cmd * s + 2.5f;
  float roll_out  = roll_cmd + 0.6f;

  m1.loopFOC();
  m2.loopFOC();
  m1.move(pitch_out);
  m2.move(roll_out);

  static uint32_t t0 = 0;
  if (millis() - t0 > 10) {
    t0 = millis();
    float enc_pitch_deg = (s1.getAngle() - 2.5f) * 180.0f / (float)M_PI;
    float enc_roll_deg  = (s2.getAngle() - 0.5f) * 180.0f / (float)M_PI;
    Serial.printf("%.2f, %.2f, %.2f, %.2f\n", a.pitch, a.roll, enc_pitch_deg, -enc_roll_deg);
  }
}

//IMU
void readImuFiltered() {
  getRawData(raw_accel, raw_gyro);

  int16_t tmp[6];
  tmp[0]=raw_accel[0]; tmp[1]=raw_accel[1]; tmp[2]=raw_accel[2];
  tmp[3]=raw_gyro[0];  tmp[4]=raw_gyro[1];  tmp[5]=raw_gyro[2];

  for (uint8_t i = 0; i < 6; i++) {
    tmp[i] = (int16_t)((float)tmp[i] - MpuOffset[i]);

    if (i < 3) {
      static _1_ekf_filter ekf[3] = {
        {0.02f,0,0,0,0.001f,0.543f},
        {0.02f,0,0,0,0.001f,0.543f},
        {0.02f,0,0,0,0.001f,0.543f}
      };
      kalman1(&ekf[i], (float)tmp[i]);
      tmp[i] = (int16_t)ekf[i].out;
    } else {
      uint8_t k = i - 3;
      const float alpha = 0.15f;
      static float gLPF[3] = {0,0,0};
      gLPF[k] = gLPF[k]*(1.0f - alpha) + (float)tmp[i]*alpha;
      tmp[i]  = (int16_t)gLPF[k];
    }
  }

  noInterrupts();
  MpuRaw.accX = tmp[0]; MpuRaw.accY = tmp[1]; MpuRaw.accZ = tmp[2];
  MpuRaw.gyroX = tmp[3]; MpuRaw.gyroY = tmp[4]; MpuRaw.gyroZ = tmp[5];
  interrupts();
}

void getRawData(int16_t* accel, int16_t* gyro) {
  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  accel[0] = (Wire.read()<<8) | Wire.read();
  accel[1] = (Wire.read()<<8) | Wire.read();
  accel[2] = (Wire.read()<<8) | Wire.read();
  Wire.read(); Wire.read(); // skip temp
  gyro[0]  = (Wire.read()<<8) | Wire.read();
  gyro[1]  = (Wire.read()<<8) | Wire.read();
  gyro[2]  = (Wire.read()<<8) | Wire.read();
}

void mpuInit() {
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);

  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x19); Wire.write(0x02); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x03); Wire.endTransmission();
}

void kalman1(struct _1_ekf_filter *ekf, float input) {
  ekf->Now_P = ekf->LastP + ekf->Q;
  ekf->Kg    = ekf->Now_P / (ekf->Now_P + ekf->R);
  ekf->out   = ekf->out + ekf->Kg * (input - ekf->out);
  ekf->LastP = (1.0f - ekf->Kg) * ekf->Now_P;
}

float Q_rsqrt(float number) {
  long i; float x2, y; const float threehalfs = 1.5F;
  x2 = number * 0.5F; y = number;
  i = *(long *)&y; i = 0x5f3759df - (i >> 1); y = *(float *)&i;
  y = y * (threehalfs - (x2 * y * y));
  return y;
}

static float NormAccz;
void GetAngle(const _st_Mpu *pMpu, _st_AngE *pAngE, float dt) {
  volatile struct { float x,y,z; } Gravity, Acc, Gyro, AccGravity;
  static float KpDef = 0.5f, KiDef = 0.00015f;

  float HalfTime = dt * 0.5f;

  Gravity.x = 2.0f * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
  Gravity.y = 2.0f * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
  Gravity.z = 1.0f - 2.0f * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

  float NormAcc = Q_rsqrt(squa(pMpu->accX) + squa(pMpu->accY) + squa(pMpu->accZ));
  Acc.x = pMpu->accX * NormAcc;
  Acc.y = pMpu->accY * NormAcc;
  Acc.z = pMpu->accZ * NormAcc;

  AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
  AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
  AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

  GyroIntegError.x += AccGravity.x * KiDef;
  GyroIntegError.y += AccGravity.y * KiDef;
  GyroIntegError.z += AccGravity.z * KiDef;

  Gyro.x = pMpu->gyroX * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x;
  Gyro.y = pMpu->gyroY * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
  Gyro.z = pMpu->gyroZ * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;

  float q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
  float q1_t = ( NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
  float q2_t = ( NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
  float q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

  NumQ.q0 += q0_t; NumQ.q1 += q1_t; NumQ.q2 += q2_t; NumQ.q3 += q3_t;

  float NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
  NumQ.q0 *= NormQuat; NumQ.q1 *= NormQuat; NumQ.q2 *= NormQuat; NumQ.q3 *= NormQuat;

  float vecxZ =  2.0f * NumQ.q1 * NumQ.q3 - 2.0f * NumQ.q0 * NumQ.q2;
  float vecyZ =  2.0f * NumQ.q2 * NumQ.q3 + 2.0f * NumQ.q0 * NumQ.q1;
  float veczZ =  1.0f - 2.0f * NumQ.q1 * NumQ.q1 - 2.0f * NumQ.q2 * NumQ.q2;

  float yaw_G = pMpu->gyroZ * Gyro_G;
  if (yaw_G > 1.0f || yaw_G < -1.0f) {
    pAngE->yaw += yaw_G * dt;   
  }

  pAngE->pitch = -asinf(vecxZ) * RtA;
  pAngE->roll  =  atan2f(vecyZ, veczZ) * RtA;

  NormAccz = pMpu->accX * vecxZ + pMpu->accY * vecyZ + pMpu->accZ * veczZ;
}
