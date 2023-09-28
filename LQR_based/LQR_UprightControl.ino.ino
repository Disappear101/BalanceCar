#include <Wire.h>
#include <Zumo32U4.h>
#include "LQR_Controller.h"

#define Sensitivity_Gyro 8.75
#define Sensitivity_Acc 0.061
//sample period 
#define Ts 0.05
#define TAU 0.49
#define r 19.5e-3
//m per circle
#define mpc 1.3922967e-4

//sample period:0.1
//float A[] = {1, 0.1, -0.580223740512393, -0.014977891463680,
//            0, 1, 19.211944838014490, 0.580223740512393,
//            0, 0, 10.025702630192775, 0.332989422768348,
//            0, 0, 2.98852475258003e2, 10.025702630192773};

//float B[] = {0.437143092004022, 0.437143092004022,
//            20.201098187052665, 20.201098187052665,
//            13.594353123544150, 13.594353123544150,
//            4.501262945348955e2, 4.501262945348955e2};

//float F[] = {-1.437847149083173e-05, -2.810610173765218e-04, 0.339771550688593, 0.011358839207432, 1.437847149083173e-05, 2.782536543098856e-04, -6.387504154186516e-05, -7.077701894702613e-04,
//            -1.437847149083173e-05, -2.810610173765218e-04, 0.339771550688593, 0.011358839207432, 1.437847149083173e-05, 2.782536543098856e-04, -6.387504154186516e-05, -7.077701894701497e-04};

//sample period:0.05
float A[] = {1, 0.0500000000000000, -0.0254028365991793, -0.000413164595671624,
             0, 1, -1.07827337782434, -0.0254028365991793,
             0, 0, 1.39515523598723, 0.0564270048215586,
             0, 0, 16.7731414328231, 1.39515523598723};

float B[] = {-0.00202539641780109, -0.00202539641780109,
             -0.174640015819246, -0.174640015819246,
             0.595175803671824, 0.595175803671824,
             25.2634079552072, 25.2634079552072};

float F[] = {-3.45485113752267e-05, -0.000671836383321564, 0.352010588659830, 0.0261373863651278,
             -3.45485113752267e-05, -0.000671836383321564, 0.352010588659830, 0.0261373863651279};


float xd[] = {0, 0, 0, 0};

float x[] = {0, 0, 0, 0};

float u[] = {0, 0};


int Motor1 = 0, Motor2 = 0;
int n = 4, p = 2;

Zumo32U4LCD display;
Zumo32U4IMU imu;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
LQR_Controller lqr(A, B, F, x, xd, u, n, p);

//angle from acc
float aAngle;
//angle from gyro
float gAngle;
//fused angle
float Angle_Y;

//angular speed from gyro 
float gyroY;

int Gyro_Zero_Offset;

float LeftVelocity, RightVelocity, Displacement;

void IMUInitiation();
void UpdateEncoderData();
void UpdateAngleAcc();
void ComplementaryFilter(float acc_Angle, float gyro_Y, float dt);

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  IMUInitiation();

  
  
  for (uint16_t i = 0; i < 1024; i++)
  {
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Y axis reading to the total.
    Gyro_Zero_Offset += imu.g.y;
  }
  Gyro_Zero_Offset /= 1024;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  static byte lastCorrectionTime = 0;
  byte m = millis();
  if ((byte)(m - lastCorrectionTime) >= Ts*1000)
  {
    lastCorrectionTime = m;
   
    UpdateAngleAcc();
    
    UpdateAngleGyroAndFusion();

    display.clear();
    display.print("An:");
    //display.print(Angle_Y);
    if (abs(Angle_Y) < 55)
    {
      UpdateEncoderData();
      x[0] = Displacement;
      x[1] = LeftVelocity;
      x[2] = Angle_Y * PI / 180;
      x[3] = gyroY * PI / 180;
//      Serial.println("--------------------------");
      Serial.println(x[0]);
      Serial.println(x[1]);
      Serial.println(x[2]);
      Serial.println(x[3]);

      float* control_input = lqr.Cal_Control_Input();

      Motor1 = 1e5 * control_input[0] / 1.5;
      Motor2 = 1e5 * control_input[1] / 1.5;
//      display.gotoXY(0, 1);
//      display.print("PWM:");
//      display.print(1e5*control_input[0]);
      Motor1 = constrain(Motor1, -400, 400);
      Motor2 = constrain(Motor2, -400, 400);
      
    }
    else
    {
      Motor1 = 0;
      Motor2 = 0;
    }
    motors.setSpeeds(Motor1, Motor2);
  }
  
}

void IMUInitiation()
{
  if (!imu.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to initialize IMU sensors."));
      delay(1000);
    }
  }

  imu.enableDefault();
}

void UpdateEncoderData()
{
  static uint16_t lastUpdateTime = 0;
  static int lastLeftCount = 0, lastRightCount = 0;
  uint16_t m = micros();
  uint16_t dt = m - lastUpdateTime;
  lastUpdateTime = m;
  
  int16_t countsLeft = encoders.getCountsLeft();
  int16_t countsRight = encoders.getCountsRight();

  Displacement = countsLeft*mpc;

  LeftVelocity = mpc * 1000000 * (countsLeft - lastLeftCount) / dt;
  RightVelocity = mpc * 1000000 * (countsRight - lastRightCount) / dt;

  //Serial.println("--------------------------");
  //Serial.println(countsLeft);
  //Serial.println(lastLeftCount);
  //Serial.println(dt);
  //Serial.println(LeftVelocity);

  lastLeftCount = countsLeft;
  lastRightCount = countsRight;
}

void UpdateAngleAcc()
{
  //updata acc data and calculate angle
  imu.readAcc();
  aAngle = -atan2(imu.a.z, -imu.a.x) * 180 / M_PI;
}

void UpdateAngleGyroAndFusion()
{
  //updata gyro data and calculate angle
  static uint16_t lastUpdate = 0;
  uint16_t m = micros();
  uint16_t dt = m - lastUpdate;
  lastUpdate = m;
  
  imu.readGyro();
  //gyroY = imu.g.y;
  //float gyroX = ((float)imu.g.x - Gyro_Zero_Offset) * Sensitivity_Gyro / 1000;

  gyroY = ((float)imu.g.y - Gyro_Zero_Offset) * Sensitivity_Gyro / 1000;

  ComplementaryFilter(aAngle, gyroY, Ts);
}

/*ComplementaryFilter
 * a = tau/(tau+dt)
*/
void ComplementaryFilter(float acc_Angle, float gyro_Y, float dt)
{
  float a = TAU / (TAU + dt);
  //Serial.println("--------------------------");
  //Serial.println(dt);
  //Serial.println(a);
  Angle_Y = a * (Angle_Y + gyro_Y * dt) + (1 - a) * acc_Angle;
}

void MatrixMultiplication(const float *left, const float* right, float *res, int row, int col1, int col2)
{
  int i = 0, j = 0;
  for (int idx = 0; idx < row * col2; idx++)
  {
    i = idx / col2;
    j = idx - i * col2;
    for (int k = 0; k < col1; k++)
    {
      res[idx] = res[idx] + left[i * col1 + k] * right[j + k * col2];
    }
  }
}
