#include <Wire.h>
#include <Zumo32U4.h>

// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
// Zumo32U4LCD display;
Zumo32U4LCD display;
Zumo32U4IMU imu;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;

float Target_Speed = 0;
//mechnaical balance degree
float Med_Angle = 0;

/* First Adjustment of parameters
float Vertical_Kp = 70 * 0.6,//70
      Vertical_Kd = 0.12 * 0.6;//0.1,     0.12        0.15
float Coeff = -0.015;//     -0.015(200), -0.015(200) -0.16(200), 
float Velocity_Kp = Coeff,
      Velocity_Ki = Coeff / 250;
      //comment:intergral term is too big, so i need to consider reduce Velocity_Ki
float Turn_Kp = 0.8;//need modify
*/

/* Second Adjustment of parameters*/
//vertical loop KD controller
float Vertical_Kp = 60 * 0.6,
      Vertical_Kd = 0.8 * 0.6;

//velocity loop KI controller
float Coeff = -0.025;//   
float Velocity_Kp = Coeff,
      Velocity_Ki = Coeff / 200;
float Turn_Kp = 1.2;


//angle from acc
float aAngle;
//angle from gyro
float gAngle;
//fused angle by ComplementaryFilter
float Angle_Y;

//output of Velocity loop
float Velocity_out;
//output of vertical loop
int Vertical_out;
//output of turning loop
int Turn_out;
//PWM value for control input
int PWM_out;
//left and right pwm input
int Motor1 = 0, Motor2 = 0;
//gyro zero offset
int Gyro_Zero_Offset;
int LeftVelocity, RightVelocity;
//velocity around X-axis and Y-axis from gyro
float gyroY, gyroX;

void IMUInitiation();
void TimerInit();
void UpdateAngleAcc();
void UpdateAngleGyroAndFusion();
void UpdateEncoderData();
int Vertical(float Med, float Angle, float gyro_y);
float Velocity(int Target, int V_left, float V_right);
int Turn(float gyro_X);
void Limit(int & motor1, int& motor2);
void ComplementaryFilter(float acc_Angle, float gyro_Y, float dt);

#define Sensitivity_Gyro 8.75
#define Sensitivity_Acc 0.061

void setup()
{
  Wire.begin();
  IMUInitiation();

  //calculate Gyro Zero Offset
  for (uint16_t i = 0; i < 1024; i++)
  {
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    Gyro_Zero_Offset += imu.g.y;
  }
  Gyro_Zero_Offset /= 1024;
}

void loop()
{
  static byte lastCorrectionTime = 0;
  byte m = millis();
  //sample period 10ms
  if ((byte)(m - lastCorrectionTime) >= 10)
  {
    lastCorrectionTime = m;
    display.clear();
    UpdateAngleAcc();
    
    UpdateAngleGyroAndFusion();
    display.print("A:");
    display.print(Angle_Y);
    display.gotoXY(0, 1);
    display.print("aA:");
    display.print(aAngle);
    
    if (abs(Angle_Y) < 55)
    {
      UpdateEncoderData();
      Velocity_out = Velocity(Target_Speed, LeftVelocity, RightVelocity);
      display.print(Velocity_out);
      //Serial.println(Velocity_out);
      Vertical_out = Vertical(Velocity_out + Med_Angle, Angle_Y, gyroY);
      Turn_out = Turn(gyroX);

      PWM_out = Vertical_out;
      Motor1 = PWM_out - Turn_out;
      Motor2 = PWM_out + Turn_out;
      //Serial.println(Motor1);
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
  
  
  
  
  //display.print("ASpeed:");
  //display.gotoXY(0, 1);
  //display.print(angle_speed);

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
  gyroX = ((float)imu.g.x - Gyro_Zero_Offset) * Sensitivity_Gyro / 1000;

  gyroY = ((float)imu.g.y - Gyro_Zero_Offset) * Sensitivity_Gyro / 1000;

  ComplementaryFilter(aAngle, gyroY, 0.01);
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

  LeftVelocity = 100000 * (countsLeft - lastLeftCount) / dt;
  RightVelocity = 100000 * (countsRight - lastRightCount) / dt;

  //Serial.println("--------------------------");
  //Serial.println(countsLeft);
  //Serial.println(-lastLeftCount);
  //Serial.println(dt);
  //Serial.println(LeftVelocity);

  lastLeftCount = countsLeft;
  lastRightCount = countsRight;
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

void TimerInit()
{
    
}

/*********************************************
 * Vertical loop
 * Kp*Ek+Kd*Ek_D
 * arguments: mechanical median(expected angle),    
 *            measured angle(real angle
 *            measured angular speed
 * output:    PWM value
 **********************************************/
int Vertical(float Med, float Angle, float gyro_y)
{
  int PWM_out;

  PWM_out = Vertical_Kp*(Angle - Med) + Vertical_Kd*(gyro_y - 0);
  return PWM_out;
}

/*********************************************
 * Velocity loop
 * Kp*Ek+Kd*Ek_I
 * arguments: Target speed,    
 *            left wheel speed
 *            right wheel speed
 * output:    input of vertical loop
 **********************************************/
float Velocity(int Target, int V_left, float V_right)
{
  static int V_Error_s, V_Error_filtered, V_Error, V_Error_Last, V_Error_filtered_Last;
  float Angle_out;
  float a = 0.7;

  //cal difference
  V_Error = (V_left + V_right) - Target;

  //low filter(avoid high frequencu affecting D componnet in Vertical ring)
  V_Error_filtered = (1-a) * V_Error + a *  V_Error_filtered_Last;
  V_Error_filtered_Last = V_Error_filtered;

  //inregral
  V_Error_s += V_Error_filtered;

  //magnitude limitation of integral
  V_Error_s = constrain(V_Error_s, -50000, 50000);
  Serial.println(V_Error_s);

  //velocity output
  Angle_out = Velocity_Kp*V_Error_filtered + Velocity_Ki*V_Error_s;
  return Angle_out;
}

//turn ring
int Turn(float gyro_X)
{
  int PWM_out;

  PWM_out = Turn_Kp*gyro_X;

  return PWM_out;
}

void Limit(int* varible, int DownLimit, int UpLimit)
{
  *varible = *varible > UpLimit ? UpLimit : (*varible < (DownLimit) ? (DownLimit) : *varible);
}



/*ComplementaryFilter
 * a = tau/(tau+dt)
*/
void ComplementaryFilter(float acc_Angle, float gyro_Y, float dt)
{
  float tau = 0.49;
  float a = tau / (tau + dt);
  //Serial.println("--------------------------");
  //Serial.println(dt);
  //Serial.println(a);
  Angle_Y = a * (Angle_Y + gyro_Y * dt) + (1 - a) * acc_Angle;
}
