/*
* Author: Anirudh Aatresh
* Credit to automaticaddison.com for the kalman filter implementation. 
* Email ID: anirudhashokaatresh@gmail.com
* Homepage: aaatresh.github.io
*/


///////////////////// Including all relevant libraries //////////
#include "I2Cdev.h"
#include "MPU6050.h"
#include<Wire.h>
#include<TimerOne.h>
#include<PinChangeInt.h>
/////////////////////////////////////////////////////////////////


// Global object to handle IMU measurements
MPU6050 mpu6050;     
 
// Define three-axis acceleration, three-axis gyroscope variables
int16_t ax, ay, az, gx, gy, gz; 
 
// The tilt angle
float Angle;
 
// Angular velocity along each axis as measured by the gyroscope
// The units are degrees per second.
float Gyro_x,Gyro_y,Gyro_z;  
 
/////////////////////// Kalman_Filter ////////////////////////////

// Covariance of gyroscope noise
volatile float Q_angle = 0.001;  
 
// Covariance of gyroscope drift noise
volatile float Q_gyro = 0.003;  
 
// Covariance of accelerometer
volatile float R_angle = 0.5;    
volatile char C_0 = 1;
 
// The filter sampling time.
float dt = 0.008;
 
// a function containing the Kalman gain is used to 
// calculate the deviation of the optimal estimate.
float K1 = 0.05; 
float K_0,K_1,t_0,t_1;
float angle_err;
 
// Gyroscope drift 
float q_bias;    
 
float accelz = 0;
float angle;
float angle_speed;
 
float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float  PCt_0, PCt_1, E;
//////////////////////////////////////////////////////////



////////////////// function definitions //////////////

void isr_main();
void pid_update();
void pwm_update_routine();

void speed_pid_update();
void count_left_isr();
void count_right_isr();
void count_pulses();

//////////////////////////////////////////////////////

///////////////////////// ANGLE PID //////////////////
unsigned long old_time, old_time1 = 0;
int pwm_pid,pwm_l,pwm_r;

// Set point angle of the robot. This is the angle which the robot will attempt to maintain when standing still. Any deviations from this set point will result in forward or 
// backward movement.
float setp_angle = -8.0;

float kp = 18.0,kd = 0.7,ki = 0.7;
double sum_angle_error = 0;
//////////////////////////////////////////////////////

////////////////// turning control var ////////////////////

/*
  In order to turn, turn_left_pwm and turn_right_pwm variables are used to scale down the pwm values assigned to the left and right wheels respectively by multiplying them. 
  
  For values of 1.0, the pwm values are not scaled down and the wheels move at their assigned speed. For values less than one but greater than zero, they run at slower speeds, 
  leading to a turning effect in the direction of the slower wheel.
*/

float turn_left_pwm = 1.0;
float turn_right_pwm = 1.0;

//////////////////////////////////////////////////////////

////////////////// mobile app interface variables ///////////////

// A working variable to store incoming serial data 
char c;
/////////////////////////////////////////////////////////

// Setup method called once during program execution
void setup() {
  
  // output pins
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(11,OUTPUT);
  
  // Initialize I2C communication
  Wire.begin(); 
 
  // Set the baud rate
  Serial.begin(9600);   
 
  // Give 1.5s for the communication interface to initialize correctly
  delay(1500);
 
  // Initialize the MPU6050
  mpu6050.initialize();                       
  
  old_time = 0;
  old_time1 = 0;
  
  
  //Serial.println("----------------------------");
  //Serial.println("initialization complete");
  //Serial.println("----------------------------");

  // 50ms for final initialization time before timer-1 interrupt initialization
  delay(50);

  Timer1.initialize(5000);
  Timer1.attachInterrupt(isr_main); 
  
}


// Loop method is an infinite loop that runs after the setup() method is called.
void loop() {
  
  
  // Read characters from the serial port and take an appropriate decision
  if(Serial.available() > 0)
  {
     c = Serial.read();
    
     switch(c)
     {
        case 'f':
          // Move forward command
          setp_angle = setp_angle + 0.1;
          break;
        case 'b':
          // Move backward command
          setp_angle = setp_angle - 0.1;
          break;
        case 'r':
          // Turn right command
          if(turn_left_pwm < 1.0)
            turn_left_pwm = turn_left_pwm + 0.1;
          else if(turn_right_pwm > 0.5)
            turn_right_pwm = turn_right_pwm - 0.1;
            
          break;
        case 'l':
          // Turn left command
          if(turn_right_pwm < 1.0)
            turn_right_pwm = turn_right_pwm + 0.1;
          else if(turn_left_pwm > 0.5)
            turn_left_pwm = turn_left_pwm - 0.1;
          
          break;  
     } 
  }
  
  
}



void pwm_update_routine()
{
  
  if(pwm_pid > 255)
  {
     pwm_pid = 255;
  } 
  else if(pwm_pid < -255)
  {
     pwm_pid = -255; 
  }
  
  // scale down pwm values when turning
  pwm_l = pwm_pid * turn_left_pwm;
  pwm_r = pwm_pid * turn_right_pwm;
  
  // check direction of pwm value and assign pwm values of corresponding pins 
  if(pwm_l >= 0)
  {
     analogWrite(5,pwm_l);
     digitalWrite(3,0); 
  }
  else 
  {
     pwm_l = (-1) * pwm_l;
     digitalWrite(5,0);
     analogWrite(3,pwm_l); 
  }
  
  if(pwm_r >= 0)
  {
     analogWrite(6,pwm_r);
     digitalWrite(11,0); 
  }
  else 
  {
     pwm_r = (-1) * pwm_r;
     digitalWrite(6,0);
     analogWrite(11,pwm_r); 
  }
  
}

void pid_update_routine()
{
   // Maintain running sum for integral update
   sum_angle_error = sum_angle_error + (angle - setp_angle);
   
   // calculate pwm value through a pid calculation step
   pwm_pid = (kp * (angle - setp_angle)) + (ki * sum_angle_error) + (kd * angle_speed); 
}

void isr_main()
{
  // Enable global interrupts
  sei();
  
  // Use I2C to get MPU6050 six-axis data  ax ay az gx gy gz
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 
  // Obtain angle and Kalman Filter 
  angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  
  // update angle pid values through IMU readings 
  pid_update_routine();
  
  // update pwm values for both motors
  pwm_update_routine();
}

/////////////////////////////angle calculate///////////////////////
void angle_calculate(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0,float K1)
{
  // Radial rotation angle calculation formula; negative sign is direction processing
  Angle = -atan2(ay , az) * (180/ PI); 
 
  // The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
  Gyro_x = -gx / 131;      
 
  // KalmanFilter 
  Kalman_Filter(Angle, Gyro_x);            
}
////////////////////////////////////////////////////////////////
 
///////////////////////////////KalmanFilter/////////////////////
void Kalman_Filter(double angle_m, double gyro_m)
{
  // Prior estimate
  angle += (gyro_m - q_bias) * dt;          
  angle_err = angle_m - angle;
 
  // Differential of azimuth error covariance
  Pdot[0] = Q_angle - P[0][1] - P[1][0];    
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
 
  // The integral of the covariance differential of the prior estimate error
  P[0][0] += Pdot[0] * dt;    
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
   
  // Intermediate variable of matrix multiplication
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
   
  // Denominator
  E = R_angle + C_0 * PCt_0;
   
  // Gain value
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
 
  // Intermediate variable of matrix multiplication
  t_0 = PCt_0;  
  t_1 = C_0 * P[0][1];
 
  // Posterior estimation error covariance
  P[0][0] -= K_0 * t_0;     
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
 
  // Posterior estimation
  q_bias += K_1 * angle_err;    
 
  // The differential value of the output value; work out the optimal angular velocity
  angle_speed = gyro_m - q_bias;   
 
  ////Posterior estimation; work out the optimal angle
  angle += K_0 * angle_err; 
}
