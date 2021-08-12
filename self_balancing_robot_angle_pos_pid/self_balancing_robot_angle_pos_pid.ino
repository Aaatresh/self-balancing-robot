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
 
//Define three-axis acceleration, three-axis gyroscope variables
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
float dt = 0.005;
 
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

void pos_pid_update_routine();
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

float kp = 32.0,kd = 0.70,ki = 0.80;
double sum_angle_error = 0;
//////////////////////////////////////////////////////

///////////////////////// POS PID ////////////////////

int pos_flag = 0;

// Position set point
int setp_position = 0;

float speed_filtered_old, speed_filtered, speeds;
float error = 0.0, error_old = 0.0;

float kp_pos = 0.8, ki_pos = 0.8, kd_pos = 0.3;
int pos_pid = 0;
int pos = 0;
int cc = 0;

///////////////////////////////////////////////////////////

////////////////// motor speed counter ////////////////////

// Motors to read the encoder to calculate the speed of the wheels
volatile long countl, countr; 
long pulseright, pulseleft;

//////////////////////////////////////////////////////////

////////////////// turning control var ////////////////////

/*
  In order to turn, turn_left_pwm and turn_right_pwm variables are used to scale down the pwm values assigned to the left and right wheels respectively by multiplying them. 
  
  For values of 1.0, the pwm values are not scaled down and the wheels move at their assigned speed. For values less than one but greater than zero, they run at slower speeds, 
  leading to a turning effect in the direction of the slower wheel.
*/
float turn_left_pwm = 1.0;
float turn_right_pwm = 1.0;

//////////////////////////////////////////////////////////


// Setup method called once during program execution
void setup() {
  
  // output pins
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(11,OUTPUT);
  
  // input pins
  pinMode(7, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  
  // Initialize I2C communication
  Wire.begin(); 
 
  // Set the baud rate
  Serial.begin(115200);   
 
  // Give 1.5s for the communication interface to initialize correctly
  delay(1500);
 
  // Initialize the MPU6050
  mpu6050.initialize();                       
 
  
  Serial.println("----------------------------");
  Serial.println("initialization complete");
  Serial.println("----------------------------");

  // 50ms for final initialization time before timer-1 interrupt initialization
  delay(50);

  Timer1.initialize(5000);
  Timer1.attachInterrupt(isr_main); 
  
  // Add pin change interrupts to read the square wave encoder outputs to estimate the speed of each wheel
  attachPinChangeInterrupt(7,count_left_isr,CHANGE);
  attachPinChangeInterrupt(A2,count_right_isr,CHANGE);
  
}


// Loop method is an infinite loop that runs after the setup() method is called.
void loop() {
  
  // Reattach pin change interrupts
  attachPinChangeInterrupt(7,count_left_isr,CHANGE);
  attachPinChangeInterrupt(A2,count_right_isr,CHANGE);
  
}

// Left wheel encoder pulse counter
void count_left_isr()
{
   countl++; 
}

// Right wheel encoder pulse counter
void count_right_isr()
{
   countr++; 
}

// Method to accumulate the pulses from encoders in both wheels for a speed estimate
void count_pulses()
{
     
   if(pwm_pid < 0)
   {
      countl = -1 * countl; 
      countr = -1 * countr; 
   }
       
   pulseright += countr;
   pulseleft += countl;
   
   countr = countl = 0;
}

// Method to update the position PID algorithm
void pos_pid_update_routine()
{
   // Estimate the speed from right and left wheels
   speeds = (pulseleft + pulseright) * 1.0;
   
   pulseleft = pulseright = 0;
  
   // Filter the speed using a first order filter -- More preference is given to the previous speed estimate to reduce the impact of spikes in readings 
   speed_filtered_old = speed_filtered_old * 0.7;
   speed_filtered = speed_filtered_old + (speeds * 0.3);
   speed_filtered_old = speed_filtered;
   
   // Running sum of position for the integral step
   pos = pos + speed_filtered;
   
   // Error in position
   error = setp_position - speed_filtered;
   
   // Limit position
   pos = constrain(pos, -3550, 3550);
   
   // Estimate position PID
   pos_pid = (kp_pos * (setp_position - speed_filtered)) + (ki_pos * (setp_position - pos)) + (kd_pos * (error - error_old));
   
   error_old = error;
   
   Serial.println(pos_pid);
}

// Method to update the pwm values for the wheels by combining the angle and position PIDs.
void pwm_update_routine()
{
  // Combining both angle and position PIDs.
  pwm_pid = pwm_pid - (pos_flag * pos_pid);
  
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

void pid_update()
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
  
  // count pulses from each wheel and get a speed estimate
  count_pulses();
  
  // Use I2C to get MPU6050 six-axis data  ax ay az gx gy gz
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 
  // Obtain angle and Kalman Filter 
  angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);
  
  // update angle pid
  pid_update();
  
  // update speed pid
  cc++;
  if(cc == 8)
  {
    pos_pid_update_routine();
    cc = 0;
  }
  
  // update pwm values for motors
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
