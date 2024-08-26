#include <ArduinoHardware.h>
#include "CytronMotorDriver.h"
#include <ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <Encoder.h>
#include "batly_msgs/Imu.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include <sensor_msgs/MagneticField.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define COMMAND_RATE 20
#define IMU_PUBLISH_RATE 20 //hz
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

#define ACCEL_SCALE 1 / 16384 // LSB/g
#define GYRO_SCALE 1 / 131 // LSB/(deg/s)
#define MAG_SCALE 0.3 // uT/LSB
#define G_TO_ACCEL 9.81


CytronMD motor1(PWM_PWM, 8, 9);  //LF_PM1A, LF_PM1B
CytronMD motor2(PWM_PWM, 7, 6); //RF_PM1A, RF_PM1B
CytronMD motor3(PWM_PWM, 4, 5); //LB_PM1A, LB_PM1B
CytronMD motor4(PWM_PWM, 3, 2); //RB_PM1A, RB_PM1B

char temp;
char buffer[6];
int i = 0;
long encPos_A;
long encPos_B;
long encPos_C;
long encPos_D;

char log_msg[100];

Encoder Enc_A(15, 14); //LF_enA, LF_enB
Encoder Enc_B(12, 13); //RF_enA, RF_enB
Encoder Enc_C(17, 16); //LB_enA, LB_enB
Encoder Enc_D(10, 11); //RB_enA, RB_enB


float setpoint1, setpoint2, setpoint3, setpoint4;
float output1, output2, output3, output4;
float w1, w2, w3, w4;
float vel1, vel2, vel3, vel4;
float Kp = 0, Ki = 0, Kd = 0;
float Op = 0, Oi = 0, Od = 0;
float dt = 0.05;  //s
float error, error_n, error_p;
float I_sum;
float r = 0.0500;   //m
float lx = 0.2100;  //m
float ly = 0.1475;  //m
double pi = 3.1428;
double ppr = 1441;
float Vx, Vy, Wz;
float prev_setpoint = 0;
unsigned long prev_control_time = 0;
unsigned long g_prev_command_time = 0;
unsigned long prev_debug_time = 0;
unsigned long prev_imu_time = 0;



void commandCallback(const geometry_msgs::Twist& cmd_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("/cmd_vel", commandCallback);

ros::NodeHandle nh;
geometry_msgs::Twist msg;
geometry_msgs::Twist enc;
ros::Publisher Enc("raw_vel", &enc);
sensor_msgs::Imu raw_imu_msgs;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msgs);


void setup() {
  delay(2000);
  Wire.begin();
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");


  Kp = 70;
  Ki = 5;
  Kd = 1;
  

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(Enc);
  nh.advertise(raw_imu_pub);
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("BATLYBOT CONNECTED");
  delay(1);
}
void loop() {
  
  if ((millis() - prev_control_time) >= 50)
  {
    //input_cmd();
    float check_error1 = w1 - vel1;
    float check_error2 = w2 - vel2;
    float check_error3 = w3 - vel3;
    float check_error4 = w4 - vel4;
    
    char result4[8];
    
    dtostrf(check_error4, 6, 2, result4); // Leave room for too large numbers!
    sprintf(log_msg, "check_error4 = %s", result4);
    nh.loginfo(log_msg);

    char result4_1[8];
    dtostrf(w4, 6, 2, result4_1); // Leave room for too large numbers!
    sprintf(log_msg, "w4 = %s", result4_1);
    nh.loginfo(log_msg);


    movebase();
    encoder();
    prev_control_time = millis();

  }

  if ((millis() - g_prev_command_time) >= 400 )
  {
    stopBase();

  }
  if ((millis() - prev_imu_time) >= 50)
  {

    publishIMU();
    prev_imu_time = millis();  //nh.loginfo("IMU publshed")
  }

  nh.spinOnce();
}

void commandCallback(const geometry_msgs::Twist& cmd_msg)

{
  Vx = cmd_msg.linear.x;
  Vy = cmd_msg.linear.y;
  Wz = cmd_msg.angular.z;

  w1 = (Vx - Vy - (lx + ly) * Wz) * (1 / r);
  w2 = (Vx + Vy + (lx + ly) * Wz) * (1 / r);
  w3 = (Vx + Vy - (lx + ly) * Wz) * (1 / r);
  w4 = (Vx - Vy + (lx + ly) * Wz) * (1 / r);

  g_prev_command_time = millis();
}

void input_cmd()
{

  w1 = (Vx - Vy - (lx + ly) * Wz) * (1 / r);
  w2 = (Vx + Vy + (lx + ly) * Wz) * (1 / r);
  w3 = (Vx + Vy - (lx + ly) * Wz) * (1 / r);
  w4 = (Vx - Vy + (lx + ly) * Wz) * (1 / r);

  setpoint1 = w1;
  setpoint2 = w2;
  setpoint3 = w3;
  setpoint4 = w4;
}

void movebase() {
  int encA = Enc_A.readAndReset() / 4; //pulse
  int encB = Enc_B.readAndReset() / 4; //pulse
  int encC = Enc_C.readAndReset() / 4; //pulse
  int encD = Enc_D.readAndReset() / 4; //pulse
  vel1 = pulse_to_radps(encA);
  vel2 = pulse_to_radps(encB);
  vel3 = pulse_to_radps(encC);
  vel4 = pulse_to_radps(encD);//pulse to radain per second


  output1 = PID_control(vel1, w1);  //use radian per second
  output2 = PID_control(vel2, w2);
  output3 = PID_control(vel3, w3);
  output4 = PID_control(vel4, w4);
  //  Serial.println("output1 : " + String(output1));/
  motor1.setSpeed(output1);
  motor2.setSpeed(output2);
  motor3.setSpeed(output3);
  motor4.setSpeed(output4);

}
int PID_control(float Steady_State, float SetPoint) {

  if (SetPoint == 0) {
    nh.loginfo("Set 0");
    I_sum = 0;
    Op = 0;
    Oi = 0;
    Od = 0;
    int out_put = int(Op + Oi + Od);
    int output_ = constrain(out_put, -255, 255);
    return output_;
  }

  else {
    error = (SetPoint - Steady_State);
    error_n = error;
    Op = Kp * error;

    I_sum = I_sum + error;
    I_sum = constrain(I_sum, -150, 150);

    Oi = Ki * (I_sum * dt);

    Od = Kd * (float(error_n - error_p) * 20);
    error_p = error_n;

    int out_put = int(Op + Oi + Od);

    int output_ = constrain(out_put, -255, 255);
    return output_;
  }
}

float pulse_to_radps(int pul) {
  return pul * 125.66 / ppr;  //(pul*20*2pi)/ppr
}

void encoder()
{

  float average_rpm_x = (float)(vel1 + vel2 + vel3 + vel4) / 4;
  float linear_x = average_rpm_x * r ;


  float average_rpm_y = (float)(-vel1 + vel2 + vel3 - vel4) / 4;
  float linear_y  = average_rpm_y * r ;



  float average_rpm_a = (float)(-vel1 + vel2 - vel3 + vel4) / 4 ;
  float angular_z = average_rpm_a  * r / (lx + ly);


  enc.linear.x = linear_x;
  enc.linear.y =  linear_y;
  enc.angular.z =  angular_z;
  Enc.publish(&enc);
}
void stopBase()
{
  Vx = 0;
  Vy = 0;
  Wz = 0;
}
void publishIMU()
{
  
  //pass accelerometer data to imu object
  raw_imu_msgs.linear_acceleration = readAccelerometer();

  //pass gyroscope data to imu object
  raw_imu_msgs.angular_velocity = readGyroscope();

  //publish raw_imu_msg
  raw_imu_pub.publish(&raw_imu_msgs);
}

geometry_msgs::Vector3 readAccelerometer()
{
    geometry_msgs::Vector3 accel;
    
    accelgyro.getAcceleration(&ax, &ay, &az);

    accel.x = ax * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.y = ay * (double) ACCEL_SCALE * G_TO_ACCEL;
    accel.z = az * (double) ACCEL_SCALE * G_TO_ACCEL;

    return accel;
}

geometry_msgs::Vector3 readGyroscope()
{
    geometry_msgs::Vector3 gyro;

    accelgyro.getRotation(&gx, &gy, &gz);

    gyro.x = gx * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.y = gy * (double) GYRO_SCALE * DEG_TO_RAD;
    gyro.z = gz * (double) GYRO_SCALE * DEG_TO_RAD;

    return gyro;
}

geometry_msgs::Vector3 readMagnetometer()
{
    geometry_msgs::Vector3 mag;

    mag.x = 0; //mx * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.y = 0; //my * (double) MAG_SCALE * UTESLA_TO_TESLA;
    mag.z = 0;  //mz * (double) MAG_SCALE * UTESLA_TO_TESLA;

    return mag;
}
