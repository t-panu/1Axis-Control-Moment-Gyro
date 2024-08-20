//6301002810023 KMUTNB MAE
//Phanuwat Thamnita
//Prof. Suwat KUNTANAPREEDA
//For WEMOS D1R32 (ESP32)

//Libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Arduino.h>
#include <BluetoothSerial.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
//***********************************************************************************************************************************************************
//Define
#define PI 3.1415926535897932384626433832795
#define deg2rad 0.017453292519943295769236907684886
#define rad2deg 57.295779513082320876798154814105
// 200Steps per revolution for nema17
// 360deg/200step >> microstep1/16 = 0.1125 deg/step
#define pstep_size 0.1125         //deg/step
#define nstep_size -0.1125        //deg/step
#define pstep_size_rad 0.0019635  //rad/step
#define nstep_size_rad -0.0019635 //rad/step
#define I 0.0335                  //Inertia by Trifilar Pendulum method
#define hw 0.0742                 //Wheel angular velocity 1165 rad/s
#define c 0.04                    //Damping Ratio
#define NData 1000                //Amounts of values
//***********************************************************************************************************************************************************
// Declare variables
int iData;
int b_yaw[NData];                 //Yaw
int b_omg[NData];                 //Omega
int b_u[NData];                   //Control Signal
int b_d[NData];                   //Delta
int b_ddot[NData];                //Delta dot
// Define pin connections
const int dirPin = 26;            //uno pin 2
const int stepPin = 25;           //uno pin 3
const int m1 = 27;                //uno pin 6
const int m2 = 16;                //uno pin 5
const int m3 = 17;                //uno pin 4
const int ledHigh = 18;           //uno pin 13
const int ledLow = 19;            //uno pin 12
const int a0 = 2;                 //uno pin A0
const int a1 = 4;                 //uno pin A1
const int a2 = 36;                //uno pin A2
// Stepper motor control
float freq;
float pulse_delay;
float step_move;
int n_step;
// Controller Sampling Time
float dt;                          // msec > time left to control motor
unsigned long samp_time = 10;      // msec > Overall action time per loop
// Sampling time = 10ms = 0.01sec
// Min speed (1 step per dt) = 0.1125/0.01 = 11.25 deg/sec
unsigned long start_time;         // msec
unsigned long stop_time;          // msec
// Potentimeter
float yaw ;         // deg
float y_rad;        // rad
// Rate Gyro
float omg_offset = 0;
float omg_rad;      // rad
// PD Control law
float u;
float kp;
float kd;
// SC Control law
float eta;
float lamda;
// Steering
float delta = 0.0;  // rad
float del;
float ddot;         // rad/s

//***********************************************************************************************************************************************************
Adafruit_MPU6050 mpu;                 //Start MPU6500
BluetoothSerial SerialBT;             //Define bluetooth serial name
//***********************************************************************************************************************************************************

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void setup()
{
  //*****************************************************************************************************************
  Serial.begin(115200);               // sets the serial port to 115200 best for bluetooth
  SerialBT.begin("SGCMG_hardware");   // Set Bluetooth device name
  //*****************************************************************************************************************
  // MCU6500 Setting
  while (!Serial)
    delay(10);
  // Try to initialize!
  if (!mpu.begin()) {
    while (1) {
      delay(10);
    }
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);  //Gyroscope Range 500 deg/s
  //*****************************************************************************************************************
  // Stepping Motor Microstep setting
  // L-L-L = Full, H-L-L = 1/2, L-H-L = 1/4, H-H-L = 1/8,H-H-H = 1/16
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  digitalWrite(m1, HIGH);
  digitalWrite(m2, HIGH);
  digitalWrite(m3, HIGH);
  // Declare pins as stepping Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  // Declare Status Led output
  pinMode(ledHigh, OUTPUT);
  pinMode(ledLow, OUTPUT);
  //*****************************************************************************************************************
  del = map(analogRead(a0), 0, 4095, -31415, 31416) / 10000 ; //A0 = pin 2
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void loop()
{
  delay(5000);
  for (iData = 0; iData < NData; iData++)
  {
    //***********************************************************************************************************************************************************
    start_time = millis();                                    //Set timer
    // Compute control law + doing work
    // Potentiometer
    yaw = map(analogRead(a1), 0, 4095, -18000, 17900) / 100 ; //A1 = pin2
    b_yaw[iData] = yaw * 100;
    y_rad = yaw * deg2rad;                                    //rad
    // Rate Gyro
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    omg_rad = -g.gyro.z - omg_offset;                         //Read as rad/s
    b_omg[iData] = omg_rad * 100 ;
    //***********************************************************************************************************************************************************
    // Control Law
    kp = 2;
    kd = 0.5;
    // SC Control law
    eta = 0.5;
    lamda = 4;
    // u = -(kp * y_rad);
     u = -(kp * y_rad) - (kd * omg_rad) ;
    // u = I * (-(lamda * omg_rad) - (1 / eta) * (omg_rad + (lamda * y_rad)))-c*omg_rad;
    // Steering Law
    b_u[iData] = u * 100000;
    ddot = -u / (hw * cos(delta));
    // ddot = u;
    // Find Stepping Frequency
    freq = abs(ddot / pstep_size_rad) / 1000;                 //freq(Hz) => step/msec
    stop_time = millis();                                     //Timer stopped
    // All work done >> Command stepping
    //***********************************************************************************************************************************************************
    dt = samp_time - (stop_time - start_time);                // msec time left
    n_step = freq * dt;                                       //n_step step/dt(10ms) = > freq(step/ms) x [10ms/dt] => unit step/dt
    pulse_delay = dt / n_step * 500; //(dt/n_step/2)*1000;    //usec
    //***********************************************************************************************************************************************************
    // Pulse Delay time saturation
    if ( pulse_delay < 500 ) {
      pulse_delay = 500;
      n_step = 8;
      digitalWrite(ledHigh, HIGH);
    }
    else if ( pulse_delay > 3500 ) {
      pulse_delay = 3500;
      n_step = 1;
      digitalWrite(ledLow, HIGH);
    }
    else {
      digitalWrite(ledHigh, LOW);
      digitalWrite(ledLow, LOW);
    }
    //  Steppinng Motor
    if ( ddot <= 0 ) {
      // Set motor direction clockwise
      digitalWrite(dirPin, HIGH);
      step_move = nstep_size_rad;
      b_ddot[iData] = -pulse_delay;
    }
    else if ( ddot > 0 ) {
      // Set motor direction counterclockwise
      digitalWrite(dirPin, LOW);
      step_move = pstep_size_rad;
      b_ddot[iData] = pulse_delay;
    }
    //***********************************************************************************************************************************************************
    // Command stepper motor
    for (int i = 1; i <= n_step ; i++)
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(pulse_delay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(pulse_delay);
      delta += step_move;
      del = map(analogRead(a0), 0, 4095, -179, 180) ; //A0 = pin 2
    }
    b_d[iData] = delta * 10000;
  }
  //***********************************************************************************************************************************************************
  // Send data via bluetooth
  for (int j = 0; j < NData; j++)
  {
    SerialBT.print(b_yaw[j]);
    SerialBT.print(",");
    SerialBT.print(b_omg[j]);
    SerialBT.print(",");
    SerialBT.print(b_u[j]);
    SerialBT.print(",");
    SerialBT.print(b_d[j]);
    SerialBT.print(",");
    SerialBT.println(b_ddot[j]);
    delayMicroseconds(10);
  }
  while (1);
}
