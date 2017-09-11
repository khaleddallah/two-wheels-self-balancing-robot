#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#define G 16384.0      //Both Values obtained from MPU6050 Datasheet
#define GYRO_SCL 131.0

#include <EEPROMex.h>
// 1.X Interface PINS and Modules Variables
// 1.0 Timing Variables
uint32_t loopStartTime;

// 1.1 Motor PINS and variables   //Both Motors are drived together
const int a1 = 3;          //left motor  / forward  
const int a2 = 5;          //Right motor / forward
const int ENa = 10;// pin 1 on L293D IC
const int b1 = 6;       //left motor  / backward 
const int b2 = 9;       //Right motor / backward 
const int ENb = 11;    //pin 9 on L293D IC
double Speed = 0;
char Dir = 'f';

//1.2 MPU6050 PINS  
/*
  Pin 1 = 3.3V
  Pin 2 = GND
  Pin 3 = A5
  Pin 4 = A4
*/

//1.3 Output Led Pin and Variables
//#define LED_PIN 13
//bool blinkState = false;

//1.4 MPU6050 Variables
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
double acc_angle;
double gyro_rate;

//1.5 Kalman Filter Variables
float Q_angle  =  0.001;
float Q_gyro   =  0.003;
float R_angle  =  0.03;

double actAngle; //Output Angle
double angle = 0;  // Reset the angle
float bias = 0;   // Reset bias
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;  //error covirance matrix reset to 0
float dt, y, S;
float K_0, K_1;

//1.6 PID Variables
//Setpoint. where the robot is balanced.  
double Setpoint = -1;
//distance away from setpoint
double error, ITerm, lastInput, dInput;
         // <<<<<---------

unsigned long SampleTime= 100;
unsigned long lastTime;
double outMin = -255.0, outMax = 255.0;
double output; //Temp Var for debugging


int addressFloat=0;
float Kp = 0.0; 
float Ki = 0.0; 
float Kd = 0.0;  
String Deb = "nn";
//3.0 Implementation Code
//3.1 Setup Intialization Code 
void setup() {
  
 Kp = EEPROM.readFloat(0); 
 delay(100);
 Ki = EEPROM.readFloat(8) * 20;
 delay(100); 
 Kd = EEPROM.readFloat(16) / 20; 
 delay(100);
 Setpoint = EEPROM.readFloat(32) ;

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    
    // initialize serial communication
    Serial.begin(9600);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    accelgyro.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(accelgyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    loopStartTime = millis();
    
    SetSampleTime(5);
    lastTime = millis()-SampleTime;
    addressFloat  = EEPROM.getAddress(sizeof(float));   
    //configure Arduino Pins
    pinMode(a1, OUTPUT);
    pinMode(a2, OUTPUT);
    pinMode(b1, OUTPUT);
    pinMode(b2, OUTPUT);
   // pinMode(ENa, OUTPUT);
   // pinMode(ENb, OUTPUT);
      Serial.print(Kp);
      Serial.print('/');
      Serial.print(Ki);
      Serial.print('/');
      Serial.print(Kd);
      Serial.print('/');
      Serial.print(Setpoint);
      Serial.print('/');
      Serial.println(Deb);
      delay(400);


}  //setup

//3.2 Execution Loop
void loop() {
    if (Serial.available() > 0) {//If we sent the program a command deal with it 
      for (int x = 0; x < 6; x++) {
        switch (x) {
          case 0: Kp = Serial.parseFloat(); EEPROM.updateFloat(0,Kp);break;
          case 1: Ki = Serial.parseFloat(); EEPROM.updateFloat(8,Ki);break;
          case 2: Kd = Serial.parseFloat(); EEPROM.updateFloat(16,Kd);break; 
          case 3: Setpoint = Serial.parseFloat(); EEPROM.updateFloat(32,Setpoint);break;
          case 4: Deb= Serial.readString();
        }
      }
      Serial.print(Kp);
      Serial.print('/');
      Serial.print(Ki);
      Serial.print('/');
      Serial.print(Kd);
      Serial.print('/');
      Serial.print(Setpoint);
      Serial.print('/');
      Serial.println(Deb);
      delay(400);
      
    }
  // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);    //Debug_Raw();
    
    acc_angle = getAccAng(ay, az);
    gyro_rate = gx / GYRO_SCL;    //Debug_Orient();
    
    actAngle = kalmanCalculate( acc_angle, gyro_rate, millis() - loopStartTime )+90;
    loopStartTime = millis();    //Debug_Filtered();
    
    error = Setpoint - actAngle; //distance away from setpoint
    
    if( error > 0 ) {
      Dir = 'f';
    }
    else if ( error < 0 ) {
      Dir = 'r';
    }
    
    Compute();    
    
    if(Speed < 75)
      Speed = 0;    //To Stop Whinning Sound of the Motors as Low Speeds
    
    if ( actAngle < -30 || actAngle > 30)
      Speed = 0;    //No point of Trying to balance the robot if it is after 30 degrees

    if (Deb=="yy")
       Debug_PID();
    
    Drive_Motor( Speed, Dir); 
    
    // blink LED to indicate activity
   // blinkState = !blinkState;
  //  digitalWrite(LED_PIN, blinkState);
}  //loop

//4.X Additional Function
//4.1 Calculate Acc Angle
inline double getAccAng(int16_t ay, int16_t az) {
    // Convert to 360 degrees resolution
    // atan2 outputs the value of -π to π (radians)
    // We are then convert it to 0 to 2π and then from radians to degrees
    double angle = ( ( atan2( -ay , -az ) + PI ) * RAD_TO_DEG );
    
    if( angle >= 180 ) {  //to map the range from -180 to 180
      angle -= 360;
    }
    return angle;
}

//4.2 Kalman Function
double kalmanCalculate(float newAngle, float newRate,int looptime) {
    
    dt = looptime / 1000.0;
    angle += dt * (newRate - bias);  //angle = rate * timesample
    
    P_00 +=  dt * ( dt * P_11 - P_01 - P_10 + Q_angle);
    P_01 -=  dt * P_11;
    P_10 -=  dt * P_11;
    P_11 +=  Q_gyro * dt;
    
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;
    
    y = newAngle - angle;    
    angle +=  K_0 * y;
    bias  +=  K_1 * y;
    
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;
    
    return angle;
}  //Kalman

//4.3 Drive Motors
void Drive_Motor( double Speed, char Dir) {
  
  if(Dir == 'r') { //
 
    digitalWrite(a1,LOW);
    digitalWrite(a2,LOW);
    digitalWrite(b1,HIGH);
    digitalWrite(b2,HIGH);
  }
  else if( Dir == 'f' ) {//
    digitalWrite(a1,HIGH);
    digitalWrite(a2,HIGH);
    digitalWrite(b1,LOW);
    digitalWrite(b2,LOW);
   
  }
  
  analogWrite(ENa, (int) Speed);  //PWM is written last to ensure the Motor move in the
  analogWrite(ENb, (int) Speed);                                 //correct direction
  
}  //Drive_Motor

void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime / (double)SampleTime;
      Ki *= ratio;
      Kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}

void Compute()
{
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange >= SampleTime)
   {
      /*Compute all the working error variables*/     
      ITerm += (Ki * error);
      
      if(ITerm > outMax) ITerm= outMax;      //To reduce the time ITerm needs to settle down after error was recovered 
      else if(ITerm < outMin) ITerm= outMin;
      
      dInput = (actAngle - lastInput);
      
      /*Compute PID Output*/
      output = Kp * error + ITerm - Kd * dInput;
      
      if(output > outMax) output = outMax;
      if(output < outMin) output = outMin;
      if(output < 0) output = abs(output);
      
      Speed = output;
	  
      /*Remember some variables for next time*/
      lastInput = actAngle;
      lastTime = now;
   }
}

//5.X Debug Functions
//5.1 Printing MPU6050 Output Raw Values
//TODO: More Style Formating
void Debug_Raw() {
    // display tab-separated accel/gyro x/y/z values
    Serial.print(F("MPU Raw Readings: "));
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
}

void Debug_Orient() {
    Serial.print(F("Robot Orientation: "));
    Serial.print(acc_angle); Serial.print("\t");
    Serial.println(gyro_rate);
}

void Debug_Filtered() {
    Serial.print(F("Robot Actual Angle: "));
    Serial.print(F("Angle:"));         Serial.println(actAngle);
}

void Debug_PID() {
    Serial.print(F("PID: "));
    Serial.print(F("A:"));         Serial.print(actAngle);       Serial.print("\t");
    Serial.print(F("E:"));         Serial.print(error);          Serial.print("\t");
    Serial.print(F("P:"));  Serial.print(Kp * error);     Serial.print("\t");
    Serial.print(F("I:"));      Serial.print(ITerm);          Serial.print("\t");
    Serial.print(F("D:"));    Serial.print(Kd * dInput);    Serial.print("\t");
    Serial.print(F("O:"));        Serial.print(output);         Serial.print("\t");
    Serial.print(F("D:"));           Serial.print(Dir);            Serial.print("\t");
    Serial.print(F("S:"));         Serial.println(Speed);     Serial.print("\t");
    Serial.print(F("Kp:"));         Serial.println(Kp);     Serial.print("\t");
    Serial.print(F("ki:"));         Serial.println(Ki);     Serial.print("\t");
    Serial.print(F("kd:"));         Serial.println(Kd);     
}

void Debug_Motion() {
    Serial.print(F("Robot Motion: "));
    Serial.print(F("Angle:"));         Serial.print(actAngle);       Serial.print("\t");
    Serial.print(F("Dir:"));           Serial.print(Dir);            Serial.print("\t");
    Serial.print(F("Speed:"));         Serial.println(Speed);
}
