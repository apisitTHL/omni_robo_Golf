#include <Encoder.h>
#include <ArduinoHardware.h>
#include <ros.h> 
#include <geometry_msgs/Twist.h> 
#include <std_msgs/Float32.h> 

int Cout = 0;
// Kinematic constants
const float L = 0.15;  // Distance from the center of the robot to the center of the wheel (in meters)
const float sqrt3 = 1.732; // Approximation for sqrt(3)

// Desired velocities
float in_Vx = 0.0;  
float in_Vy = 0.0;  
float in_omega = 0.0; 

float Vx = 0.0;  // Linear velocity in the x-direction (m/s)
float Vy = 0.0;  // Linear velocity in the y-direction (m/s)
float omega = 0.0; // Angular velocity (rad/s)

const float radius = 0.05; 
float rpmToLinearVelocity(float rpm) {
  return (rpm / 60.0) * 2.0 * PI * radius;
}

// PID constants
float kp_A = 1.0; 
float ki_A = 0.01; 
float kd_A = 0.007; 

float kp_B = 1.0; 
float ki_B = 0.01; 
float kd_B = 0.007; 

float kp_C = 1.0; 
float ki_C = 0.01; 
float kd_C = 0.007; 

double input_A = 0;
double setPoint_A = 100.0; // Desired speed (RPM)
double output_A = 0.0; 
double error_A = 0.0;
double lastError_A = 0.0;
double integral_A = 0.0;
double derivative_A = 0.0;

double input_B = 0;
double setPoint_B = 100.0; // Desired speed (RPM)
double output_B = 0.0; 
double error_B = 0.0;
double lastError_B = 0.0;
double integral_B = 0.0;
double derivative_B = 0.0;

double input_C = 0;
double setPoint_C = 100.0; // Desired speed (RPM)
double output_C = 0.0; 
double error_C = 0.0;
double lastError_C = 0.0;
double integral_C = 0.0;
double derivative_C = 0.0;

int Start_PIN = 59; 
int Stop_PIN = 60; 
int Reset_PIN = 61;

// Encoder variables
volatile long A_Position = 0;
long previousEncoder_A = 0;

volatile long B_Position = 0;
long previousEncoder_B = 0;

volatile long C_Position = 0;
long previousEncoder_C = 0;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
Encoder myEnc_A(3, 2);
Encoder myEnc_B(18, 19);
Encoder myEnc_C(20, 21);

int DIR1 = 7, DIR2 = 9, DIR3 = 11;
int PWM1 = 8, PWM2 = 10, PWM3 = 12;

ros::NodeHandle nh;

geometry_msgs::Twist msg;
std_msgs::Float32 encA_msg;
std_msgs::Float32 encB_msg;
std_msgs::Float32 encC_msg;

ros::Publisher EncA("Enc_A", &encA_msg);
ros::Publisher EncB("Enc_B", &encB_msg);
ros::Publisher EncC("Enc_C", &encC_msg);
  
void roverCallBack(const geometry_msgs::Twist& cmd_vel)
{


  in_Vx = cmd_vel.linear.x;  
  in_Vy = -cmd_vel.linear.y;  
  in_omega = cmd_vel.angular.z; 

 
}

ros::Subscriber <geometry_msgs::Twist> Motor("/cmd_vel", roverCallBack);

void setup() {
//  Serial.begin(9600);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);

  pinMode(Start_PIN, INPUT_PULLUP);
  pinMode(Stop_PIN, INPUT_PULLUP);
  pinMode(Reset_PIN, INPUT_PULLUP);
  nh.initNode();
  nh.subscribe(Motor);
  nh.advertise(EncA);
  nh.advertise(EncB);
  nh.advertise(EncC);
}

void loop() {
  
  A_Position = myEnc_A.read();
  encA_msg.data = A_Position/10;
   EncA.publish( &encA_msg );
   
  B_Position = myEnc_B.read();
  encB_msg.data = B_Position/10;
   EncB.publish( &encB_msg );
   
  C_Position = myEnc_C.read();
  encC_msg.data = C_Position/10;
   EncC.publish( &encC_msg );

  float v1 = Vx + omega * L;
  float v2 = -0.5 * Vx + (sqrt3 / 2) * Vy + omega * L;
  float v3 = -0.5 * Vx - (sqrt3 / 2) * Vy + omega * L;
                
  float v1_in = rpmToLinearVelocity(input_C);
  float v2_in = rpmToLinearVelocity(input_A);
  float v3_in = rpmToLinearVelocity(input_B);          

  float Vx_in = (2.0/3.0) * ((sqrt3 / 2.0) * v2_in - (sqrt3 / 2.0) * v3_in);
  float Vy_in = (2.0/3.0) * (v1_in - 0.5 * v2_in - 0.5 * v3_in);
  float Omega_in = (1.0/3.0) * ((v1_in / L) + (v2_in / L) + (v3_in / L));
  
  setPoint_A = v2*190.985;
  setPoint_B = v3*190.985; 
  setPoint_C = v1*190.985; 

  if(setPoint_A == 0){
    error_A = 0.0;
    integral_A = 0.0;
    derivative_A = 0.0;
  }          
  if(setPoint_B == 0){
    error_B = 0.0;
    integral_B = 0.0;
    derivative_B = 0.0;       
   }
   if(setPoint_C == 0){
    error_C = 0.0;
    integral_C = 0.0;
    derivative_C = 0.0;
      
   }       
                        
  bool SW_Start = digitalRead(Start_PIN);
  bool SW_Stop = digitalRead(Stop_PIN);
  bool SW_Reset = digitalRead(Reset_PIN);
  
  currentMillis = millis();
//  if (currentMillis - previousMillis == 0.0001) {
      
      Encoder_Speed_Wheel();
      PID_Speed_Wheel();

    Cout = Cout + 1;

    if (SW_Start == LOW) {
           in_Vx = 0.6;  
           in_Vy = 0.0;  
           in_omega = 0.0; 
    }
    if (SW_Reset == LOW) {
           in_Vx = 0.0;  
           in_Vy = 0.6;  
           in_omega = 0.0;
    }
    if (SW_Stop == LOW) {
           in_Vx = 0.0;  
           in_Vy = 0.0;  
           in_omega = 0.0;
    }
      Vx = in_Vy;  
      Vy = in_Vx;  
      omega = in_omega; 
   
//    Serial.print(" # V = ");
//    Serial.print(v1);
//    Serial.print(" | ");
//    Serial.print(v2);
//    Serial.print(" | ");
//    Serial.print(v3);

//    Serial.print(" # RPM = ");
//    Serial.print(setPoint_A);
//    Serial.print(" | ");
//    Serial.print(setPoint_B);
//    Serial.print(" | ");
//    Serial.print(setPoint_C);
//    
//    Serial.print(" # input_A = ");
//    Serial.print(input_A);
//    Serial.print(" # input_B = ");
//    Serial.print(input_B);
//    Serial.print(" # input_C = ");
//    Serial.print(input_C);

//    Serial.print(" # v1_in = ");
//    Serial.print(v1_in);
//    Serial.print(" # v1_in = ");
//    Serial.print(v2_in);
//    Serial.print(" v2_in = ");
//    Serial.print(v3_in);
    

//    Serial.print(" # Vx_in = ");
//    Serial.print(Vx_in);
//    Serial.print(" # Vy_in = ");
//    Serial.print(Vy_in);
//    Serial.print(" # Omega_in = ");
//    Serial.print(Omega_in);
//
//    Serial.print(" # output_A = ");
//    Serial.print(output_A);
//    Serial.print(" # output_B = ");
//    Serial.print(output_B);
//    Serial.print(" # output_C = ");
//    Serial.print(output_C);
    
//    Serial.println(" ");
  
    previousMillis = currentMillis;
    
//  }/

nh.spinOnce();
}
void Encoder_Speed_Wheel(){      
    long encoderTicks_A = A_Position - previousEncoder_A;
    previousEncoder_A = A_Position;
    // Calculate RPM (ticks per revolution and time interval should be adjusted as per your encoder)
    input_A = (encoderTicks_A / 2400.000) * (60000.000 / (currentMillis - previousMillis));
    

    long encoderTicks_B = B_Position - previousEncoder_B;
    previousEncoder_B = B_Position;
    // Calculate RPM (ticks per revolution and time interval should be adjusted as per your encoder)
    input_B = (encoderTicks_B / 2400.000) * (60000.000 / (currentMillis - previousMillis));

    long encoderTicks_C = C_Position - previousEncoder_C;
    previousEncoder_C = C_Position;
    // Calculate RPM (ticks per revolution and time interval should be adjusted as per your encoder)
    input_C = (encoderTicks_C / 2400.000) * (60000.000 / (currentMillis - previousMillis));
}
void PID_Speed_Wheel(){
    // PID calculations
      error_A = setPoint_A - input_A;
      integral_A += error_A * (currentMillis - previousMillis);
      derivative_A = (error_A - lastError_A) / (currentMillis - previousMillis);
      output_A = (kp_A * error_A) + (ki_A * integral_A) + (kd_A * derivative_A);
      lastError_A = error_A;

      // Control the motor with PID output
        controlMotorA(output_A);

      // PID calculations
      error_B = setPoint_B - input_B;
      integral_B += error_B * (currentMillis - previousMillis);
      derivative_B = (error_B - lastError_B) / (currentMillis - previousMillis);
      output_B = (kp_B * error_B) + (ki_B * integral_B) + (kd_B * derivative_B);
      lastError_B = error_B;

      // Control the motor with PID output
        controlMotorB(output_B);

      // PID calculations
      error_C = setPoint_C - input_C;
      integral_C += error_C * (currentMillis - previousMillis);
      derivative_C = (error_C - lastError_C) / (currentMillis - previousMillis);
      output_C = (kp_C * error_C) + (ki_C * integral_C) + (kd_C * derivative_C);
      lastError_C = error_C;

      // Control the motor with PID output
        controlMotorC(output_C);
  }
void controlMotorA(double speed_A) {
  if (speed_A > 0) {
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, constrain(speed_A, 0, 255));
  } else {
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, constrain(-speed_A, 0, 255));
  }
}
void controlMotorB(double speed_B) {
  if (speed_B > 0) {
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, constrain(speed_B, 0, 255));
  } else {
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, constrain(-speed_B, 0, 255));
  }
}
void controlMotorC(double speed_C) {
  if (speed_C > 0) {
    digitalWrite(DIR3, LOW);
    analogWrite(PWM3, constrain(speed_C, 0, 255));
  } else {
    digitalWrite(DIR3, HIGH);
    analogWrite(PWM3, constrain(-speed_C, 0, 255));
  }
}
