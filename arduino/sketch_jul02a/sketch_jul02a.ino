#include <ros.h>
#include <std_msgs/Float64MultiArray.h>

ros::NodeHandle nh;
std_msgs::Float64MultiArray angle_array;
ros::Publisher p("/inch/phi_encoder", &angle_array);

#define A1 2
#define B1 3
#define X1 7 
#define A2 19
#define B2 18
#define X2 17

#define gear_ratio 1 // no gear
#define counter_to_deg 0.0703125 // 360(deg) / 5120(PPR)

int counter1 = 0; int counter_past1 = 0;        
int state_rotation1; int state_rotation_past1;  

int counter2 = 0; int counter_past2 = 0;        
int state_rotation2; int state_rotation_past2;  

int x1_counter = 0;
int x2_counter = 0;
  
float angle1 = 0.0; float angle2 = 0.0;

void Read_encoder1()
{
  state_rotation1 = digitalRead(A1);
  
  if (state_rotation1 != state_rotation_past1 && state_rotation1 == 1) counter1 = digitalRead(B1) != state_rotation1 ? counter1+1 : counter1-1;
  state_rotation_past1 = state_rotation1; 
  counter_past1 = counter1;
  angle1 = (counter1/gear_ratio)*counter_to_deg;
}

void Read_encoder2()
{
  state_rotation2 = digitalRead(A2);
  if (state_rotation2 != state_rotation_past2 && state_rotation2 == 1) counter2 = digitalRead(B2) != state_rotation2 ? counter2+1 : counter2-1;
  state_rotation_past2 = state_rotation2; 
  counter_past2 = counter2;
  angle2 = (counter2/gear_ratio)*counter_to_deg;

}

void setup() 
{
  pinMode(A1, INPUT_PULLUP);
  pinMode(B1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(B2, INPUT_PULLUP);

  pinMode(X1, INPUT_PULLUP);
  pinMode(X2, INPUT_PULLUP);
  //Serial.begin(9600);
  state_rotation_past1 = digitalRead(A1);
  state_rotation_past2 = digitalRead(A2);
  
  attachInterrupt(digitalPinToInterrupt(A1), Read_encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A2), Read_encoder2, CHANGE);
  
  nh.initNode();
  nh.advertise(p);

  // 배열 크기 설정
  angle_array.data_length = 2;
  angle_array.data = new float[2];
}

void loop() 
{
  
  //Serial.println(String(angle1)+","+String(angle2));
  angle_array.data[0] = angle1;
  angle_array.data[1] = angle2;
  p.publish(&angle_array);
  nh.spinOnce();
}
