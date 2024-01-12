#include <Arduino.h>
#include <Servo.h>

//——————**以下为参数设置**——————
int turning_sensitivity = 50    ;//避障转弯速度
int run_speed = 100    ;//直行速度

//——————**以下为IO定义**——————
const int TrackLeft_2 = A2;//左循迹传感器(P3.3 0UT2)
const int TrackRight_2 = A3;//右循迹传感器(P3.2 0UT1)
const int SensorLeft_2 = A1;//左避障传感器(P3.5 0UT4)
const int SensorRight_2 = A0;//右避障传感器(P3.4 0UT3)

int Left_motor_go = 9;//左电机前进(IN2)
int Left_motor_back = 8;//左电机后退(IN1)
int Right_motor_go = 10; // 右电机前进(IN3)
int Right_motor_back = 12;// 右电机后退(IN4)

int TL_2;
int TR_2;
int SL_2;//左避障传感器状态
int SR_2;//右避障传感器状态

int b_pin = 11;//定义舵机引脚
int r_pin = 5;
int f_pin = 3;
int c_pin = 6;

Servo base, rArm, fArm, claw ;

void init_arm(){  //初始化舵机函数
  base.attach(b_pin);
  rArm.attach(r_pin);
  fArm.attach(f_pin);
  claw.attach(c_pin);
  base.write(90);
  delay(2000);
  base.write(120);
  delay(2000);
  base.write(90);
  delay(2000);
  rArm.write(90);
  delay(2000);
  fArm.write(90);
  delay(2000);
  claw.write(105);
  delay(100);
}

void setup(){//初始化电机驱动I0为输出方式
  pinMode(Left_motor_go, OUTPUT);// PIN 9 (PWI)
  pinMode(Left_motor_back, OUTPUT); // PIN 8
  pinMode(Right_motor_go, OUTPUT); // PIN 10 (PWM)
  pinMode(Right_motor_back, OUTPUT); // PIN 12
  //init_arm();
  //定义循迹传感器为输入方式
  pinMode(TrackLeft_2,INPUT); //定义左循迹传感器为输入
  pinMode(TrackRight_2,INPUT);//定义右循迹传感器为输入
  //定义避障传感器为输入方式
  pinMode(SensorLeft_2,INPUT); //定义左避障传感器为输入
  pinMode(SensorRight_2,INPUT);//定义右道障传感器为输入
}


void run(int speed){
  analogWrite(Left_motor_go,speed*0.88);
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Right_motor_go,speed);
  digitalWrite(Right_motor_back,LOW);
  //delay(time);
}

void brake(){
  analogWrite(Left_motor_go,0);
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Right_motor_go,0);
  digitalWrite(Right_motor_back,LOW);
  //delay(time);
}

void left(int speed)  //左转（左轮不动右轮前进）
{
  analogWrite(Left_motor_go,0); //左轮不动
  digitalWrite(Left_motor_back,LOW);
  analogWrite(Right_motor_go,speed); //右侧电机前进，PWM0-255调速
  digitalWrite(Right_motor_back,LOW);
  //delay(time*100);  //执行时间，可以调整
}

void spin_left(int speed) //左转（左轮后退，右轮前进）
{
  analogWrite(Left_motor_go,0); //左轮后退
  digitalWrite(Left_motor_back,HIGH);
  analogWrite(Right_motor_go,speed); //右侧电机前进，PWM0-255调速
  digitalWrite(Right_motor_back,LOW);
  //delay(time*100);  //执行时间，可以调整
}

void right(int speed)  //右转（右轮不动左轮前进）
{
  analogWrite(Right_motor_go,0); //右轮不动
  digitalWrite(Right_motor_back,LOW);
  analogWrite(Left_motor_go,speed); //左侧电机前进，PWM0-255调速
  digitalWrite(Left_motor_back,LOW);
  //delay(time*100);  //执行时间，可以调整
}

void spin_right( int speed) //右转（右轮后退，左轮前进）
{
  analogWrite(Right_motor_go,0); //右轮后退
  digitalWrite(Right_motor_back,HIGH);
  analogWrite(Left_motor_go,speed); //左侧电机前进，PWM0-255调速
  digitalWrite(Left_motor_back,LOW);
  //delay(time*100);  //执行时间，可以调整
}

void back(int time){
  analogWrite(Left_motor_go,0);
  digitalWrite(Left_motor_go,HIGH);
  analogWrite(Right_motor_go,0);
  digitalWrite(Right_motor_back,HIGH);
  delay(time);
  }

void pickUp(){
  
}

void loop(){
  SL_2=digitalRead(SensorLeft_2);
  SR_2=digitalRead(SensorRight_2);
  if (SL_2==HIGH && SR_2==HIGH){//无障碍物
/*———————————————————————————————————————*/
    TL_2=digitalRead(TrackLeft_2);
    TR_2=digitalRead(TrackRight_2);
    if (TL_2 == LOW && TR_2 == LOW){
      run(run_speed);}
    else if(TL_2 == HIGH && TR_2 == LOW){TL_2=digitalRead(TrackLeft_2);
  TR_2=digitalRead(TrackRight_2);
  if (TL_2 == LOW && TR_2 == LOW){
    run(run_speed);}
  else if(TL_2 == HIGH && TR_2 == LOW){
    left(turning_sensitivity );}
  else if(TL_2 == LOW && TR_2 == HIGH){
    right(turning_sensitivity );}
  else 
    brake();
      left(turning_sensitivity );}
    else if(TL_2 == LOW && TR_2 == HIGH){
      right(turning_sensitivity );}
    else 
      brake();}
/*————————————————————————————————————————*/
  else if(SL_2==HIGH && SR_2==LOW)
    left(100);
  else if(SL_2==LOW && SR_2==HIGH)
    right(100);
  else 
    brake();
  }