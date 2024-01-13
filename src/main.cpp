#include <Arduino.h>
#include <Servo.h>

//——————**以下为参数设置**——————
int turning_sensitivity = 50    ;//避障转弯速度
int run_speed = 100    ;//直行速度
int globalTurnDelay = 15 ;//舵机转向循环间隔
int baseStd = 50 ;//base舵机初始位置重定义

//——————**以下为IO定义**——————
const int TrackLeft_2 = A2;//左循迹传感器(P3.3 0UT2)
const int TrackRight_2 = A3;//右循迹传感器(P3.2 0UT1)
const int SensorLeft_2 = A1;//左避障传感器(P3.5 0UT4)
const int SensorRight_2 = A0;//右避障传感器(P3.4 0UT3)

int Left_motor_go = 9;//左电机前进(IN2)
int Left_motor_back = 8;//左电机后退(IN1)
int Right_motor_go = 10; // 右电机前进(IN3)
int Right_motor_back = 12;// 右电机后退(IN4)

int b_pin = 11;//定义舵机引脚
int r_pin = 5;
int f_pin = 3;
int c_pin = 6;

/*——————全局变量定义——————*/
Servo base, rArm, fArm, claw ;
int angB, angR, angF, angC ;//舵机角度状态
int TL_2;//左循迹传感器状态
int TR_2;//右循迹传感器状态
int SL_2;//左避障传感器状态
int SR_2;//右避障传感器状态

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

void spin_right(int speed) //右转（右轮后退，左轮前进）
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

void capture(){
  claw.attach(c_pin);
  for (angC = 45; angC <= 105; angC += 1) {
    claw.write(angC);
    delay(globalTurnDelay); } 
}

void release(){
  claw.attach(c_pin);
  for (angC = 105; angC >=45; angC += -1) {
    claw.write(angC);
    delay(globalTurnDelay); } 
}

void arMove(char flag, char direct, char angX){
  base.attach(b_pin);
  rArm.attach(r_pin);
  fArm.attach(f_pin);
  int angP, angT;
  if (flag == 'b'){
    if (direct == 'l'){
      angT = angB + angX;
      for (angP = angB; angP <= angT; angP += 1) {
        base.write(angP);
        delay(globalTurnDelay); }   
      angB = angT;
    }
    else if (direct == 'r'){
      angT = angB - angX;
      for (angP = angB; angP >= angT; angP += -1) {
        base.write(angP);
        delay(globalTurnDelay); }   
      angB = angT;
    }
  }
  else if (flag == 'r'){
    if (direct == 'f'){
      angT = angR + angX;
      for (angP = angR; angP <= angT; angP += 1) {
        rArm.write(angP);
        delay(globalTurnDelay); }  
      angR = angT;
    }
    else if (direct == 'b'){
      angT = angR - angX;
      for (angP = angR; angP >= angT; angP += -1) {
        rArm.write(angP);
        delay(globalTurnDelay); } 
        angR = angT;
    }
  }
  else if (flag == 'f'){
    if (direct == 'u'){
      angT = angF + angX;
      for (angP = angF; angP <= angT; angP += 1){
        fArm.write(angP);
        delay(globalTurnDelay); }
        angF = angT;
    }
    else if (direct == 'd'){
      angT = angF - angX;
      for (angP = angF; angP >= angT; angP += -1){
        fArm.write(angP);
        delay(globalTurnDelay); }
        angF = angT;
    }
  }
}

/*
舵机正负表
base     | +向左  -向右
rArm     | +前压  
fArm     | +up
claw     | 45开 105闭合
*/

void pickUp(){
  base.attach(b_pin);
  rArm.attach(r_pin);
  fArm.attach(f_pin);
  claw.attach(c_pin);
  base.write(baseStd);
  delay(500);
  rArm.write(90);
  delay(500);
  fArm.write(90);
  delay(500);
  claw.write(45);
  delay(500);
  /*————————————————————*/
  for (angB = 60-15; angB <= 90-15; angB += 1) {
    base.write(angB);
    delay(globalTurnDelay); }    
  for (angF = 90; angF >= 40; angF += -1) {
    fArm.write(angF);
    delay(globalTurnDelay); }     
  for (angR = 90; angR <= 155; angR += 1) {
    rArm.write(angR);
    delay(globalTurnDelay); }    
  for (angC = 45; angC <= 105; angC += 1) {
    claw.write(angC);
    delay(globalTurnDelay); } 
  for (angF = 40; angF <= 110; angF += 1) {
    fArm.write(angF);
    delay(globalTurnDelay); }     
  for (angR = 150; angR >= 110; angR += -1) {
    rArm.write(angR);
    delay(globalTurnDelay); }  
  for (angB = 90-15; angB <= 168; angB += 1) {
    base.write(angB);
    delay(globalTurnDelay); } 
  rArm.write(90);
  delay(500);
  fArm.write(90);
  delay(500);
  for (angF = 90; angF >= 25; angF += -1) {
    fArm.write(angF);
    delay(globalTurnDelay); }     
  for (angR = 90; angR <= 135; angR += 1) {
    rArm.write(angR);
    delay(globalTurnDelay); }    
  for (angC = 105; angC >=48; angC += -1) {
    claw.write(angC);
    delay(globalTurnDelay); } 
}



void loop(){
  SL_2=digitalRead(SensorLeft_2);
  SR_2=digitalRead(SensorRight_2);
  if (SL_2==HIGH && SR_2==HIGH){
    TL_2=digitalRead(TrackLeft_2);
    TR_2=digitalRead(TrackRight_2);
    if (TL_2 == LOW && TR_2 == LOW){
      run(run_speed);}
    else if(TL_2 == HIGH && TR_2 == LOW){
          left(turning_sensitivity );}
    else if(TL_2 == LOW && TR_2 == HIGH){
          right(turning_sensitivity );}
    else {brake();}
    }
  else if(SL_2==LOW && SR_2==LOW){
    brake();
    pickUp();
    delay(3000);
  }
}