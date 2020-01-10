
#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

int DIST=6;
int vcc =43;
int trig = 45;
int echo = 47;
int gnd = 49;
long dist;
long duration;

int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;
int ena = 5;
int enb = 10;
int error=0;
int prev_error=0;
double KP = 0.03;
double KD = 0.04;
double kp=0.02;
double kd=0.03;
double ki=0.001;
double integral=0;
int speedMultiply;
char c;
char recieved_pi_Char[2];
int ndx=0;
int cvt_int;

enum states{MANUAL, AUTO, DEF,STOP,RASP_PI, TURING};

enum states state;


void setup() {

  kp = KP;
  kd = KD;
 Serial2.begin(38400);
 Serial3.begin(38400);
 Serial.begin(38400);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){40,42,44,46,48}, SensorCount);
  qtr.setEmitterPin(38);
  pinMode(38,OUTPUT);

  pinMode(vcc, OUTPUT);
digitalWrite(vcc,HIGH);

pinMode(gnd, OUTPUT);
digitalWrite(gnd,LOW);

pinMode(trig, OUTPUT);
pinMode(echo, INPUT);


  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  //digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  
  for (uint16_t i = 0; i < 10; i++)
  {
    qtr.calibrate();
  }
//  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration


  //Serial.begin(38400);
  int cal=700;
  int cal1=1800;
  qtr.calibrationOn.minimum[0]=cal;
  qtr.calibrationOn.minimum[1]=cal;
  qtr.calibrationOn.minimum[2]=cal;
  qtr.calibrationOn.minimum[3]=cal;
  qtr.calibrationOn.minimum[4]=cal;

  qtr.calibrationOn.maximum[0] = cal1;
  qtr.calibrationOn.maximum[1] = cal1;
  qtr.calibrationOn.maximum[2] = cal1;
  qtr.calibrationOn.maximum[3] = cal1;
  qtr.calibrationOn.maximum[4] = cal1;
  
  
//  for (uint8_t i = 0; i < SensorCount; i++)
//  {
//    
//    Serial.print(qtr.calibrationOn.minimum[i]);
//    Serial.print(' ');
//  }
  Serial.println();

//  for (uint8_t i = 0; i < SensorCount; i++)
//  {
//    Serial.print(qtr.calibrationOn.maximum[i]);
//    Serial.print(' ');
//  }
  Serial.println();
  Serial.println();
  delay(1000);

 pinMode(38,OUTPUT);
  digitalWrite(38,HIGH);

  /////////////////////////////
    pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena, OUTPUT);
    pinMode(enb, OUTPUT);

 speedMultiply = 55+20*2;

}

void loop() {

    checkCollision();
  

 
 if(Serial2.available()){

      c = Serial2.read();

      if(c=='p'){
        state=AUTO;
        Serial.println("Auto state initiated");
      }
      if(c=='q'){
        state=MANUAL;
      }


      if(isdigit(c)){
 
  speedMultiply = 55 + 20*(c - '0');
  analogWrite(ena,speedMultiply);
  analogWrite(enb,speedMultiply);
  
}
else{
switch(c){

case 'a' : forward(speedMultiply);break;
case 'b' : stop1();break;
case 'r' : rightFine();break;
case 'l' : leftFine();break;
case 'D' : defaultD(speedMultiply);break;
case 'u' :kp=kp+0.01;break;
case 'j' :kp=kp-0.01;break;
case 'i' :kd=kd+0.01;break;
case 'k' :kd=kp-0.01;break;
case 'm' : kp=KP; kd=KD;break;
default : break;
  
}
    
    }

  
 }
  
  switch(state){
    
    
//    case AUTO: irControl();Serial.println("IR state initiated");;break;
//    case MANUAL:remoteControl();Serial.println("man state initiated");break;
//    default: ;Serial.println("def state initiated");break;     
//    
      case AUTO: irControl();break;
    case MANUAL:     Serial.println("manual control");break;
//    case MANUAL:  RaspiSerial();break;
//      case MANUAL:irControl();break;

//    case MANUAL:turning();  ;break;
    case STOP : Serial.println("stopeed");break;
    case RASP_PI: RaspiSerial();break;
    case TURING: turning();break;
    default:state=MANUAL ;Serial.println("def state initiated");break;


    //set default state to turning for testing 2211 05/01/2020
    
    }
  
 



}


void irControl(){

uint16_t position = qtr.readLineBlack(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.print(position);
  Serial.print('\t');

  delay(15);

  error = (2000 - position);

//  integral +=error;
//  if(integral>10 || integral<-10){
//    integral=0;
//  }
  int speedMotor = kp*error + kd*(error - prev_error);

   MotorControl(speedMotor);

  prev_error = error;

  


  
}


void MotorA(int speed_error){
  digitalWrite(in1,LOW);
digitalWrite(in2, HIGH);
  analogWrite(ena,speed_error);
  analogWrite(ena,speed_error);
  
}

void MotorB(int speed_error){
  
digitalWrite(in3,LOW);
digitalWrite(in4, HIGH);
 analogWrite(ena,speed_error);
  analogWrite(enb,speed_error);
  
}
int motorAspeed;
int motorBspeed;
int base1= 90;
void MotorControl(int speed_error){
  digitalWrite(in1,LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4, HIGH);
if(speed_error>-base1 && speed_error<base1){

//  if(abs(speed_error)>30&& speed_error>0){
//    speed_error=25;
//  }
//  else if(abs(speed_error)>30&& speed_error<0){
//    speed_error=-25;
//  }

motorAspeed=base1 - speed_error;
motorBspeed=base1+speed_error;

if(motorAspeed>base1){
  motorAspeed=base1;
}
else if (motorAspeed<0){
  motorAspeed=0;
}
if(motorBspeed>base1){
  motorBspeed=base1;
}
else if(motorBspeed<0){
   motorBspeed=0;
}
 
analogWrite(ena,motorAspeed);
analogWrite(enb,motorBspeed);

checkBlack();
//checkCorner();
  Serial.print(speed_error);
  Serial.print('\t');
  Serial.print(motorAspeed);
  Serial.print('\t');
  Serial.println(motorBspeed);

}
  


}


void checkBlack(){
  int count=0;
 for(int val:sensorValues){
  
    if(val>700){
      count++;
      if(count>4){
        analogWrite(ena,0);
        analogWrite(enb,0);
        state=TURING;
        
      }
  
}}}

void checkCorner(){
  int count=0;
// for(int val:sensorValues){
//    if(val>700){
//      count++;
//      if(count>2){
//        Serial.print('\t');
//      Serial.print("corner");
//      }
//      
//    }
// }

if(sensorValues[0]>700&&sensorValues[1]>700){

  leftFine();
  delay(400);
  Serial.println("takingright");
}


  
}


void forward(int c){

//MOTOR A
digitalWrite(in1,LOW);
digitalWrite(in2, HIGH);
analogWrite(ena, c);
//MOTOR B
digitalWrite(in3,LOW);
digitalWrite(in4, HIGH);
analogWrite(enb, c);
Serial.println(speedMultiply);

 }

 void stop1(){
//MOTOR A
digitalWrite(in1,LOW);
digitalWrite(in2, LOW);
analogWrite(ena, 0);
//MOTOR B
digitalWrite(in3,LOW);
digitalWrite(in4, LOW);
analogWrite(enb, 0);
Serial.println("stopped");

  
 }


 void rightFine(){
  analogWrite(ena, 90);
    analogWrite(enb,30);
  
  }

 void leftFine(){
  analogWrite(enb,90);
   analogWrite(ena, 30);
  }

void defaultD(int c){
analogWrite(ena, c);
  analogWrite(enb, c);
  }

////////////////////////////////RASPBERRY PI//////////////////////////////////////
double Rkp=12;
double Rkd=10;
int prev_error_object;

 void RaspiSerial(){
//    Serial.println("Raspi");

  if(Serial3.available()){
    
char str = Serial3.read();
if(str=='s'){
  stop1();
  
}
else if(isdigit(str)){
recieved_pi_Char[0]=str;
//ndx++;

//if(ndx==2){
 
//  recieved_pi_Char[ndx]='\0';
  sscanf(recieved_pi_Char, "%d", &cvt_int);
  

  int object_frame_error = 5-cvt_int ;


    int speedMotor_PI = Rkp*object_frame_error + Rkd*(error - prev_error);

//   MotorControl(speedMotor);
     MotorControl_PI(speedMotor_PI);

  prev_error_object = object_frame_error;
  Serial.print("char");  
  Serial.println(recieved_pi_Char);
  
  
//}

   
  }
  }
 }
 int base =80;
//////////////////////////PI_MOTOR CONTROL////////////////////////////  
 int motorAspeed_PI;
int motorBspeed_PI;
void MotorControl_PI(int speed_error){
  digitalWrite(in1,LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4, HIGH);
if(speed_error>-base && speed_error<base){


motorAspeed=base - speed_error;
motorBspeed=base+speed_error;

if(motorAspeed>base){
  motorAspeed=base;
}
else if (motorAspeed<0){
  motorAspeed=0;
}
if(motorBspeed>base){
  motorBspeed=base;
}
else if(motorBspeed<0){
   motorBspeed=0;
}
 
analogWrite(ena,motorAspeed);
analogWrite(enb,motorBspeed);


//checkCorner();
  Serial.print(speed_error);
  Serial.print('\t');
  Serial.print(motorAspeed);
  Serial.print('\t');
  Serial.println(motorBspeed);
  

}
  


}
 
//////////////////////////TURNING////////////////////////////    

int Rcount=0;
int writeCount=0;
void turning(){
writeCount++;
if(writeCount>500){
  Serial3.print('y');
  writeCount=0;
}


  if(Serial3.available()){
    
char str = Serial3.read();
if(str=='b' || str=='g' ||str=='y' ){
  for(int i=0; i<8;i++){
    char c = Serial3.read();
      delay(1000);
    
            Serial.println(c);
            if(colorCheck(c)){
              Serial.println(Rcount);
              Rcount++;
            }

                  if(Rcount>4){
                    directionSelect(c);
                    Rcount=0;
                    break;
                  }
    
  }
        }


  }
  }


char prev_char;
bool colorCheck(char c){

      if(c==prev_char){
        
        return true;
      }
      else{
        prev_char=c;
        return false;
      }

  
}

void directionSelect(char str){

switch(str){

  case 'b': leftSpecial(100); Serial.println("blue");break;
  case 'g': rightSpecial(100);Serial.println("green");break;
  case 'y' :goStraight(100);Serial.println("yellow"); break;
  default : Serial.println("nostring");break;
}


delay(900);
forward(5);
stop1();
state = RASP_PI;


}


void leftSpecial(int c){
Serial.println("blue");
  //MOTOR A
digitalWrite(in1,LOW);
digitalWrite(in2, HIGH);
 analogWrite(enb,c);


//MOTOR B
digitalWrite(in3,HIGH);
digitalWrite(in4, LOW);


 
  analogWrite(ena, c);


  
}
void rightSpecial(int speed1){
  Serial.println("right");
  //MOTOR A
digitalWrite(in1,HIGH);
digitalWrite(in2, LOW);
//MOTOR B
digitalWrite(in3,LOW);
digitalWrite(in4, HIGH);


  analogWrite(enb,speed1);
  analogWrite(ena, speed1);
  
}

void goStraight(int speed1){
Serial.println("straight");
  forward(5);
  
}
  


long getDistance(){
  
digitalWrite(trig, LOW);
delayMicroseconds(5);

digitalWrite(trig, HIGH);
delayMicroseconds(10);
digitalWrite(trig,LOW);

duration = pulseIn(echo, HIGH);

dist = duration * 0.032/2;

return dist;

}
int distVerify=0;
void checkCollision(){

dist = getDistance();
if (dist>50){return;}
Serial.print("Distance " );
Serial.println(dist); 
  
if(dist<DIST){

distVerify++;


if(distVerify>10){
  
  while(dist<DIST){
    dist = getDistance();
    analogWrite(ena,0);
    analogWrite(enb,0);
    Serial.println("under");
  }
  distVerify=0;
  
}
  
}


  
}





