#include <SoftwareSerial.h> // 상위 제어기와 통신하기 위한 라이브러리 
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <erp42_msgs/DriveCmd.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

//조향 변수
int potential_pot = A3;    
int val;
int potential_val;
float kp = 0.2;
float ki = 0.00000;
float kd = 2.00;
float Theta, Theta_d;
int dt;
unsigned long t;
unsigned long t_prev = 0;
int val_prev =0;
float e, e_prev = 0, inte, inte_prev = 0;
float Vmax = 24;
float Vmin = -24;
float V = 0.1;
int St_M = 500;// 직진 조향값

//엔코더 변수
volatile unsigned int temp, PlateCount = 32500; //This variable will increase or decrease depending on the rotation of encoder
const int pulsesPerRevolution = 100;
volatile long DriveCount = 32500; // 엔코더 값 변수
const byte interruptPinA = 20;  // 구동모터의 엔코더센서 핀번호
const byte interruptPinB = 21;
int currentCounter = 0;
int previousCounter = 0;
float rpm = 0;

//시간 변수
unsigned long previousTime = 0;
const long sampleInterval = 100; // 샘플링 간격을 밀리초 단위로 설정 (1초)
int currentTime = 0;

//모터 드라이버 변수
const byte DirSteer = 5;
const byte PWMSteer = 6; // 조향 모터의 모터드라이브 PWM,DIR 핀번호 
const byte DirDrive = 9;
const byte PWMDrive = 10; // 구동 모터의 모터드라이브 PWM, DIR 핀번호
const byte DirPlate = 11;
const byte PWMPlate = 12; // 플레이트 모터의 모터드라이브 PWM, DIR 핀번호

int Direction = 0;
int number = 0;

float Speed = 0;
int tmpSpeed = 0;
int Steer = St_M;
int HSteer,HSpeed; // 상위 제어기와 통신을 위한 변수 
byte ESTOP, Mode,HGear;
byte State = 0;
byte ALIVE = 0;

int Thro,Aux,Gear,ALIE,T,A,G,AL; // 조종기를 위한 변수
int tempAux2 = 0;
int tempAux1 = 0; // 튀는 값으로 인한 Auto Mode 전환 방지

int target_speed = 0;
int target_angle = 0;

int PlateSpeed = 0;

void SteerCon(float Q) { // 조향모터의 제어를 위한 함수, Q = 조향 목표값  
    val = Q;                           
    if(val>St_M+300){  // 조향 최대값 제한
      val=St_M+300;
    }
    else if(val<St_M-300){ // 조향 최소값 제한 
      val= St_M-300;
    }
    potential_val = analogRead(potential_pot);               // Read V_out from Feedback Pot
    t = millis();
    dt = (t - t_prev);                                  // Time step
    Theta = val;                                        // Theta= Actual Angular Position of the Motor
    Theta_d = potential_val;                              // Theta_d= Desired Angular Position of the Motor

    e = Theta_d - Theta;                                // Error
    inte = inte_prev + (dt * (e + e_prev) / 2);         // Integration of Error
    V = kp * e + ki * inte + (kd * (e - e_prev) / dt) ; // Controlling Function

    if (V > Vmax) {
        V = Vmax;
       inte = inte_prev;
     }
    if (V < Vmin) {
      V = Vmin;
      inte = inte_prev;
      val_prev= val;
     }
    int PWMval = int(255 * abs(V) / Vmax);
    if (PWMval > 200) {
      PWMval = 200;
    }
    if (V > 0.8) {
      digitalWrite(DirSteer, LOW);
      analogWrite(PWMSteer, PWMval);
    }
    else if (V < -0.8) {
      digitalWrite(DirSteer, HIGH);
      analogWrite(PWMSteer, PWMval);
    }
    else {
      digitalWrite(DirSteer, LOW);
      analogWrite(PWMSteer, 0);
    }
    t_prev = t;
    inte_prev = inte;
    e_prev = e;
}

void Mdrive(const char* m, int q) {
    int Dir;
    int PWM;
    if(strcmp(m, "Drive") == 0) {
        Dir = DirDrive;
        PWM = PWMDrive;
    } else if(strcmp(m, "Plate") == 0) {
        Dir = DirPlate;
        PWM = PWMPlate;
    } else {
        return;
    }

    if(q > 0){
        digitalWrite(Dir, 1);
        analogWrite(PWM, q);
    } else if(q < 0){
        digitalWrite(Dir, 0);
        analogWrite(PWM, abs(q));
    } else {
        digitalWrite(Dir, 0);
        analogWrite(PWM, 0);
    }
}

void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if (digitalRead(3) == LOW) {
    PlateCount++;
  } else {
    PlateCount--;
  }
}

void ai1() {
  // ai1 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if (digitalRead(2) == LOW) {
    PlateCount--;
  } else {
    PlateCount++;
  }
}

void ISR_EncoderA() { // 엔코더 값을 저장하기 위한 내부 인터럽트 서비스 함수
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      DriveCount++;
    }
    else {
      DriveCount--;
    }
  }

  else {
    if (PinA == HIGH) {
      DriveCount--;
    }
    else {
      DriveCount++;
    }
  }
}
void ISR_EncoderB() { // 엔코더 값을 저장하기 위한 내부 인터럽트 서비스 함수
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      DriveCount--;
    }
    else {
      DriveCount++;
    }
  }

  else {
    if (PinB == HIGH) {
      DriveCount++;
    }
    else {
      DriveCount--;
    }
  }
}

void controller() {
  Aux = pulseIn(A2, HIGH, 50000);   //  아날로그2번 핀의 pwm입력신호가 LOW로 바뀐 순간부터 HIGH로 바뀌는 순간까지의 경과시간을 마이크로초 단위로 Aux변수에 저장. 만약, 50ms시간동안 펄스의 변화가 없다면, 0값이 Aux변수에 저장된다.
  Thro = pulseIn(A1, HIGH, 50000);  //  아날로그1번 핀의 pwm입력신호가 LOW로 바뀐 순간부터 HIGH로 바뀌는 순간까지의 경과시간을 마이크로초 단위로 Thro변수에 저장. 만약, 50ms시간동안 펄스의 변화가 없다면, 0값이 Thro변수에 저장된다.
  ALIE = pulseIn(A0, HIGH, 50000);  //  아날로그0번 핀의 pwm입력신호가 LOW로 바뀐 순간부터 HIGH로 바뀌는 순간까지의 경과시간을 마이크로초 단위로 Rudd변수에 저장. 만약, 50ms시간동안 펄스의 변화가 없다면, 0값이 Rudd변수에 저장된다.
  T = Thro/50; Thro = T*50; A = Aux/50; Aux = A*50; G = Gear/50; Gear = G*50; AL = ALIE/50; ALIE = AL*50; // 조종기 신호 안정화
  if(ALIE>1700){ ALIE=1700; } else if(ALIE<1200){ ALIE = 1200; } // 조종기 ALIE(조향) 신호 최대 최소값 제한
  if(Thro>1850){ Thro=1850; }  else if(Thro<1050){ ESTOP = 1;} else{ESTOP=0;} // 조종기 THRO(구동) 신호 최대값 제한, Throttle Cut할때 ESTOP신호 주도록 설정 
  if(Thro >= 1450) {
    tmpSpeed = map(Thro, 1450, 1650, 0, 150);
    Speed = tmpSpeed;
  }
  else if(Thro < 1400){
    tmpSpeed = map(Thro, 1400, 1100, 0, 150);
    Speed = -tmpSpeed;
  }
  if(Speed>255){ Speed = 255; } else if(abs(Speed)<10 or ESTOP == 1 ){ Speed = 0; } // Speed값 최대 최소값 제한 및 ESTOP상황에서 멈추도록 설정
  Steer = map(ALIE, 1200, 1700, St_M-300, St_M+300); // 조종기 ALIE 신호를 Steer값으로 변환
  if(Steer>St_M+300){ Steer = St_M+300; } else if(Steer<St_M-300){ Steer = St_M-300; } // Steer값 최대 최소값 제한
  else if(Steer>St_M-20 and Steer<St_M+20){ Steer = St_M;} // 조향 유지를 위한 DeadZone
  if(Aux != 1800 && Aux != 1050) {Aux = 1450;} 
  if(((tempAux1+tempAux2+Aux)/3) == 1800){ State = 0;} else if(((tempAux1+tempAux2+Aux)/3) == 1050){ State = 1; } // A : Auto, B : Manual
  tempAux2 = tempAux1;
  tempAux1 = Aux;
}

void getEncoder(){
  unsigned long currentTime = millis();
  
  if (currentTime - previousTime >= sampleInterval) {
    currentCounter = PlateCount;
    rpm = (currentCounter - previousCounter) * 60000 / (pulsesPerRevolution * sampleInterval);
    
    // RPM 값이 1000 이상인 경우를 0으로 대체
    if (rpm >= 1000) {
      rpm = 0;
    }
    previousCounter = PlateCount;
    previousTime = currentTime;
  }
}

ros::NodeHandle  nh;

void drive(const erp42_msgs::DriveCmd &msg){
  target_speed = msg.KPH;
  target_angle = msg.Deg;
}
std_msgs::Float64 stdmsg;

ros::Subscriber<erp42_msgs::DriveCmd> sub("/erp42_serial/drive", &drive );
ros::Publisher pub("espeed", &stdmsg);

void setup() {
  Serial.begin(57600); // 시리얼 통신을 위한 Baudrate 
  pinMode(PWMSteer, OUTPUT);
  pinMode(DirSteer, OUTPUT); // 조향 모터 
  pinMode(PWMDrive,OUTPUT);
  pinMode(DirDrive,OUTPUT); // 구동 모터
  pinMode(PWMPlate, OUTPUT);
  pinMode(DirPlate, OUTPUT); // 플레이트 모터
  pinMode(2, INPUT_PULLUP); // internal pullup input pin 2 
  pinMode(3, INPUT_PULLUP); // internalเป็น pullup input pin 3
  pinMode(21, INPUT_PULLUP); // internal pullup input pin 2 
  pinMode(20, INPUT_PULLUP); // internalเป็น pullup input pin 3
  attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);
  attachInterrupt(2, ISR_EncoderA, CHANGE); // 엔코더 값을 읽기 위한 인터럽트 서비스 루틴 
  attachInterrupt(3, ISR_EncoderB, CHANGE);  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  }


void loop() {
  controller();  // 조종기 명령값 수신 
  if(State == 1){ //AUTO상태일때 State == 1
    nh.spinOnce();
    //stdmsg = 엔코더 값;
    pub.publish(&stdmsg);
    HSpeed = target_speed;
    HSpeed = map(HSpeed, -20, 20, -255, 255);
    HSteer = target_angle; //상위 제어기에서 STEER값 저장 
    HSteer = map(HSteer, -20, 20, St_M+300, St_M-300);
    Mdrive("Drive", HSpeed);    //SPEED값 제어
    SteerCon(HSteer);    //STEER값 제어
    }
  else { // MANUAL상태일때 플랫폼 제어
    Mdrive("Drive", Speed); //SPEED값 제어
    SteerCon(Steer); //STEER값 제어
    getEncoder();
    if (Serial.available() > 0) {
      int newNumber = Serial.parseInt(); // 새로운 데이터 읽기
      if (newNumber != 0) {
        number = newNumber;
        PlateSpeed = map(number, -600, 600, -255, 255);
        Mdrive("Plate", PlateSpeed);
      }
    }
  }
  delay(1);

  
  /*
  Serial.print("AUX : ");
  Serial.print(Aux);
  Serial.print(" // Thro : ");
  Serial.print(Thro);
  Serial.print(" // ALIE : ");
  Serial.print(ALIE);
  Serial.print(" // State : ");
  Serial.print(State);
  Serial.print(" // ros_spd : ");
  Serial.print(target_speed);
  Serial.print(" // ros_ang : ");
  Serial.print(target_angle);
  Serial.print(" // PWM_spd : ");
  Serial.print(HSpeed);
  Serial.print(" // target_ang :");
  Serial.print(HSteer);
  Serial.print(" // manual_spd : ");
  Serial.print(Speed);
  Serial.print(" // manual_ang : ");
  Serial.print(Steer);
  Serial.print(" // current_ang : ");
  Serial.print(potential_val);
  */
  Serial.print(" // input : ");
  Serial.print(number);
  Serial.print(" // Plate Count : ");
  Serial.print(PlateCount);
  Serial.print(" // Pate Speed : ");
  Serial.println(PlateSpeed);
}
