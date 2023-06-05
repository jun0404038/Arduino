/////////////////////IMU//////////////////////

#include <Wire.h>
#include <LSM303.h>


#define THRESHOLD_ANGLE1  7
#define THRESHOLD_ANGLE2  3


LSM303 compass;


float heading_angle      = 0.0;
float init_heading_angle = 17.0;  //초기방향
float target_heading_angle = 0.0;
float heading_angle_error = 0.0;  //error값
////////////////////////////////////////////////////////

float front_sonar      =  0.0;
float left_sonar       =  0.0;
float right_sonar      =  0.0;

float front_sonar1     = 0.0;
float left_sonar1      = 0.0;
float right_sonar1     = 0.0;

float front_sonar2     = 0.0;
float left_sonar2      = 0.0;
float right_sonar2     = 0.0;

float front_sonar3     = 0.0;
float left_sonar3      = 0.0;
float right_sonar3     = 0.0;
/////////////////// Mission ////////////////
#define NO_MISSION 4
bool mission_flag[NO_MISSION] = {-1,0,1,2};
int flag_runner = -1; // 0번 라인 following / 1번 MAze / 2번 장애뭉 우회
// 초음파 센서 + 라인센서호 판단
// 라인 Folloing : 초음파센서 장애물 없어야함
//1번 maze : chdmavkrk ㅡㅐㅇㄷ(오른쪽, 왼쪽 ,직진)

////////////// line sensor ///////////////// 

int line_sensor_pin[5] = { 34, 35, 36, 37, 38}; // line_sensor_pin 배열을 선언
int line_sensor[5] = {  0,  0,  0,  0,  0}; // line_sensor 값 배열을 선언

 int read_line_sensor(void)
 {
  int i, line_index;
  int sum = 0; // line sensor 에서 불이 들어온 sensor의 갯수
  for(i = 0; i < 5; i++)
    {
      line_sensor[i] = digitalRead(line_sensor_pin[i]);
      sum += line_sensor[i];
      Serial.print(line_sensor[i]);  Serial.print("  ");
    }
    Serial.print("sum  ");   Serial.println(sum);
 
 if(sum == 1)
 {
  if(line_sensor[0]==1) line_index = -4;
  if(line_sensor[1]==1) line_index = -2;
  if(line_sensor[2]==1) line_index =  0;
  if(line_sensor[3]==1) line_index =  2;
  if(line_sensor[4]==1) line_index =  4;
 }
 else if(sum == 2)
 {
  if((line_sensor[0]==1) && (line_sensor[1]== 1)) line_index = -3;
  if((line_sensor[1]==1) && (line_sensor[2]== 1)) line_index = -1;
  if((line_sensor[2]==1) && (line_sensor[3]== 1)) line_index =  1;
  if((line_sensor[3]==1) && (line_sensor[4]== 1)) line_index =  3;
 }
 else if (sum == 5)
 {
  line_index = -10;
 }
 else
 {
  
 }
 Serial.print("line_index: ");
 Serial.print(line_index);
 Serial.println("");
 delay(100);

 return line_index;
 }

 void line_following(int line_index)
{
  switch(line_index)
  {
    case -4:
      // 왼쪽으로 큰 각도로 회전
          motor_R_control(80);
          motor_L_control(-40);
          break;
    case -3:
      // 왼쪽으로 약간 각도로 회전
          motor_R_control(80);
          motor_L_control(5);
          break;

    case -2:
      // 왼쪽으로 약간 각도로 회전
          motor_R_control(80);
          motor_L_control(35);
          break;

    case -1:
      // 왼쪽으로 작은 각도로 회전
          motor_R_control(80);
          motor_L_control(60);
          break;

    case 0:  
      // 직진
          motor_R_control(80);
          motor_L_control(80);
          break;

    case 1:
      // 오른쪽으로 작은 각도로 회전
          motor_R_control(60);
          motor_L_control(80);
          break;

    case 2:
      // 오른쪽으로 약간 각도로 회전
          motor_R_control(60);
          motor_L_control(80);
          break;

    case 3:
      // 오른쪽으로 약간 각도로 회전
          motor_R_control(-50);
          motor_L_control(80);
          break;

    case 4:
      // 오른쪽으로 큰 각도로 회전
          motor_R_control(-40);
          motor_L_control(80);
          break;

    case -10:
      // 선을 발견하지 못한 경우, 정지
          motor_R_control(0);
          motor_L_control(0);
          break;
  }
}

 void line_run(void)
 { 
  check_maze_status();
  int flag = 1;
  int line_index;
  while(flag)
  {
    line_index = read_line_sensor();
    read_sonar_sensor();
    line_following(line_index);
    
    if(front_sonar < 200)
    {
        motor_R_control(-50);motor_L_control(-50);
        delay(100);
        motor_R_control(0);motor_L_control(0);
        flag = 0;
    }
  }
 }

 void obstacle_run(void)
 {
  
  target_heading_angle;
  //turn
  delay(1000);
  target_heading_angle;
  //turn()
  bool flag = 1;

  while(flag)
  {
   int line_index;
   line_index = read_line_sensor();
   if (line_index >0)
   {
     motor_R_control(-50);motor_L_control(-50);
      delay(100);
      motor_R_control(0);motor_L_control(0);
      flag = 0;
   }
  }
 }
 ////////////////////PID///////////////////////

double Kp=2, Ki=0.0, Kd=0;


////////////////////Driving///////////////////////

float t_angle[4] = { 0, 90, 180, 270};


////////////////////////////////////////////////////////////////////////

#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 150 // Maximum distance (in cm) to ping.
#define WALL_GAP_DISTANCE      450 //mm  단위
#define WALL_GAP_DISTANCE_HALF 250 //mm  단위
#define MOTOR_PWM_OFFSET 10

#define Front 0
#define Left  1
#define Right 2

#define TRIG1 2  // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 3  // 초음파 센서 1번 Echo 핀 번호

#define TRIG2 4  // 초음파 센서 2번 Trig 핀 번호
#define ECHO2 5  // 초음파 센서 2번 Echo 핀 번호

#define TRIG3 6  // 초음파 센서 3번 Trig 핀 번호
#define ECHO3 7  // 초음파 센서 3번 Echo 핀 번호

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
NewPing(TRIG1, ECHO1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
NewPing(TRIG2, ECHO2, MAX_DISTANCE),
NewPing(TRIG3, ECHO3, MAX_DISTANCE)
};




//////////////////L298N//////////////////////////////
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13
//////////Maze Status///////////////
int maze_status = 0;

void setup() {
  int i;
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
for(i=0;i<5;i++)
{
  pinMode(line_sensor_pin[i], INPUT);
}

  Serial.begin(115200); // 통신 속도를 115200으로 정의함

  Wire.begin();       //IMU initiallize
  //compass.init();
  //compass.enableDefault();

/*
  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };
 */
 compass.m_min = (LSM303::vector<int16_t>) {-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>) {+32767, +32767, +32767};

}

void motor_R_control(int motor_speed_R) // 모터 A의 속도(speed)제어
  {
    if (motor_speed_R >= 0)
    {
      digitalWrite(IN1, HIGH);         //모터의 방향 제어
      digitalWrite(IN2, LOW);
      if(motor_speed_R>=255) motor_speed_R = 255;
      analogWrite(ENA, motor_speed_R); //모터의 속도 제어
    }
    else
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      if(motor_speed_R<=-255) motor_speed_R = -255;
      analogWrite(ENA, -motor_speed_R);
    }
  }


  void motor_L_control(int motor_speed_L) // 모터 A의 속도(speed)제어
  {
    if (motor_speed_L >= 0)
    {
      digitalWrite(IN3, LOW);         //모터의 방향 제어
      digitalWrite(IN4, HIGH);
      if(motor_speed_L>=255) motor_speed_L = 255;
      analogWrite(ENB, motor_speed_L); //모터의 속도 제어
    }
    else
    {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      if(motor_speed_L<=-255) motor_speed_L = -255;
      analogWrite(ENB, -motor_speed_L);
    }
  }
void check_maze_status(void)
{
  if ((left_sonar >= 0) && (left_sonar <= 200) && (right_sonar >= 0) && (right_sonar <= 200) && (front_sonar >= 0) && (front_sonar <= 125))
  { //3면이 다 막힌경우
    maze_status = 4;
    Serial.println("maze_status = 4");

  }
  else if ( (left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE_HALF) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 1;
    Serial.println("maze_status = 1");
  }
  else if ((left_sonar >= 0) && (left_sonar <= 250) && (front_sonar >= 0) && (front_sonar <= 125))
  {
    maze_status = 2;
    Serial.println("maze_status = 2");

  }
  else if ((right_sonar >= 0) && (right_sonar <= 220) && (front_sonar >= 0) && (front_sonar <= 125))
  {
    maze_status = 3;
    Serial.println("maze_status = 3");

  }
  else if (front_sonar <= 200)
  {
    maze_status = 5;
    Serial.println("maze_status = 5");

  }
  else
  {
    maze_status = 0;
    Serial.println("maze_status = 0;");
  }
  
}

// 먼저 left_pwm =0;   right_pwm = 100;   으로 해서 왼쪽 오른쪽 방향 찾기
void wall_collision_avoid (int base_speed) {
  float error = 0.0;    // 스티어 민감도 함수 초기화
  float Kp = 0.5;       // 모터 속도 민감도
  int pwm_control = 0;  // PWM 제어 초기화
  int right_pwm = 0;    // 오른쪽 모터 속도 초기화
  int left_pwm = 0;     // 왼쪽 모터 속도 초기화
   
  error = (right_sonar - left_sonar); // 벽면 간의 거리
  error = Kp * error;   // 스티어 양 조절
 
  if (error >= 23) error = 23;  // 과도한 스티어링 방지
  if (error <= -23) error = -23;
 
  right_pwm = 80 - error;  // 오른쪽 바퀴 회전수 조절
  left_pwm = 80 + error;   // 왼쪽 바퀴 회전수 조절
 
  if (right_pwm <= 0) right_pwm = 0;
  if (left_pwm <= 0) left_pwm = 0;

  if (right_pwm >= 255) right_pwm = 255;
  if (left_pwm >= 255) left_pwm = 255;
 
}
void imu_rotation(void)
{
  bool flag = 1; //bool type은 0,1
   while(flag)
   {
      read_sonar_sensor();
      read_imu_sensor();
      
      if(heading_angle_error > THRESHOLD_ANGLE1) // 반시계방향으로 회전
      {
          motor_L_control(100); 
          motor_R_control(-90);
      }
      else if((heading_angle_error >=THRESHOLD_ANGLE2) && (heading_angle_error <=THRESHOLD_ANGLE1)) // 
      {
        motor_L_control(60); 
        motor_R_control(-60);
      }
      else if((heading_angle_error >-THRESHOLD_ANGLE2) && (heading_angle_error <THRESHOLD_ANGLE2)) // 정지
      {
        motor_L_control(0); 
        motor_R_control(0);
        flag = 0;
      }
      else if((heading_angle_error >=-THRESHOLD_ANGLE1) && (heading_angle_error <=-THRESHOLD_ANGLE2)) // 
      {
        motor_L_control(-60); 
        motor_R_control(60);
      }
      else // heading_angle_error <-THRESHOLD_ANGLE // 시계방향으로 회전
      {
        motor_L_control(-100); 
        motor_R_control(90);
      }

  Serial.print("Heading Angle Error : ");                           //시리얼 모니터 확인용
  Serial.print(heading_angle_error); //heading_angle error 표시
  Serial.print(" = ");
  Serial.print(target_heading_angle);
  Serial.print(" - ");
  Serial.println(heading_angle); //heading_angle 표시
  


 
    }
}
void read_sonar_sensor(void)
{
  float front_sonar1 = sonar[Front].ping_cm() * 10;
  float left_sonar1  = sonar[Left].ping_cm() * 10;
  float right_sonar1 = sonar[Right].ping_cm() * 10;

  float front_sonar2 = sonar[Front].ping_cm() * 10;
  float left_sonar2  = sonar[Left].ping_cm() * 10;
  float right_sonar2 = sonar[Right].ping_cm() * 10;

  float front_sonar3 = sonar[Front].ping_cm() * 10;
  float left_sonar3  = sonar[Left].ping_cm() * 10;
  float right_sonar3 = sonar[Right].ping_cm() * 10;

  front_sonar = (front_sonar1 + front_sonar2 + front_sonar3) / 3;
  left_sonar  = (left_sonar1 + left_sonar2 + left_sonar3) / 3;
  right_sonar = (right_sonar1 + right_sonar2 + right_sonar3) / 3;

  if (front_sonar == 0.0) front_sonar = MAX_DISTANCE ;
  if (left_sonar  == 0.0) left_sonar = MAX_DISTANCE ;
  if (right_sonar == 0.0) right_sonar = MAX_DISTANCE ;

  Serial.print("L:"); Serial.print(left_sonar); Serial.print(" ");
  Serial.print("F:"); Serial.print(front_sonar); Serial.print(" ");
  Serial.print("R:"); Serial.println(right_sonar);

  
}
void read_imu_sensor(void)
  {
     compass.read();
     float heading1 = compass.heading();
     compass.read();
     float heading2 = compass.heading();
     heading_angle = (heading1 + heading2) / 2;
     heading_angle = 360 - heading_angle; // 회전 좌표계를 반시계 방향으로 할 것    
     heading_angle_error = target_heading_angle - heading_angle;

     if(heading_angle_error > 180)
     {
         heading_angle_error = heading_angle_error - 360;
     }
     else if(heading_angle_error < -180)
     {
         heading_angle_error = heading_angle_error + 360;  
     }
     else
     {


     }
    Serial.print("Heading Angle Error : ");
    Serial.print(heading_angle_error); //heading_angle error 표시
    Serial.print(" = ");
    Serial.print(target_heading_angle);
    Serial.print(" - ");
    Serial.println(heading_angle); //heading_angle 표시
}
void run_heading_angle(void)
  {
     bool flag = 1;
     double Output;
    
     while(flag)
     {
       read_sonar_sensor();
       read_imu_sensor();
       Output = Kp * heading_angle_error;

       motor_L_control((90 + Output )*1.3); 
       motor_R_control(90 - Output );

       if(front_sonar < 200) 
       {
         flag = 0;
         motor_L_control(0); 
         motor_R_control(0);
       }
     }
     
  }

void loop ()
{  
  
  int l_index = -10;
  l_index = read_line_sensor;
  read_line_sensor();
  line_following(read_line_sensor());
  
}
void loop1()
{
  int l_index = -10;

  

  // put your main code here, to run repeatedly:


  if (mission_flag[0]==0) //line 추적
  {
    read_line_sensor();
    if((front_sonar >= 0) && (front_sonar <= 100))
    {
      mission_flag[0]=1;
      mission_flag[1]=0;
    }
    else if ( (l_index == 0) && (front_sonar >= 0) && (right_sonar <= 100) && (front_sonar >= 0) && (left_sonar <= 100) )
    {
      mission_flag[2]=0;
    }
    else
    {
      line_following(read_line_sensor());
    }
  }
  
  if(mission_flag[1] == 0) // line 추적
  {
    int obstacle_avoidance();
    mission_flag[0]=1;  //라인 추적
  }

 if(mission_flag[2]==0) // IMU 주행
  {
    int imu_run();
    mission_flag[2]=1;
    mission_flag[3]=0;
  }

if (mission_flag[3]==0) //line 추적
  {
    int sum = 0;
    if ( (front_sonar < 200) && (sum == 5) )
    {
      mission_flag[0]=1;
      mission_flag[1]=0;
    }
  }
}
