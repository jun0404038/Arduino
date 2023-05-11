/////////////////////// sonar ///////////////////////
#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define WALL_GAP_DISTANCE 400 // mm
#define WALL_GAP_DISTANCE_HALF 200 // mm
#define MOTOR_PWM_OFFSET 10
#define MOTOR_SPEED 105

#define Front 0
#define Left  1
#define Right 2

#define TRIG1 2  //  1번 초음파 센서 Trig 핀 번호
#define ECHO1 3  //  1번 초음파 센서 Echo 핀 번호

#define TRIG2 4  //  2번 초음파 센서 Trig 핀 번호
#define ECHO2 5  //  2번 초음파 센서 Echo 핀 번호

#define TRIG3 6  //  3번 초음파 센서 Trig 핀 번호
#define ECHO3 7  //  3번 초음파 센서 Echo 핀 번호

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(TRIG1, ECHO1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIG2, ECHO2, MAX_DISTANCE),
  NewPing(TRIG3, ECHO3, MAX_DISTANCE)
};

float front_sonar, left_sonar, right_sonar = 0.0;

///////////////L298/////////////////
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

////////////////////// Maze Status ////////////////////
int maze_status = 0;


void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(115200); // 통신 속도를 115200으로 정의
}
void motor_A_control(int direction_a, int motor_speed_a) {  //모터 A의 방향(direction)과 속도(speed)제어

  if (direction_a == HIGH) {
    digitalWrite(IN1, HIGH);  // 모터의 방향 제어
    digitalWrite(IN2, LOW);   //
    analogWrite(ENA, motor_speed_a);  // 모터의 속도 제어
  }

  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, motor_speed_a);
  }

}

void motor_B_control(int direction_b, int motor_speed_b) {  //모터 B의 방향(direction)과 속도(speed)제어

  if (direction_b == HIGH) {
    digitalWrite(IN3, LOW);   // 모터의 방향 제어
    digitalWrite(IN4, HIGH);  
    analogWrite(ENB, motor_speed_b);  // 모터의 속도 제어
  }

  else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, motor_speed_b);
  }

}

void check_maze_status(void) {
  if ((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE_HALF) && (right_sonar >= 0) && (right_sonar <= 200) && (front_sonar >= 0) && (front_sonar <= 140)) { // 세 면이 다 막힌 경우
    maze_status = 4;
    Serial.println("maze_status = 4");
  }
  else if ((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE_HALF) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE_HALF) && (front_sonar >= 150)) {
    maze_status = 1;
    Serial.println("maze_status = 1");
  }
  else if ((left_sonar >= 0) && (left_sonar <= 220) && (front_sonar >= 0) && (front_sonar <= 180)) {
    maze_status = 2;
    Serial.println("maze_status = 2");
  }
  else if ((right_sonar >= 0) && (right_sonar <= 230) && (front_sonar >= 0) && (front_sonar <= 200)) {
    maze_status = 3;
    Serial.println("maze_status = 3");
  }
  else {
    maze_status = 0;
    Serial.println("maze_status = 0");
  }
}
// 먼저 left_pwm = 0; right_pwm = 100; 으로 해서 왼쪽 오른쪽 방향 찾기

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
 
  right_pwm = MOTOR_SPEED - error;  // 오른쪽 바퀴 회전수 조절
  left_pwm = MOTOR_SPEED + error;   // 왼쪽 바퀴 회전수 조절
 
  if (right_pwm <= 0) right_pwm = 0;
  if (left_pwm <= 0) left_pwm = 0;

  if (right_pwm >= 255) right_pwm = 255;
  if (left_pwm >= 255) left_pwm = 255;
 
  motor_A_control(HIGH, right_pwm);
  motor_B_control(HIGH, left_pwm);
}

void loop() {
  front_sonar = sonar[Front].ping_cm() * 10; // 전방 센서 측정
  left_sonar = sonar[Left].ping_cm() * 12;  // 좌측 센서 측정
  right_sonar = sonar[Right].ping_cm() * 10; // 우측 센서 측정

  if (front_sonar == 0.0)  front_sonar = MAX_DISTANCE;
  if (left_sonar == 0.0)  left_sonar = MAX_DISTANCE;
  if (right_sonar == 0.0)  right_sonar = MAX_DISTANCE;

  Serial.print("L: "); Serial.print(left_sonar);   Serial.print("  ");
  Serial.print("F: "); Serial.print(front_sonar);  Serial.print("  ");
  Serial.print("R: "); Serial.println(right_sonar);

  check_maze_status();

  if(maze_status == 4){
    motor_A_control(HIGH, 0); // 오른쪽 전진
    motor_B_control(LOW, 0);  // 왼쪽 후진
    delay(300);
     // 180도 반시계 방향으로 회전
    Serial.println("Rotate CCW");
    motor_A_control(HIGH, 100); // 오른쪽 전진
    motor_B_control(LOW, 100);  // 왼쪽 후진
    delay(1400);  
    motor_A_control(HIGH, 0); // 오른쪽 전진
    motor_B_control(LOW, 0);  // 왼쪽 후진
    delay(300);  
    Serial.println("Rotate CW");
    motor_A_control(LOW, 100); // 오른쪽 전진
    motor_B_control(HIGH, 100);
  }
  else if(maze_status == 3){
    motor_A_control(HIGH, 0); // 오른쪽 전진
    motor_B_control(LOW, 0);  // 왼쪽 후진
    delay(300);
    // 90도 반시계 방향으로 회전
    Serial.println("Rotate CCW");
    motor_A_control(HIGH, 100); // 오른쪽 전진
    motor_B_control(LOW, 100);  // 왼쪽 후진
    delay(650);
    motor_A_control(HIGH, 0); // 오른쪽 전진
    motor_B_control(LOW, 0);  // 왼쪽 후진
    delay(300);
    Serial.println("Rotate CW");
    motor_A_control(LOW, 100); // 오른쪽 전진
    motor_B_control(HIGH, 100);
  }
  else if(maze_status == 2){
    motor_A_control(HIGH, 0); // 오른쪽 전진
    motor_B_control(LOW, 0);  // 왼쪽 후진
    delay(300);
    // 90도 시계 방향으로 회전
    Serial.println("Rotate CW");
    motor_A_control(LOW, 100); // 오른쪽 전진
    motor_B_control(HIGH, 100);  // 왼쪽 후진
    delay(650);
    motor_A_control(HIGH, 0); // 오른쪽 전진
    motor_B_control(LOW, 0);  // 왼쪽 후진
    delay(300);
    Serial.println("Rotate CW");
    motor_A_control(LOW, 100); // 오른쪽 전진
    motor_B_control(HIGH, 100);
  }
 
  else {
    // 직진
    Serial.println("Go straight");
    wall_collision_avoid(MOTOR_SPEED);
  }
}
