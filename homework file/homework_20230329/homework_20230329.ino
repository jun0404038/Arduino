///////////////////////////// sonar //////////////////////////////////
#include <NewPing.h>

#define SONAR_NUM 1      // Number of sensors.
#define MAX_DISTANCE 150 // Maximum distance (in cm) to ping.


#define Front 0


#define TRIG1 2 //초음파 센서 1번 Trig 핀 번호
#define ECHO1 3 //초음파 센서 1번 Echo 핀 번호

NewPing sonar(TRIG1, ECHO1, MAX_DISTANCE);

///////////////////////////// L298 //////////////////////////////////////

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT); 
  Serial.begin(115200); 
}

long sonar_front(void)
{
  long duration, distance;

  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);

  duration = pulseIn(ECHO1, HIGH);
  distance = ((float)(340 * duration) / 1000) / 2;
  
  return distance;
}
void loop() 
{
  float front_sonar = 0.0;

  front_sonar = sonar.ping_cm()*10;
  if(front_sonar == 0.0)  front_sonar = MAX_DISTANCE;
   
  Serial.print("Distance: ");
  Serial.print(front_sonar);  //   거리 출력
  Serial.println("mm");

  if((front_sonar > 0) && (front_sonar <= 200.0))
  {  // 200mm보다 가까우면 LED 켜짐, 멀어지면 LED 꺼짐
    digitalWrite(LED_BUILTIN, HIGH);   // LED 켜짐
    delay(1000);  //  1초동안 켜짐
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);    // LED 꺼짐
    delay(1000);  //  1초동안 꺼짐
  }
}
