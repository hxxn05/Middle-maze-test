#include <NewPing.h>  // 초음파 센서 라이브러리를 포함

#define MAX_DISTANCE 150       // 초음파 센서의 최대 거리 (cm 단위)
#define WALL_GAP_DISTANCE 400  // 벽과의 간격 (mm 단위)
#define WALL_GAP_DISTANCE_HALF 210  // 벽과의 절반 간격 (mm 단위)
#define MOTOR_PWM_OFFSET 10  // 모터 속도 조정 오프셋

#define FTRIGGER_PIN 12  // 앞쪽 초음파 센서의 트리거 핀
#define FECHO_PIN 13  // 앞쪽 초음파 센서의 에코 핀
#define LTRIGGER_PIN 16  // 왼쪽 초음파 센서의 트리거 핀
#define LECHO_PIN 17  // 왼쪽 초음파 센서의 에코 핀
#define RTRIGGER_PIN 14  // 오른쪽 초음파 센서의 트리거 핀
#define RECHO_PIN 15  // 오른쪽 초음파 센서의 에코 핀

NewPing sonar_front(FTRIGGER_PIN, FECHO_PIN, MAX_DISTANCE);  // 앞쪽 초음파 센서 객체 생성
NewPing sonar_left(LTRIGGER_PIN, LECHO_PIN, MAX_DISTANCE);  // 왼쪽 초음파 센서 객체 생성
NewPing sonar_right(RTRIGGER_PIN, RECHO_PIN, MAX_DISTANCE);  // 오른쪽 초음파 센서 객체 생성

float front_sonar = 0.0;  // 앞쪽 초음파 센서 거리 값
float left_sonar = 0.0;  // 왼쪽 초음파 센서 거리 값
float right_sonar = 0.0;  // 오른쪽 초음파 센서 거리 값

#define ENR 6  // 오른쪽 모터 속도 제어 핀
#define IN1 11  // 오른쪽 모터 방향 제어 핀 1
#define IN2 10  // 오른쪽 모터 방향 제어 핀 2
#define IN3 9  // 왼쪽 모터 방향 제어 핀 1
#define IN4 8  // 왼쪽 모터 방향 제어 핀 2
#define ENL 7  // 왼쪽 모터 속도 제어 핀

int maze_status = 0;  // 미로 상태 변수

void setup() 
{
  pinMode(ENR, OUTPUT);  // 오른쪽 모터 속도 제어 핀을 출력 모드로 설정
  pinMode(IN1, OUTPUT);  // 오른쪽 모터 방향 제어 핀 1을 출력 모드로 설정
  pinMode(IN2, OUTPUT);  // 오른쪽 모터 방향 제어 핀 2를 출력 모드로 설정
  pinMode(IN3, OUTPUT);  // 왼쪽 모터 방향 제어 핀 1을 출력 모드로 설정
  pinMode(IN4, OUTPUT);  // 왼쪽 모터 방향 제어 핀 2를 출력 모드로 설정
  pinMode(ENL, OUTPUT);  // 왼쪽 모터 속도 제어 핀을 출력 모드로 설정
  Serial.begin(115200);  // 시리얼 통신 속도를 115200으로 설정
}

void motor_A_control(int direction_a, int motor_speed_a) 
{
  if(direction_a == HIGH)  // 방향이 앞으로일 때
  {
    digitalWrite(IN1, LOW);  // 오른쪽 모터 방향 제어 핀 1을 LOW로 설정
    digitalWrite(IN2, HIGH);  // 오른쪽 모터 방향 제어 핀 2를 HIGH로 설정
    analogWrite(ENR, motor_speed_a);  // 오른쪽 모터 속도 설정
  }
  else  // 방향이 뒤로일 때
  {
    digitalWrite(IN1, HIGH);  // 오른쪽 모터 방향 제어 핀 1을 HIGH로 설정
    digitalWrite(IN2, LOW);  // 오른쪽 모터 방향 제어 핀 2를 LOW로 설정
    analogWrite(ENR, motor_speed_a);  // 오른쪽 모터 속도 설정
  }
}

void motor_B_control(int direction_b, int motor_speed_b) 
{
  if(direction_b == HIGH)  // 방향이 앞으로일 때
  {
    digitalWrite(IN3, LOW);  // 왼쪽 모터 방향 제어 핀 1을 LOW로 설정
    digitalWrite(IN4, HIGH);  // 왼쪽 모터 방향 제어 핀 2를 HIGH로 설정
    analogWrite(ENL, motor_speed_b);  // 왼쪽 모터 속도 설정
  }
  else  // 방향이 뒤로일 때
  {
    digitalWrite(IN3, HIGH);  // 왼쪽 모터 방향 제어 핀 1을 HIGH로 설정
    digitalWrite(IN4, LOW);  // 왼쪽 모터 방향 제어 핀 2를 LOW로 설정
    analogWrite(ENL, motor_speed_b);  // 왼쪽 모터 속도 설정
  }
}

void check_maze_status(void)
{
  if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 4;  // 모든 방향에 벽이 있을 때
    Serial.println("maze_status = 4");
  }
  else if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 1;  // 앞쪽에 벽이 없을 때
    Serial.println("maze_status = 1");
  }
  else if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 2;  // 오른쪽에 벽이 없을 때
    Serial.println("maze_status = 2");
  }
  else if((right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 3;  // 왼쪽에 벽이 없을 때
    Serial.println("maze_status = 3");
  }
  else
  {
    maze_status = 0;  // 미로 상태를 알 수 없을 때
    Serial.println("maze_status = 0");
  }
}

void wall_collision_avoid(int base_speed)
{
  float error = 0.0;  // 에러 값 초기화
  float Kp = 0.2;  // 비례 상수 (회전 강도 조정)
  int right_pwm = 0;  // 오른쪽 모터 속도
  int left_pwm = 0;  // 왼쪽 모터 속도
  error = (right_sonar - left_sonar);  // 왼쪽 벽 회피를 우선으로 하는 에러 값
  error = Kp * error;  // 에러 값 조정

  if(error >= 50) error = 50;  // 에러 값 최대값 설정
  if(error <= -50) error = -50;  // 에러 값 최소값 설정

  right_pwm = base_speed - error;  // 오른쪽 모터 속도 계산
  left_pwm = base_speed + error;  // 왼쪽 모터 속도 계산

  if(left_pwm <= 0) left_pwm = 0;  // 왼쪽 모터 속도 최소값 설정
  if(right_pwm <= 0) right_pwm = 0;  // 오른쪽 모터 속도 최소값 설정

  if(left_pwm >= 255) left_pwm = 120;  // 왼쪽 모터 속도 최대값 설정
  if(right_pwm >= 255) right_pwm = 120;  // 오른쪽 모터 속도 최대값 설정

  motor_A_control(HIGH, left_pwm);  // 왼쪽 모터 앞으로
  motor_B_control(HIGH, right_pwm);  // 오른쪽 모터 앞으로
}

void loop() 
{
  front_sonar = sonar_front.ping_cm() * 10;  // 앞쪽 초음파 센서 거리 값 측정
  left_sonar = sonar_left.ping_cm() * 10;  // 왼쪽 초음파 센서 거리 값 측정
  right_sonar = sonar_right.ping_cm() * 10;  // 오른쪽 초음파 센서 거리 값 측정

  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE * 10;  // 앞쪽 거리 값이 0일 때 최대 거리로 설정
  if(left_sonar == 0.0) left_sonar = MAX_DISTANCE * 10;  // 왼쪽 거리 값이 0일 때 최대 거리로 설정
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE * 10;  // 오른쪽 거리 값이
