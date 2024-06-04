#include <NewPing.h> // NewPing 라이브러리를 포함합니다.

#define MAX_DISTANCE 150       // 초음파 센서의 최대 거리 (cm 단위)입니다.
#define WALL_GAP_DISTANCE 400  // 벽 간격 거리 (mm 단위)입니다.
#define WALL_GAP_DISTANCE_HALF 230  // 벽 간격의 절반 거리 (mm 단위)입니다.
#define MOTOR_PWM_OFFSET 10  // 모터의 PWM 오프셋 값입니다.

#define FTRIGGER_PIN 12 // 전방 초음파 센서의 트리거 핀을 정의합니다.
#define FECHO_PIN 13 // 전방 초음파 센서의 에코 핀을 정의합니다.
#define LTRIGGER_PIN 16 // 좌측 초음파 센서의 트리거 핀을 정의합니다.
#define LECHO_PIN 17 // 좌측 초음파 센서의 에코 핀을 정의합니다.
#define RTRIGGER_PIN 14 // 우측 초음파 센서의 트리거 핀을 정의합니다.
#define RECHO_PIN 15 // 우측 초음파 센서의 에코 핀을 정의합니다.

NewPing sonar_front(FTRIGGER_PIN, FECHO_PIN, MAX_DISTANCE); // 전방 초음파 센서를 초기화합니다.
NewPing sonar_left(LTRIGGER_PIN, LECHO_PIN, MAX_DISTANCE); // 좌측 초음파 센서를 초기화합니다.
NewPing sonar_right(RTRIGGER_PIN, RECHO_PIN, MAX_DISTANCE); // 우측 초음파 센서를 초기화합니다.

float front_sonar = 0.0; // 전방 초음파 센서 값을 저장할 변수를 선언합니다.
float left_sonar = 0.0; // 좌측 초음파 센서 값을 저장할 변수를 선언합니다.
float right_sonar = 0.0; // 우측 초음파 센서 값을 저장할 변수를 선언합니다.

#define ENR 6 // 오른쪽 모터의 PWM 핀을 정의합니다.
#define IN1 11 // 오른쪽 모터의 제어 핀 1을 정의합니다.
#define IN2 10 // 오른쪽 모터의 제어 핀 2를 정의합니다.
#define IN3 9 // 왼쪽 모터의 제어 핀 1을 정의합니다.
#define IN4 8 // 왼쪽 모터의 제어 핀 2를 정의합니다.
#define ENL 7 // 왼쪽 모터의 PWM 핀을 정의합니다.

int maze_status = 0; // 미로의 상태를 저장할 변수를 선언합니다.

void setup() 
{
  pinMode(ENR, OUTPUT); // 오른쪽 모터 PWM 핀을 출력 모드로 설정합니다.
  pinMode(IN1, OUTPUT); // 오른쪽 모터 제어 핀 1을 출력 모드로 설정합니다.
  pinMode(IN2, OUTPUT); // 오른쪽 모터 제어 핀 2를 출력 모드로 설정합니다.
  pinMode(IN3, OUTPUT); // 왼쪽 모터 제어 핀 1을 출력 모드로 설정합니다.
  pinMode(IN4, OUTPUT); // 왼쪽 모터 제어 핀 2를 출력 모드로 설정합니다.
  pinMode(ENL, OUTPUT); // 왼쪽 모터 PWM 핀을 출력 모드로 설정합니다.
  Serial.begin(115200); // 시리얼 통신을 115200bps로 시작합니다.
}

void motor_A_control(int direction_a, int motor_speed_a) 
{
  if(direction_a == HIGH)
  {
    digitalWrite(IN1, LOW); // IN1 핀을 LOW로 설정합니다.
    digitalWrite(IN2, HIGH); // IN2 핀을 HIGH로 설정합니다.
    analogWrite(ENR, motor_speed_a); // ENR 핀에 motor_speed_a 값을 아날로그 신호로 출력합니다.
  }
  else
  {
    digitalWrite(IN1, HIGH); // IN1 핀을 HIGH로 설정합니다.
    digitalWrite(IN2, LOW); // IN2 핀을 LOW로 설정합니다.
    analogWrite(ENR, motor_speed_a); // ENR 핀에 motor_speed_a 값을 아날로그 신호로 출력합니다.
  }
}

void motor_B_control(int direction_b, int motor_speed_b) 
{
  if(direction_b == HIGH)
  {
    digitalWrite(IN3, LOW); // IN3 핀을 LOW로 설정합니다.
    digitalWrite(IN4, HIGH); // IN4 핀을 HIGH로 설정합니다.
    analogWrite(ENL, motor_speed_b); // ENL 핀에 motor_speed_b 값을 아날로그 신호로 출력합니다.
  }
  else
  {
    digitalWrite(IN3, HIGH); // IN3 핀을 HIGH로 설정합니다.
    digitalWrite(IN4, LOW); // IN4 핀을 LOW로 설정합니다.
    analogWrite(ENL, motor_speed_b); // ENL 핀에 motor_speed_b 값을 아날로그 신호로 출력합니다.
  }
}

void check_maze_status(void)
{
  // 벽과의 거리에 따라 미로의 상태를 판단합니다.
  if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 4; // 전방, 좌측, 우측 모두 벽이 있을 때 상태를 4로 설정합니다.
    Serial.println("maze_status = 4");
  }
  else if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 1; // 전방이 열려있고 좌우측에 벽이 있을 때 상태를 1로 설정합니다.
    Serial.println("maze_status = 1");
  }
  else if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 2; // 좌측과 전방에 벽이 있을 때 상태를 2로 설정합니다.
    Serial.println("maze_status = 2");
  }
  else if((right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 3; // 우측과 전방에 벽이 있을 때 상태를 3으로 설정합니다.
    Serial.println("maze_status = 3");
  }
  else
  {
    maze_status = 0; // 다른 경우에는 상태를 0으로 설정합니다.
    Serial.println("maze_status = 0");
  }
}

void wall_collision_avoid(int base_speed)
{
  float error = 0.0; // 좌우 벽 사이의 차이를 저장할 변수를 선언합니다.
  float Kp = 0.2; // P 제어기의 비례 이득 값을 설정합니다.
  int right_pwm = 0; // 오른쪽 모터의 PWM 값을 저장할 변수를 선언합니다.
  int left_pwm = 0; // 왼쪽 모터의 PWM 값을 저장할 변수를 선언합니다.
  error = (right_sonar - left_sonar); // 좌우 벽 간격의 차이를 계산합니다.
  error = Kp * error; // 제어 이득을 곱합니다.

  if(error >= 50) error = 50; // 에러 값을 최대 50으로 제한합니다.
  if(error <= -50) error = -50; // 에러 값을 최소 -50으로 제한합니다.

  right_pwm = base_speed - error; // 오른쪽 모터 속도를 조정합니다.
  left_pwm = base_speed + error; // 왼쪽 모터 속도를 조정합니다.

  if(left_pwm <= 0) left_pwm = 0; // 왼쪽 모터 PWM 값을 0 이상으로 제한합니다.
  if(right_pwm <= 0) right_pwm = 0; // 오른쪽 모터 PWM 값을 0 이상으로 제한합니다.

  if(left_pwm >= 255) left_pwm = 120; // 왼쪽 모터 PWM 값을 120으로 제한합니다.
  if(right_pwm >= 255) right_pwm = 120; // 오른쪽 모터 PWM 값을 120으로 제한합니다.

  motor_A_control(HIGH, left_pwm); // 왼쪽 모터를 전진 방향으로 제어합니다.
  motor_B_control(HIGH, right_pwm); // 오른쪽 모터를 전진 방향으로 제어합니다.
}

void loop() 
{
  front_sonar = sonar_front.ping_cm() * 10; // 전방 초음파 센서 값을 mm로 변환하여 저장합니다.
  left_sonar = sonar_left.ping_cm() * 10; // 좌측 초음파 센서 값을 mm로 변환하여 저장합니다.
  right_sonar = sonar_right.ping_cm() * 10; // 우측 초음파 센서 값을 mm로 변환하여 저장합니다.

  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE * 10; // 전방 센서 값이 0
