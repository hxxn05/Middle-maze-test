#include <NewPing.h>

#define MAX_DISTANCE 150       // Maximum distance (in cm) to ping.
#define WALL_GAP_DISTANCE 400  // mm 단위
#define WALL_GAP_DISTANCE_HALF 230  // mm 단위
#define MOTOR_PWM_OFFSET 10

#define FTRIGGER_PIN 12
#define FECHO_PIN 13
#define LTRIGGER_PIN 16
#define LECHO_PIN 17
#define RTRIGGER_PIN 14
#define RECHO_PIN 15

NewPing sonar_front(FTRIGGER_PIN, FECHO_PIN, MAX_DISTANCE);
NewPing sonar_left(LTRIGGER_PIN, LECHO_PIN, MAX_DISTANCE); // 좌측 센서
NewPing sonar_right(RTRIGGER_PIN, RECHO_PIN, MAX_DISTANCE); // 우측 센서

float front_sonar = 0.0;
float left_sonar = 0.0;
float right_sonar = 0.0;

#define ENR 6
#define IN1 11
#define IN2 10
#define IN3 9
#define IN4 8
#define ENL 7

int maze_status = 0;

void setup() 
{
  pinMode(ENR, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENL, OUTPUT);
  Serial.begin(115200); // 통신속도를 115200으로 설정
}

void motor_A_control(int direction_a, int motor_speed_a) 
{
  if(direction_a == HIGH)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENR, motor_speed_a);
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENR, motor_speed_a);
  }
}

void motor_B_control(int direction_b, int motor_speed_b) 
{
  if(direction_b == HIGH)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENL, motor_speed_b);
  }
  else
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENL, motor_speed_b);
  }
}

void check_maze_status(void)
{
  if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 4;
    Serial.println("maze_status = 4");
  }
  else if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 1;
    Serial.println("maze_status = 1");
  }
  else if((left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 2;
    Serial.println("maze_status = 2");
  }
  else if((right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 3;
    Serial.println("maze_status = 3");
  }
  else
  {
    maze_status = 0;
    Serial.println("maze_status = 0");
  }
}

void wall_collision_avoid(int base_speed)
{
  float error = 0.0;
  float Kp = 0.2; // Value to be adjusted later (how much to turn)
  int right_pwm = 0;
  int left_pwm = 0;
  error = (right_sonar - left_sonar); // Change here to prioritize avoiding obstacles on the left
  error = Kp * error;

  if(error >= 50) error = 50;
  if(error <= -50) error = -50;

  right_pwm = base_speed - error; // Change here to prioritize avoiding obstacles on the left
  left_pwm = base_speed + error;

  if(left_pwm <= 0) left_pwm = 0;
  if(right_pwm <= 0) right_pwm = 0;

  if(left_pwm >= 255) left_pwm = 120; // Increase maximum speed
  if(right_pwm >= 255) right_pwm = 120; // Increase maximum speed

  motor_A_control(HIGH, left_pwm); // Left forward
  motor_B_control(HIGH, right_pwm); // Right forward
}

void loop() 
{
  front_sonar = sonar_front.ping_cm() * 10;
  left_sonar = sonar_left.ping_cm() * 10;
  right_sonar = sonar_right.ping_cm() * 10;

  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE * 10;
  if(left_sonar == 0.0) left_sonar = MAX_DISTANCE * 10;
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE * 10;

  Serial.print("L: "); Serial.print(left_sonar); Serial.print(" ");
  Serial.print("F: "); Serial.print(front_sonar); Serial.print(" ");
  Serial.print("R: "); Serial.println(right_sonar);

  check_maze_status();

  if(maze_status == 4)
  {
    // Stop
    Serial.println("Rotate CCW");
    motor_A_control(LOW, 0);
    motor_B_control(HIGH, 0);
    delay(100); // Delay for 1 second

    // Rotate 180 degrees
    motor_A_control(LOW, 250); // Left forward
    motor_B_control(HIGH, 255); // Right reverse
    delay(680); // Delay for 1 second

    // Stop
    Serial.println("Rotate CCW");
    motor_A_control(LOW, 0);
    motor_B_control(HIGH, 0);
    delay(100); // Delay for 1 second

    // Go straight
    Serial.println("Go Straight");
    motor_A_control(HIGH, 255); // Increase speed for straight run
    motor_B_control(HIGH, 255); // Increase speed for straight run
  }

  if(maze_status == 1)
  {
    // Go straight
    Serial.println("Run straight");
    wall_collision_avoid(255); // Increase speed for straight run
  } 
  
  else if(maze_status == 3)
  {
    // Stop
    Serial.println("Rotate CCW");
    motor_A_control(LOW, 0);
    motor_B_control(HIGH, 0);
    delay(100); // Delay for 1 second

    // Rotate 90 degrees to the left
    motor_A_control(LOW, 250); 
    motor_B_control(HIGH, 255); 
    delay(300); // Delay for 0.6 seconds

    // Stop
    motor_A_control(LOW, 0);
    motor_B_control(HIGH, 0);
    delay(100); // Delay for 1 second
  }
  
  else if(maze_status == 2)
  {
    // Stop
    Serial.println("Rotate CCW");
    motor_A_control(LOW, 0);
    motor_B_control(HIGH, 0);
    delay(100); // Delay for 1 second
    
    // Rotate 90 degrees to the right
    motor_A_control(HIGH, 250); 
    motor_B_control(LOW, 255); 
    delay(320); // Delay for 0.6 seconds

    // Stop
    motor_A_control(LOW, 0);
    motor_B_control(HIGH, 0);
    delay(100); // Delay for 1 second
  } 
  
  else
  {
    // Go straight
    Serial.println("Go Straight");
    motor_A_control(HIGH, 255); // Increase speed for straight run
    motor_B_control(HIGH, 255); // Increase speed for straight run
  }
}
