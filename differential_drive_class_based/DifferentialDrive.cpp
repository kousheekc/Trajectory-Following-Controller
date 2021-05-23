#include <Arduino.h>

#include "DifferentialDrive.h"

DifferentialDrive robot(0.126, 0.035);

DifferentialDrive::DifferentialDrive(float b, float w){
  Serial.begin(2400);
  body_radius = b;
  wheel_radius = w;

  en_pin = 8;

  left_wheel.dir_pin = 6;
  left_wheel.step_pin = 3;
  left_wheel.step_state = false;

  right_wheel.dir_pin = 5;
  right_wheel.step_pin = 2;
  right_wheel.step_state = false;

  pinMode(en_pin, OUTPUT);
  
  pinMode(left_wheel.dir_pin, OUTPUT);
  pinMode(left_wheel.step_pin, OUTPUT);
  pinMode(right_wheel.dir_pin, OUTPUT);
  pinMode(right_wheel.step_pin, OUTPUT);

  digitalWrite(en_pin, LOW);

  c.linear = 0;
  c.angular = 0;
}

void DifferentialDrive::init_ISR(){
  //disable interrupts
  cli();

  //initialise registers TCCR0A and TCCR0B to 0
  TCCR0A = 0;
  TCCR0B = 0;

  //initialise counter value to 0
  TCNT0  = 0;

  //interrupt gets triggered when counter reaches value in OCR0A (some default value)
//  OCR0A = 50;

  // CTC mode
  TCCR0A |= (1 << WGM01);

  // 256 prescaler
  TCCR0B |= (1 << CS02);

  // enable timer compare interrupt
//  TIMSK0 |= (1 << OCIE0A);

  //initialise registers TCCR2A and TCCR2B to 0
  TCCR2A = 0;
  TCCR2B = 0;

  //initialise counter value to 0
  TCNT2  = 0;

  //interrupt gets triggered when counter reaches value in OCR2A (some default value)
//  OCR2A = 50;

  // CTC mode
  TCCR2A |= (1 << WGM21);

  // 256 prescaler
  TCCR2B |= (1 << CS22);
  TCCR2B |= (1 << CS21);

  // enable timer compare interrupt
//  TIMSK2 |= (1 << OCIE2A);

  // enable interrupts
  sei();
}

void DifferentialDrive::right_step(){
  digitalWrite(right_wheel.dir_pin, right_wheel.dir);
  right_wheel.step_state = !right_wheel.step_state;
  digitalWrite(right_wheel.step_pin, right_wheel.step_state);
  if (right_wheel.step_state == true){
    if (right_wheel.dir == true){
      right_wheel.steps += 1;
    }
    else{
      right_wheel.steps -= 1;
    }
  }
}

void DifferentialDrive::left_step(){
  digitalWrite(left_wheel.dir_pin, left_wheel.dir);
  left_wheel.step_state = !left_wheel.step_state;
  digitalWrite(left_wheel.step_pin, left_wheel.step_state);
  if (left_wheel.step_state == true){
    if (left_wheel.dir == true){
      left_wheel.steps += 1;
    }
    else{
      left_wheel.steps -= 1;
    }
  }
}

void DifferentialDrive::update_control_velocities(float linear, float angular){
  c.linear = linear;
  c.angular = angular;
}

void DifferentialDrive::update_wheel_velocities(){
  right_wheel.velocity = (c.linear + body_radius * c.angular)/wheel_radius;
  left_wheel.velocity = (c.linear - body_radius * c.angular)/wheel_radius;
}

void DifferentialDrive::udpate_intervals(){
  right_wheel.interval = (1000000 * 2 * PI)/(right_wheel.velocity * 6400);
  left_wheel.interval = (1000000 * 2 * PI)/(left_wheel.velocity * 6400);
}

void DifferentialDrive::update_compare_register_value(){
  // TODO: clamp compare register values
  right_wheel.compare_register_value = right_wheel.interval / 16;
  left_wheel.compare_register_value = left_wheel.interval / 16;

  Serial.println(right_wheel.compare_register_value);
  Serial.println(left_wheel.compare_register_value);
}

void DifferentialDrive::update_registers(){

  if (right_wheel.compare_register_value < 0){
    right_wheel.dir = false;
  }
  else{
    right_wheel.dir = true;
  }

  if (left_wheel.compare_register_value < 0){
    left_wheel.dir = false;
  }
  else{
    left_wheel.dir = true;
  }

  if (right_wheel.compare_register_value == 0){
    TIMSK2 &= ~(1 << OCIE2A);
  }
  else{
    TIMSK2 |= (1 << OCIE2A);
    OCR2A = abs(right_wheel.compare_register_value);
  }

  if (left_wheel.compare_register_value == 0){
    TIMSK0 &= ~(1 << OCIE0A);
  }
  else{
    TIMSK0 |= (1 << OCIE0A);
    OCR0A = abs(left_wheel.compare_register_value);
  }
}

void DifferentialDrive::control(float linear, float angular){
  update_control_velocities(linear, angular);
  update_wheel_velocities();
  udpate_intervals();
  update_compare_register_value();
  update_registers();
}

void DifferentialDrive::move_angle(float theta){
  int steps = (3200 * body_radius * theta)/(2 * PI * wheel_radius);
  float angular;
  // TODO: smooth acceleration
  if (steps < 0){
    angular = -0.4;
  }
  else{
    angular = 0.4;
  }
  while (abs(right_wheel.steps) <= abs(steps)){
    control(0, angular);
  }
  control(0, 0);
  current_theta += theta;
  right_wheel.steps = 0;
  left_wheel.steps = 0;
}

void DifferentialDrive::move_distance(float distance){
  int steps = (3200 * distance)/(2 * PI * wheel_radius);
  float linear;
  // TODO: smooth acceleration
  if (steps < 0){
    linear = -0.1;
  }
  else{
    linear = 0.1;
  }
  while (abs(right_wheel.steps) <= abs(steps)){
    control(linear, 0);
  }
  control(0, 0);
  current_x += distance * cos(current_theta);
  current_y += distance * sin(current_theta);
  right_wheel.steps = 0;
  left_wheel.steps = 0;
}

void DifferentialDrive::move_it(float x, float y, float theta){
  float delta_rot_1 = atan2(y - current_y, x - current_x) - current_theta;
  float delta_tran = sqrt(sq(x - current_x) + sq(y - current_y));
  float delta_rot_2 = theta - current_theta - delta_rot_1;

  Serial.println(delta_rot_1);
  Serial.println(delta_tran);
  Serial.println(delta_rot_2);
  Serial.println();

  move_angle(delta_rot_1);
  move_distance(delta_tran);
  move_angle(delta_rot_2);

  current_x = x;
  current_y = y;
  current_theta = theta;
}

ISR(TIMER0_COMPA_vect){
  robot.left_step();
}

ISR(TIMER2_COMPA_vect){
  robot.right_step();
}
