#include <Arduino.h>
#include <Servo.h>

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
Servo motor5;
Servo motor6;

void setup() {
  // put your setup code here, to run once:
  motor1.attach(5,544,2400);
  motor2.attach(7,544,2400);
  motor3.attach(3,544,2400);
  motor4.attach(11,544,2400);
  motor5.attach(9,544,2400);
  motor6.attach(13,544,2400);
}

void loop() {
  // put your main code here, to run repeatedly:
  motor1.write(90);
  _delay_ms(1000);
  motor1.write(0);
  _delay_ms(1000);
  //motor2.write(90);
  //_delay_ms(1000);
  //motor2.write(0);
  //_delay_ms(1000);
  //motor3.write(90);
  //_delay_ms(1000);
  //motor3.write(0);
  //_delay_ms(1000);
  //motor4.write(90);
  //_delay_ms(1000);
  //motor4.write(0);
  //_delay_ms(1000);
  //motor5.write(90);
  //_delay_ms(1000);
  //motor5.write(0);
  //_delay_ms(1000);
  //motor6.write(90);
  //_delay_ms(1000);
  //motor6.write(0);
  //_delay_ms(1000);
}