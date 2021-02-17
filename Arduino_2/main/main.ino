

/* 
 * GRO 400 - Agile conception of a Stewart's platform
 * Version 1 of motor control through arduino with serial communication with raspberry pi
 * Authors: Alexis Nadeau
 * dDate: January 25th 2021
*/

/*------------------------------ Libraries ---------------------------------*/
#include <SoftTimer.h>
#include <ArduinoJson.h>
#include <Servo.h>
/*------------------------------ Constants ---------------------------------*/

#define BAUD            115200      //Serial transmission frequency
#define UPDATE_PERIOD  100         // Period of sending general state (ms)

/*---------------------------- Global Variables  ---------------------------*/
int angle_ = 0;                        // Variable for angle

Servo servoMotor_1;
Servo servoMotor_2;
Servo servoMotor_3;
Servo servoMotor_4;
Servo servoMotor_5;
Servo servoMotor_6;

Servo servoMotors [] = {servoMotor_1, servoMotor_2, servoMotor_3,
                        servoMotor_4, servoMotor_5, servoMotor_6};


volatile bool shouldSend_ = false;    // Flags when ready to send a message
volatile bool shouldRead_ = false;    // Flags when ready to read a message

String error_;                        // Error message for debugging
SoftTimer timerSendMsg_;              // Message sending timer

/*------------------------- Function Prototype -------------------------*/

void serialEvent();
void sendSerial();
void readSerial();


/*---------------------------- Main function -----------------------------*/
void setup() {
  Serial.begin(BAUD);
  
  // Power Servo
  servoMotor_1.attach(5);
  servoMotor_2.attach(7);
  servoMotor_3.attach(3);
  servoMotor_4.attach(11);
  servoMotor_5.attach(9);
  servoMotor_6.attach(13);

  // Message sending timer
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(sendSerial);
  timerSendMsg_.enable();

  // Set servoMotors to home position
  servoMotor_1.write(45);

}

void loop() {
  
  if(shouldRead_){
    readSerial();
    sendSerial();
  }
  
  timerSendMsg_.update();
}

/*---------------------------Function definition ------------------------*/

void moveServo(int servo_no, int  angle){
    servoMotors[servo_no].write(angle);
  }

void serialEvent(){
  shouldRead_=true;
}

//Read message coming from RPI
void readSerial(){
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, Serial);
    if (err) {
      error_ = "deserialization error0.";
      return;
    }else{
      error_ = "";
    }
    int angle = doc["setGoal"][0];
    if(angle>=0){
      moveServo(0, angle);
      }
    angle = doc["setGoal"][1];
    if(angle>=0){
      moveServo(1, angle);
      }
    angle = doc["setGoal"][2];
    if(angle>=0){
      moveServo(2, angle);
      }
    angle = doc["setGoal"][3];
    if(angle>=0){
      moveServo(3, angle);
      }
    angle = doc["setGoal"][4];
    if(angle>=0){
      moveServo(4, angle);
      }
   angle = doc["setGoal"][5];
    if(angle>=0){
      moveServo(5, angle);
      }
    
    shouldRead_=false;
}

//Send a message to RPI
void sendSerial(){
  StaticJsonDocument<512> doc;
  // Construction du message a envoyer
  doc["time"] = millis();
  doc["angle servo_0"] = servoMotor_1.read();
  doc["angle servo_1"] = servoMotor_2.read();
  doc["angle servo_2"] = servoMotor_3.read();
  doc["angle servo_3"] = servoMotor_4.read();
  doc["angle servo_4"] = servoMotor_5.read();
  doc["angle servo_5"] = servoMotor_6.read();
  doc["error"] = error_;
  // Serialization
  serializeJson(doc, Serial);
  // Sending
  Serial.println();
}
