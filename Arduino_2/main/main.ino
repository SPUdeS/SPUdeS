

/* 
 * GRO 400 - Agile conception of a Stewarts platform
 * Version 1 of motor control through arduino with serial communication with raspberry pi
 * Auteurs: Alexis Nadeau
 * date: 25 janvier 2021
*/

/*------------------------------ Libraries ---------------------------------*/
#include <SoftTimer.h>
#include <ArduinoJson.h>
#include <Servo.h>
/*------------------------------ Constants ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

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


volatile bool shouldSend_ = false;    // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;    // drapeau prêt à lire un message

String error_;                        // message d'erreur pour le deverminage
SoftTimer timerSendMsg_;              // chronometre d'envoie de messages
SoftTimer timerPulse_;                // chronometre pour la duree d'un pulse
/*------------------------- Prototypes de fonctions -------------------------*/

void serialEvent();
void sendSerial();
void readSerial();
void endPulse();
void startPulse();


/*---------------------------- fonctions "Main" -----------------------------*/
void setup() {
  Serial.begin(BAUD);
  
  // Power Servo
  servoMotor_1.attach(5);
  servoMotor_2.attach(7);
  servoMotor_3.attach(3);
  servoMotor_4.attach(11);
  servoMotor_5.attach(9);
  servoMotor_6.attach(13);

  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(sendSerial);
  timerSendMsg_.enable();

  servoMotor_1.write(45);

}

void loop() {
  
  if(shouldRead_){
    readSerial();
    sendSerial();
  }
  
  timerSendMsg_.update();
}

/*---------------------------Definition de fonctions ------------------------*/

void moveServo(int servo_no, int  angle){
    servoMotors[servo_no].write(angle);
  }

void serialEvent(){
  shouldRead_=true;
}

//Lire message du RPI
void readSerial(){
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, Serial);
    if (err) {
      error_ = "erreur deserialisation.";
      return;
    }else{
      error_ = "";
    }
    int angle = doc["setGoal"][0];
    if(angle>=0){
      moveServo(0, angle);
      servoMotor_1.write(doc["setGoal"][0]);
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

//Envoi d'un message au RPI
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
  //doc["inPulse"] = isInPulse_;
  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
}
