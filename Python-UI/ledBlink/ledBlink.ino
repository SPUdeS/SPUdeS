int led = 13;

void setup(){
  Serial.begin(9600);
}

void loop(){
  if (Serial.available() == '98'){
    digitalWrite(led, HIGH);
    delay(1000);
    digitalWrite(led, LOW);
    delay(1000);
  }
}
