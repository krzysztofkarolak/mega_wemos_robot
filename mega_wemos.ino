/*
 * Pins
 * D7 - WS LED
 * D8 - ECHO SR04 2
 * D9 - TRIG SR04 2
 * D11 - BUZZER
 * D13 - LED ONBOARD
 * D18 - TX
 * D19 - RX
 * D36 - Servo1 (podwozie)
 * D37 - Servo2
 * D6 PWM - MOTORS 2 SPEED
 * D5 PWM - MOTORS 1 SPEED
 * D48 - MOTORS 2 FORWARD
 * D49 - MOTORS 2 BACKWARD
 * D50 - MOTORS 1 FORWARD
 * D51 - MOTORS 1 BACKWARD
 * D52 - ECHO SR04 1
 * D53 - TRIG SR04 1
 * 
 * 
 * V62 - measure1 on/off 0-1
 * V63 - measure1 on/off 0-1
 * V64 - SERVO1 MOVE VIRTUAL - values 0-180
 * V65 - SERVO2 MOVE VIRTUAL - values 0-180
 * V66 - change measure buzzer moment - values 1-199
 * 
 */


//#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <SimpleTimer.h>
#include "Ai_WS2811.h"
#include <Servo.h>

Servo servo1;
Servo servo2;

#define WS_LED 7
Ai_WS2811 ws2811;
//RGB colors in hex
byte c000 = 0x00;
byte c255 = 0xff;

int buzzerPin = 11;
int maxDistanceBuzzer;
boolean measure1State = false;
boolean measure2State = false;

SimpleTimer sr04;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "24c1c3e1227d4b39acb0bda11143f26c";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Blynk-server";
char pass[] = "blynk1234";

// Hardware Serial on Mega, Leonardo, Micro...
#define EspSerial Serial1

#define ESP8266_BAUD 115200

ESP8266 wifi(&EspSerial);

long measureDistance(int trigPin, int echoPin) {
  long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  if (distance >= 200 || distance <= 0){
    digitalWrite(buzzerPin, LOW);
  }
  else if(distance<maxDistanceBuzzer) {
    digitalWrite(buzzerPin, HIGH);
  }
  else {
    digitalWrite(buzzerPin, LOW);
  }
  return distance;
}

void getsr04() {
  if(measure1State){
  Blynk.virtualWrite(V5, measureDistance(9,8));
  }
  if(measure2State) {
  Blynk.virtualWrite(V6, measureDistance(53,52));
  }
}

void sendLEDs()
{
  cli();
    ws2811.send();
  sei();
}

//Set color on led on virtual write
BLYNK_WRITE(V55) {
  ws2811.setColor(c000,c255,c255);
  sendLEDs();
}
BLYNK_WRITE(V56) {
  ws2811.setColor(c255,c255,c255);
  sendLEDs();
}
BLYNK_WRITE(V57) {
  ws2811.setColor(c000,c255,c000);
  sendLEDs();
}
BLYNK_WRITE(V58) {
  ws2811.setColor(c255,c255,c000);
  sendLEDs();
}
BLYNK_WRITE(V59) {
  ws2811.setColor(c255,c000,c000);
  sendLEDs();
}
BLYNK_WRITE(V60) {
  ws2811.setColor(c000,c000,c255);
  sendLEDs();
}
BLYNK_WRITE(V61) {
  ws2811.setColor(c000,c000,c000);
  sendLEDs();
}

//turn on/off distance measurement
BLYNK_WRITE(V62) {
      if (param.asInt()) {
        //HIGH
        measure1State = true;
      } else {
       //LOW
        measure1State = false;
    }
}
BLYNK_WRITE(V63) {
      if (param.asInt()) {
        //HIGH
        measure2State = true;
      } else {
       //LOW
        measure2State = false;
    }
}

//servos
BLYNK_WRITE(V64)
{
  servo1.write(param.asInt());
}
BLYNK_WRITE(V65)
{
  servo2.write(param.asInt());
}

//buzzer activate moment change
BLYNK_WRITE(V66)
{
  if(param.asInt() == 5) {
  maxDistanceBuzzer = 5;
  }
  else if(param.asInt() == 10) {
  maxDistanceBuzzer = 10;
  }
  else {
  maxDistanceBuzzer = 3;
  }
}

void setup()
{
  // Set console baud rate
  Serial.begin(115200);
  delay(10);
  // Set ESP8266 baud rate
  EspSerial.begin(ESP8266_BAUD);
  delay(10);
  pinMode(9, OUTPUT);
   pinMode(11, OUTPUT);
  pinMode(8, INPUT);
  pinMode(53, OUTPUT);
  pinMode(52, INPUT);
  pinMode(2, OUTPUT); //electromagnet


  //attach servos and set default position
  servo1.attach(36);
  servo1.write(0);
  servo2.attach(37);
  servo2.write(150);

  maxDistanceBuzzer = 3;

  //neopixel led init
  ws2811.init(WS_LED);

  Blynk.begin(auth, wifi, ssid, pass, "192.168.43.1");
  sr04.setInterval(900L, getsr04);

  analogWrite(6, 135);
  analogWrite(5, 135);
  digitalWrite(2, LOW);
}

void loop()
{
  Blynk.run();
  sr04.run();
}

