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
 * V5-6 measure distance
 * V7 - photoresistor value
 * V56-58 - WS2812 colors
 * V60 - on/off motors auto stop
 * V61 - on/off photoresistor measurements
 * V62 - measure1 on/off 0-1
 * V63 - measure1 on/off 0-1
 * V64 - SERVO1 MOVE VIRTUAL - values 0-180
 * V65 - SERVO2 MOVE VIRTUAL - values 0-180
 * V66 - change measure buzzer moment - values 1-199
 * V67 - change motors stop moment
 * 
 */


//#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <SimpleTimer.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>

#define LPIN 7
#define NUMLED 1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMLED, LPIN, NEO_GRB + NEO_KHZ800);
int lRed, lGreen, lBlue = 0;


Servo servo1;
Servo servo2;

int buzzerPin = 11;
int maxDistanceBuzzer, maxDistanceMotors, photoresVal, measure1Val, measure2Val;
boolean measure1State = false, measure2State = false;
boolean photoresState = true, photoColorToChange = true, motorstopState = false, motorsToStop = true;


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
  measure1Val = measureDistance(9,8);
  Blynk.virtualWrite(V5, measure1Val);
  }
  if(measure2State) {
  measure2Val = measureDistance(53,52);
  Blynk.virtualWrite(V6, measure2Val);
  }

  if(measure1State || measure2State) {
  //motors auto stop
  if(motorstopState) {
  if((measure1Val<maxDistanceMotors) || (measure2Val<maxDistanceMotors)) {
    if(motorsToStop) {
      digitalWrite(48, LOW);
      digitalWrite(49, LOW);
      digitalWrite(50, LOW);
      digitalWrite(51, LOW);
      motorsToStop = false;
    }
  }
  else {
    if(!motorsToStop) {
      motorsToStop = true;
    }
  }
  }//motorstopState
  }//measure1State||measure2State
  
  //photoresistor auto light
  if(photoresState) {
  photoresVal = analogRead(15);
  Blynk.virtualWrite(V7, photoresVal);
  if(photoresVal<25) {
    if(photoColorToChange) {
      pixels.setPixelColor(0,255, 255, 255);
      pixels.show();
      photoColorToChange=false;
    }
  }
  else {
    if(!photoColorToChange) {
      pixels.setPixelColor(0,0, 0, 0);
      pixels.show();
      photoColorToChange=true;
    }
  }
  } //photoresState
 
}

void setColorOnLed() {
  pixels.setPixelColor(0,lRed, lGreen, lBlue);
  pixels.show();
}

//Set color on led on virtual write
BLYNK_WRITE(V56) {
  lRed = param.asInt();
  setColorOnLed();
}
BLYNK_WRITE(V57) {
  lGreen = param.asInt();
  setColorOnLed();
}
BLYNK_WRITE(V58) {
  lBlue = param.asInt();
  setColorOnLed();

}

//turn on/off motors auto stop
BLYNK_WRITE(V60) {
   if(param.asInt()) {
     motorstopState = true;
   }
   else {
     motorstopState = false;
   }
}

//turn on/off light measurement
BLYNK_WRITE(V61) {
   if(param.asInt()) {
     photoresState = true;
   }
   else {
     photoresState = false;
   }
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
  maxDistanceBuzzer = param.asInt();
}
//motors stop moment change
BLYNK_WRITE(V67)
{
  maxDistanceMotors = param.asInt();
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

  maxDistanceBuzzer = 8;
  maxDistanceMotors = 38;

  //neopixel led init
  pixels.begin();
 

  Blynk.begin(auth, wifi, ssid, pass, "192.168.43.1");
  sr04.setInterval(400L, getsr04);

  pixels.setPixelColor(0,40, 40, 40); 
  pixels.show();
  delay(400);
  pixels.setPixelColor(0,0, 0, 0); 
  pixels.show();

  analogWrite(6, 135);
  analogWrite(5, 135);
  digitalWrite(2, LOW);
}

void loop()
{
  Blynk.run();
  sr04.run();
}

