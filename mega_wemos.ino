/*
 * Pins
 * D5(D50,51) , D6 (D48,49) - motors speed
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
 * V59 - on/off auto motors mode
 * V60 - on/off motors auto stop
 * V61 - on/off photoresistor measurements
 * V62 - measure1 on/off 0-1
 * V63 - measure1 on/off 0-1
 * V64 - SERVO1 MOVE VIRTUAL - values 0-180
 * V65 - SERVO2 MOVE VIRTUAL - values 0-180
 * V66 - change measure buzzer moment - values 1-199
 * V67 - change motors stop moment
 * V68 - on/off buzzer warning based on measurement
 * V69 - motors speed value
 * V70 - advanced turn on/off
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
int maxDistanceBuzzer, maxDistanceMotors, photoresVal, measure1Val, measure2Val, speedVal=140, speedToTurnVal, measureSamplesNum = 2;
boolean measure1State = false, measure2State = false;
boolean photoresState = true, photoColorToChange = true, motorstopState = false, motorsToStop = true, autoBuzzerState = false;
boolean d50State = false, d48State = false, d49State = false, d51State = false, d50TurnState = false, d48TurnState = false, d49TurnState = false, d51TurnState = false, advancedTurn = true, smoothState = true;


//autoride mode
boolean autoMoveState = false, autoMoveDirectionToChange = true;
int autoMoveDirection = 0;

SimpleTimer sr04;
SimpleTimer smoothTimer;

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

void autoMoveMode() {
  if(autoMoveDirection==0) {
    if(autoMoveDirectionToChange) {
      digitalWrite(50, HIGH);
      digitalWrite(48, HIGH); 
      digitalWrite(49, LOW);
      digitalWrite(51, LOW);
      autoMoveDirectionToChange = false;
    }
  }
  else if(autoMoveDirection==1) {
    if(autoMoveDirectionToChange) {
      digitalWrite(50, LOW);
      digitalWrite(48, LOW); 
      digitalWrite(49, HIGH);
      digitalWrite(51, HIGH);
      autoMoveDirectionToChange = false;
    }
  }
}

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
  else if((distance<maxDistanceBuzzer) && (autoBuzzerState)) {
    digitalWrite(buzzerPin, HIGH);
  }
  else {
    digitalWrite(buzzerPin, LOW);
  }
  return distance;
}

void getsr04() {
  if(measure1State){
  measure1Val=0;  
  for(int i=0;i<measureSamplesNum;i++){  
  measure1Val+= measureDistance(9,8);
  }
  measure1Val/=measureSamplesNum;
  Blynk.virtualWrite(V5, measure1Val);
  }
  if(measure2State) {
  measure2Val=0;  
  for(int i=0;i<measureSamplesNum;i++){  
  measure2Val+= measureDistance(53,52);
  }
  measure2Val/=measureSamplesNum;
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
      if(autoMoveState) {
        autoMoveDirection++;
        if(autoMoveDirection>1) {
          autoMoveDirection = 0;
        }
        autoMoveDirectionToChange = true;
      }
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

void setSpeedVal(int speedInVal) {
  analogWrite(5,speedInVal);
  analogWrite(6,speedInVal);
  if(speedInVal>=160) {
    speedToTurnVal=speedInVal/6;
  } else if(speedInVal<160) {
    speedToTurnVal=speedInVal/4;
  }
}

/*void smoothStart() {
  if(((d50State)&&(d48State))||((d49State)&&(d51State))) {
    if(!smoothStart) {
    smoothState = true;
    smoothSpeed = speedVal;
    speedVal /= 3;
    }
    else if(speedVal<smoothSpeed) {
      speedVal += smoothSpeed/3;
    }
    else {
      speedVal = smoothSpeed;
    }
  analogWrite(5,speedVal);
  analogWrite(6,speedVal);
  }
}
*/
void setColorOnLed() {
  pixels.setPixelColor(0,lRed, lGreen, lBlue);
  pixels.show();
}

//left forward
BLYNK_WRITE(V50) {
if(advancedTurn) {
  if(param.asInt()) {
    d50State = true;
    digitalWrite(50, HIGH);
     analogWrite(5, speedVal);
      
      if(!d48State) {
      analogWrite(6, speedToTurnVal);
      digitalWrite(48, HIGH);
      d48TurnState = true;
      }
      }
  else {
    d50State = false;
    if(d48State) {
      analogWrite(5, speedToTurnVal);
      digitalWrite(50, HIGH);
      d50TurnState = true;
    }
    else {
      digitalWrite(50, LOW);
      if(d48TurnState) { 
        digitalWrite(48, LOW);
        d48TurnState = false;
      }
  //    smoothState = false;
    }
  }
} else {
    digitalWrite(50, param.asInt());
}
}
//right forward
BLYNK_WRITE(V48) {
if(advancedTurn) {
  if(param.asInt()) {
    d48State = true;
      analogWrite(6, speedVal);
      digitalWrite(48, HIGH);
      if(!d50State) {
      analogWrite(5, speedToTurnVal);
      digitalWrite(50, HIGH);
      d50TurnState = true;
      }
      }
  else {
    d48State = false;
    if(d50State) {
      analogWrite(6, speedToTurnVal);
      digitalWrite(48, HIGH);
      d48TurnState = true;
    }
    else {
      digitalWrite(48, LOW);
        if(d50TurnState) { 
        digitalWrite(50, LOW);
        d50TurnState = false;
      }
    //  smoothState = false;
    }
  }
} else {
    digitalWrite(48, param.asInt());
}
}
//left forward
BLYNK_WRITE(V49) {
if(advancedTurn) {
  if(param.asInt()) {
    d49State = true;
      analogWrite(6, speedVal);
      digitalWrite(49, HIGH);
      if(!d51State) {
      analogWrite(5, speedToTurnVal);
      digitalWrite(51, HIGH);
      d51TurnState = true;
      }
      }
  else {
    d49State = false;
    if(d51State) {
      analogWrite(6, speedToTurnVal);
      digitalWrite(49, HIGH);
      d49TurnState = true;
    }
    else {
      digitalWrite(49, LOW);
      if(d51TurnState) { 
        digitalWrite(51, LOW);
        d51TurnState = false;
      }
    //  smoothState = false;
    }
  }
} else {
    digitalWrite(49, param.asInt());
}
}
//right forward
BLYNK_WRITE(V51) {
if(advancedTurn) {
  if(param.asInt()) {
    d51State = true;
      analogWrite(5, speedVal);
      digitalWrite(51, HIGH);
      if(!d49State) {
      analogWrite(6, speedToTurnVal);
      digitalWrite(49, HIGH);
      d49TurnState = true;
      }
      }
  else {
    d51State = false;
    if(d49State) {
      analogWrite(5, speedToTurnVal);
      digitalWrite(51, HIGH);
      d51TurnState = true;
    }
    else {
      digitalWrite(51, LOW);
      if(d49TurnState) { 
        digitalWrite(49, LOW);
        d49TurnState = false;
      }
    //  smoothState = false;
    }
  }
} else {
    digitalWrite(51, param.asInt());
}
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

//turn on/off auto motors mode
BLYNK_WRITE(V59) {
   if(param.asInt()) {
     motorstopState = true;
     autoMoveState = true;
   }
   else {
     autoMoveState = false;
   }

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
//auto buzzer setting
BLYNK_WRITE(V68)
{
        if (param.asInt()) {
        //HIGH
        autoBuzzerState = true;
        } else {
        //LOW
        autoBuzzerState = false;
        }
}
//motors speed
BLYNK_WRITE(V69) {
  speedVal=param.asInt();
  setSpeedVal(speedVal);
}
//alternative motors steering setting (boolean)
BLYNK_WRITE(V70)
{
  advancedTurn = param.asInt();
}
//how many tries to avg measurements (int)
BLYNK_WRITE(V71)
{
  measureSamplesNum = param.asInt();
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
 // smoothTimer.setInterval(500L, smoothStart);

  pixels.setPixelColor(0,40, 40, 40); 
  pixels.show();
  delay(400);
  pixels.setPixelColor(0,0, 0, 0); 
  pixels.show();

  setSpeedVal(speedVal);
  Blynk.virtualWrite(V70, 1); //enable advanced turn
  digitalWrite(2, LOW);
}

void loop()
{
  Blynk.run();
  sr04.run();
  //smoothTimer.run();
  if(autoMoveState) {
    autoMoveMode();
  }
}

