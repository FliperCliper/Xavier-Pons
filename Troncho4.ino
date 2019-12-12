/////////Posició
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor1(0x68);
MPU6050 sensor2(0x69);

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

unsigned long tiempo_prev, tiempo_prev2, tHeart;
long prev = 0;

float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

float lastV;
const int motorPin1 = 6; /// Vibrador

////// While sensor blink





//////////// SPEAKER

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

/////////// PULSO







/////////////////////////////////////////////////////////////// SET UP
void setup() {

  ////////////////POSICIÓ

  Serial.begin(57600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C
  sensor1.initialize();    //Iniciando el sensor
  sensor2.initialize();    //Iniciando el sensor


  pinMode(motorPin1, OUTPUT);

  // if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  // else Serial.println("Error al iniciar el sensor");

  ////////////////BOTÓ

  //  pinMode(pinButton, INPUT); //set the button pin as INPUT

  //////////////// SPEAKER

  mySoftwareSerial.begin(9600);

  myDFPlayer.begin(mySoftwareSerial); //Use softwareSerial to communicate with mp3.
  delay(0); // Code to compatible with ESP8266 watch dog.


  Serial.println(F("DFPlayer Mini online."));

  //myDFPlayer.volume(20);  //Set volume value. From 0 to 30

  // myDFPlayer.play(1);  //Play the first mp3



}




////////////////////////////////////////////////////////////// LOOP
void loop() {

  // int stateButton = digitalRead(pinButton); //read the state of the button

  ///////////////

  analogWrite(motorPin1, 0); ////// VIBRADOR
  if (getTilt())
  {
    Serial.println("Inclinació detectada");
    analogWrite(motorPin1, 100); ////// VIBRADOR
    delay(10);


  }
  else if (analogRead(A2) >= 400)  /// Resisténcia textil +700 conectar directament a digital (ya ho esta llegint)
  {
    Serial.println("Touch detectat");
    myDFPlayer.volume(30);  //Set volume value. From 0 to 30
    myDFPlayer.play(1);  //Play the first mp3 REPRODUIR ARXIU TACTE
    delay(2150);


  } else if (getBlink())
  {

    Serial.println("Blink detected");
    myDFPlayer.volume(20);  //Set volume value. From 0 to 30
    myDFPlayer.play(2);  //Play the second mp3 REPRODUIR ARXIU  BLINK
    delay(560);



  }  
  else
  {
    if (millis() - tHeart > 500) {
      myDFPlayer.volume(30);  //Set volume value. From 0 to 30
      myDFPlayer.play(3);  //Play the first mp3 REPRODUIR ARXIU  PULSO
      tHeart = millis();
      Serial.println("Heartbeat detected");
    }
  }

  delay(10);



}

boolean getTilt() {

  sensor2.getAcceleration(&ax, &ay, &az);
  sensor2.getRotation(&gx, &gy, &gz);

  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();

  //Calcular los ángulos con acelerometro
  //float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);

  //Calcular angulo de rotación con giroscopio y filtro complemento
  //ang_x = 0.98 * (ang_x_prev + (gx / 131) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131) * dt) + 0.02 * accel_ang_y;

  //ang_x_prev = ang_x;
  ang_y_prev = ang_y;

  //Serial.println(ang_y);

  if (ang_y < 25 || ang_y > 35)
    return true;
  else
    return false;     //False!!!!
}


boolean getBlink() {

  sensor1.getAcceleration(&ax, &ay, &az);


  long val = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));

  dt = (millis() - tiempo_prev2);
  tiempo_prev2 = millis();

  //Serial.println(abs(val - prev) / (float)dt);

  if (abs(val - prev) / (float)dt > 300) { // 150
    prev = val;
    return true;
  }
  else
  {
    prev = val;
    return false;
  }

}
