/*************************************************************************************************************/
/*                                                                                                           */
/*                                                LIBRAIRIE                                                  */
/*                                                                                                           */
/*************************************************************************************************************/


/*******Librairie Server ********/
#include "ESP8266WiFi.h"       // for WIFI connexion
#include "FS.h"                // for mount file inside the flash
#include <ESP8266WebServer.h>  // for http communication
#include "stdlib.h"
#include <ESP8266HTTPClient.h>  // for http client server communication
#include <WiFiClient.h>         // for http client communication WIFI


/*******Librairie GPS ***************/
#include <TinyGPS++.h>       // for calculate the heading the distant between é Points GPS
#include <TimeLib.h>         // for the date and time
#include <SoftwareSerial.h>  // for the communication serial withe GPS module
#include <Ultrasonic.h>      // for calculate the distance between the obstacle and the USV

/*******Librairie compass***********/
#include <Wire.h>                // For the protocole I2C
#include <Adafruit_Sensor.h>     // For all sensor ardafruit
#include <Adafruit_HMC5883_U.h>  // for the MAGNETIC COMPAS


/*******Librairie IO Expand***********/
#include "PCF8574.h"


/*******Librairie Servo***********/
#include <Adafruit_PWMServoDriver.h>


/*******Librairie PID***********/
#include <PID_v1.h>

/*******Librairie ESP_now***********/
#include <espnow.h>
/*************************************************************************************************************/
/*                                                                                                           */
/*                                                VARIABLE                                                   */
/*                                                                                                           */
/*************************************************************************************************************/

String ipRas = "192.168.43.151";  // IP adress RASPRRY PI AT HOME must  change địa chỉ của web sever bên http://112.197.246.122/(78.114.13.194)  ("112.197.244.231";)
//String ipMes = "192.168.233.40";
String port = "8080";  // IP adress RASPRRY PI AT HOME địa chỉ phần host (192, 168, 233, 70) IP adress

/*********PWM Motor ************/
#define pinMotorR 2  //
#define pinMotorL 0
#define pinMotorT 12
#define pinDebug 0
int incomingAngle;
String speed_edit = "0";
/*********Constante Server**************/
const char* ssid = "3i";             //"Bla bla bla";
const char* password = "hoangphuc";  //"blablabla01"


/**Variable of route*****/
const int nbLatLng = 15;                               //Max number GPS points
double latLng[2][nbLatLng], latLng_temp[2][nbLatLng];  //Array 2 dimmension 2*nbLatLng (mang 2 chieu)
short i = 0;                                           //Lors de la récupération des Lati-Long (lấy)
short j = 0;                                           //Lors de l'utilisation des Lati-Long (sử dụng)
String Smessage;                                       // String read
double Fmessage;                                       //String read converted in float
String token;

String SLong;
double FLat;
String SDec;
String SEnt;
String SLat;
String SPoint;
int Latu = 0;
bool Lat = false;  //Variable pour ranger si les valeurs n'arrivent pas dans l'ordre (lưu trữ ko theo thứ tự)
bool Long = false;
double nextLong, nextLati;
double latHome, longHome;

/**Variable of WEB*****/

String httpPayload;
int httpCode;
WiFiClient clientEwen;
HTTPClient httpEwen;
String adresseHttp;






/******Variable of GPS*********/
#define time_offset 10800  // define a clock offset of 10800 seconds (3 hours) ==> UTC + 3
char Time[] = "00:00:00";
char Date[] = "00-00-2000";
byte last_second, Second, Minute, Hour, Day, Month;
int Year;
String Longitude = "106.626569";  //vi độ default 10.757347, 106.626569
String Latitude = "10.757347";    // kinh độ default
String alt, sat, fixage, course1;
String speed1 = "0";
double LongitudeDouble, LatitudeDouble;

/********Variable Calculate GPS***************/
unsigned long distanceToNextPoint;
double courseToNextPoint;
short courseTo180;
short cap180;
short cap180_setpoint;

/***********Variable Boussole**************/
float actualBoussole = 0;


/************Variable motor**************/
int speed_set = 0;  //toc do se thay doi tuy thuoc vao khaong cach
bool babord = true, tribord = true;
bool arretMoteur = false;

/***********Variable Ultrasonic sensor**************/
int distance;

/***********Variable General**************/
int state = 0;
int etatGPS = 1;

/**************Timer********************/
unsigned long timerDistance;  //Timer de 5 seconde avant la prochaine messure de distance;
unsigned long timerStop;
unsigned long timerGps;  //Timer lecture GPS toute les secondes
unsigned long timerDebug;
unsigned long timerPosition;  //Timer Envoie de la position toute les 2 secondes


/********Variable Servo***************/
int pwm_temp = 90;
#define SERVOMIN 80   // Minimum value
#define SERVOMAX 600  // Maximum value#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define SER0 7        //Servo Motor 0 on connector 0
#define SER1 8        //Servo Motor 1 on connector 12
String round1;
int pwm0;
int pwm1;
int motorA = 0;  //pin on board
int motorB = 1;  //pin on board
int posDegrees;
int state_avodance = 0;
int distance_1, distance_2, distance_3, distance_4;
int rever_servo = 0;


/******Variable PID*********/
double originalSetpoint = 175.3;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

double Kp = 7;
double Kd = 0;
double Ki = 30;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);



void printIncomingReadings() {
  Serial.println("INCOMING READINGS");
  Serial.print("Distace1: ");
  Serial.println(distance_1);
  Serial.print("Distace2: ");
  Serial.println(distance_2);
  Serial.print("Distace3: ");
  Serial.println(distance_3);
  Serial.print("Distace4: ");
  Serial.println(distance_4);
  Serial.print("Angle: ");
  Serial.println(incomingAngle);
}

/*************************************************************************************************************/
/*                                                                                                           */
/*                                                  SETUP                                                    */
/*                                                                                                           */
/*************************************************************************************************************/

/*************Setup GPS**************************/
TinyGPSPlus gps;                   // The TinyGPS++ object
SoftwareSerial gpsSerial(13, 15);  // configuration gpsSerial library (RX pin, TX pin)

/*************Setup Compass*********************/
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);  // object mag for the magnetic compas

/*************Setup sever**********************/
ESP8266WebServer server(80);  //Configuration du serveur sur le port 80 de l'ESP


/************Setup Wifi**************************/
IPAddress local_IP(192, 168, 233, 70);  //USV IP cổng lan
IPAddress gateway(192, 168, 233, 56);   // router
IPAddress subnet(255, 255, 255, 0);     //
WiFiClient client;
HTTPClient http;

/************Setup IO expand**************************/
PCF8574 pcf8574(0x38);
PCF8574 pcf_MUI(0x20);

/************Setup Driver servo**************************/
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

// const char* serverName = "http://112.197.244.231:8000/GPS-post.php";  //http://112.197.244.231/post-esp-data.php/ip phai lay la ip4 cua may tinh hoac may chu
// const char* serverName_get = "http://112.197.244.231:8000/getdata.php/";

/******Variable ESP_now*********/

uint8_t broadcastAddress[] = { 0x3C, 0xE9, 0x0E, 0xAD, 0xA9, 0xBC };  //MAC:3C:E9:0E:AD:BB:64
int val;
String success;
typedef struct struct_message {
  // int id;
  // int cap180_sent;
  int angle;
  int distance1;
  int distance2;
  int distance3;
  int distance4;
} struct_message;
int BOARD_ID = 1;
// Create a struct_message called DHTReadings to hold sensor readings
struct_message myData;
struct_message incomingReadings;

// Create a struct_message to hold incoming sensor readings
void OnDataSent(uint8_t* mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.println("Delivery success");
  } else {
    Serial.println("Delivery fail");
  }
}
void OnDataRecv(uint8_t* mac_addr, uint8_t* incomingData, uint8_t len) {
  // Copies the sender mac address to a string
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  distance_1 = incomingReadings.distance1;
  distance_2 = incomingReadings.distance2;
  distance_3 = incomingReadings.distance3;
  distance_4 = incomingReadings.distance4;
  incomingAngle = incomingReadings.angle;
}

int timerPosition1=0;
void setup() {
  Serial.begin(115200);
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-180, 180);
  gpsSerial.begin(9600);
  analogWriteFreq(50);
  pinMode(pinMotorL, OUTPUT);
  pinMode(pinMotorR, OUTPUT);
  pca9685.begin();
  pca9685.setPWMFreq(50);
  if (!mag.begin())  // initiate the magnetic compas
  {

    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1)
      ;
  }

  Serial.print("Init pcf8574...");
  if (pcf8574.begin()) {
    Serial.println("OK");
  } else {
    Serial.println("KO");
  }
  pcf8574.pinMode(P0, OUTPUT);
  pcf8574.pinMode(P1, OUTPUT);
  pcf8574.pinMode(P2, OUTPUT);
  pcf8574.pinMode(P3, OUTPUT);
  pcf8574.pinMode(P4, OUTPUT);
  pcf8574.pinMode(P5, OUTPUT);
  pcf8574.pinMode(P6, OUTPUT);
  pcf8574.pinMode(P7, OUTPUT);
  pcf_MUI.pinMode(P0, OUTPUT);
  pcf_MUI.pinMode(P1, OUTPUT);



  // Set ESP-NOW
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Setting as a Wi-Fi Station..");
    analogWrite(pinMotorL, 70);
    analogWrite(pinMotorR, 10);
  }
  Serial.print("Station IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  //esp_now_register_send_cb(OnDataSent);

  // Register peer
  //esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_CONTROLLER, 1, NULL, 0);

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);




  // WiFi.begin(ssid, password);
  // Serial.println("Connecting");
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println("");
  // Serial.print("Connected to WiFi network with IP Address: ");
  analogWrite(pinMotorL, 70);
  analogWrite(pinMotorR, 10);
  // Serial.println(WiFi.localIP());
}

void loop() {

  if (timerPosition + 2000 < millis()) {
    timerPosition = millis();  //adresseHttp = "http://" + ipRas + ":" + port + "/api/esp/post-coor?lat="+ lat + "&lng=" +lng;
    adresseHttp = "http://" + ipRas + ":" + port + "/api/esp/post-coor?lat=" + Latitude + "&lng=" + Longitude + "&distance_1=" + distance_1 + "&distance_2=" + distance_2 + "&distance_3=" + distance_3 + "&distance_4=" + distance_4 + "&cap180=" + cap180 + "&distance=" + distanceToNextPoint + "&speed=" + speed_set + "&j=" + j;
    Serial.println(adresseHttp);
    http.begin(client, adresseHttp);
    httpCode = http.POST("");
    // http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    // String httpRequestData = "api_key=" + apiKeyValue + "&value1=" + Latitude
    //                          + "&value2=" + Longitude;
    // Serial.print("httpRequestData: ");
    // Serial.println(httpRequestData);
    // int httpResponseCode = http.POST(httpRequestData);
    // Serial.print("httpResponseCode: ");
    // Serial.println(httpResponseCode);
    // httpCode = http.GET();

    if (httpCode > 0) {
      httpPayload = http.getString();  //String  (LAT_Point1*LONG_Point1/LAT_Point2*LONG_P2/LAT_P3*LONG...) ex :48.409897*-4.487719/48.408658, -4.486261
      Serial.println(httpPayload);
      if (httpPayload == "0") {
        state = 6;
        analogWrite(pinMotorL, 70);
        analogWrite(pinMotorR, 10);
        Serial.println("Stop - Comeback home");
      } else {
        state = 0;
        Serial.println("Keep going");
      }
    }
    // if (timerPosition1 + 1000 < millis()) {
    //   timerPosition1=millis();
    //   adresseHttp = "http://" + ipRas + ":" + port + "/api/esp/round";  // http://192.168.0.2/src/gps/get_gps.php?cordvit=2.293399*48.858860*0*1
    //   Serial.println(adresseHttp);
    //   http.begin(client, adresseHttp);
    //   httpCode = http.GET();
    //   if (httpCode > 0) {
    //     httpPayload = http.getString();

    //     // speed_edit=getValue(httpPayload, '/', 1);
    //     Serial.print("round: ");
    //     Serial.println(httpPayload);
    //     // Serial.print("speed edit: ");
    //     // Serial.println(speed_edit);
    //   }
    //   http.end();
    // }
    http.end();
  }


    switch (state) {
      case 0:  //recieve data from database
        if (timerStop + 5000 < millis()) {
          timerStop = millis();
          adresseHttp = "http://" + ipRas + ":" + port + "/api/esp/get-all-coor?id=CO";  // http://192.168.0.2/src/gps/get_gps.php?cordvit=2.293399*48.858860*0*1
          Serial.println(adresseHttp);
          http.begin(client, adresseHttp);
          httpCode = http.GET();
          if (httpCode > 0) {
            httpPayload = http.getString();  //String  (LAT_Point1*LONG_Point1/LAT_Point2*LONG_P2/LAT_P3*LONG...) ex :48.409897*-4.487719/48.408658, -4.486261
            Serial.println(httpPayload);
            for (int i = 0; i <= 14; i++) {
              SPoint = getValue(httpPayload, '/', i);  //get cordonate GPS points without char '/' (ex: 48.409897*-4.487719 when i = 0, then 48.408658, -4.486261 when i = 1)
              //converter SPoint in float with 7 decimal
              for (int k = 0; k <= 1; k++) {
                SLat = getValue(SPoint, '*', k);
                SEnt = getValue(SLat, '.', 0);
                SDec = getValue(SLat, '.', 1);
                SLat = SEnt;
                for (int j = 0; j <= 13; j++) {
                  SLat = SLat + SDec.charAt(j);
                }
                FLat = SLat.toDouble();
                FLat = FLat / 100000000000000;
                latLng[k][i] = FLat;
                delay(20);
              }
            }
            displayArray();
            http.end();
            state = 1;
          }
        }
        break;

      case 1:
        if (timerDistance + 500 < millis())  //every  0,5 secondes for the heading correction
        {
          timerDistance = millis();
          nextLati = latLng[0][j];
          nextLong = latLng[1][j];
          //Serial.print("J: ");
          //Serial.println(j);
          Serial.println("State: ");
          Serial.println(state);
          if (nextLati == 0 || nextLong == 0)  // end of the route
          {
            if (round1.toInt() == 1) {
              state = 4;
            } else {
              state = 0;
              adresseHttp = "http://" + ipRas + ":" + port + "/api/esp/delete-coor";  // http://192.168.0.2/src/gps/get_gps.php?cordvit=2.293399*48.858860*0*1
              http.begin(client, adresseHttp);
              httpCode = http.POST("");
              Serial.println(httpCode);
              j = 0;
            }
            //Serial.print("state: ");
            //Serial.println(state);

          } else {

            // if (distance_1 < 40 || distance_2 < 40 || distance_3 < 40 || distance_4 < 40) {
            //   Serial.println("");
            //   Serial.println("---------------------------------------------OBTACLE--------------------------- ");
            //   Serial.println("");
            //   state = 5;  // state of advoing abstacle
            //   analogWrite(pinMotorR, 10);
            //   analogWrite(pinMotorL, 70);
            //   delay(2000);
            //   Serial.println("State: ");
            //   Serial.println(state);
            // }
            if (distanceToNextPoint <= 5) {
              Serial.println("Reach the point");
              j = j + 1;
              Serial.println("State: ");
              Serial.println(state);
            }
            calculOrderheadinging(magSensor());
            calculSpeed();
          }
        }
        break;

      case 4:  // comback
        double latLng_temp[2][14];
        for (int h = 0; h <= 1; h++) {
          for (int k = 0; k <= 13; k++) {
            latLng_temp[h][k] = 0;
          }
        }
        Serial.println("----------------latLng-----------------------------------------");
        for (int i = 0; i < 14; i++) {
          Serial.print((double)latLng[0][i], 8);
          Serial.print("\t");
          Serial.println((double)latLng[1][i], 8);
        }
        for (int h = 0; h <= 1; h++) {
          int kk = 0;
          for (int k = 14; k >= 0; k--) {
            if (latLng[h][k] != 0) {
              latLng_temp[h][kk] = latLng[h][k];
              kk = kk + 1;
            }
          }
        }
        Serial.println("------------------------latLng_temp----------------------------------");
        for (int i = 0; i < 14; i++) {
          Serial.print(latLng_temp[0][i], 8);
          Serial.print("\t");
          Serial.println(latLng_temp[1][i], 8);
        }
        Serial.println("--------------------------------------------------------------");
        for (int k = 0; k <= 14; k++) {
          for (int h = 0; h <= 1; h++) {
            latLng[h][k] = latLng_temp[h][k];
          }
        }
        state = 1;
        j = 0;
        break;
      case 5:
        Serial.println("Obtacle advoiding");
        obstacle_avodance();
        state_avodance = 0;
        break;
    }
    while (gpsSerial.available() > 0) {
      encodeGPS();
    }
  }
