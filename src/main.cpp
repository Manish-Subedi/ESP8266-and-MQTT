/*
  Author  -   Manish Subedi
  Example code using DHT22 & HRCS04
  Publish the data to MQTT
*/
#include <Arduino.h>
#include <Wire.h>
#include <DHTesp.h>
//#include <HCSR04.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <NewPing.h>
#include <math.h>

//define pins for HCSR04
#define TRIG_PIN 12
#define ECHO_PIN 14
#define D_TIME 1000 //delay time 

//function prototypes
void cbMQTT(char* topic, byte* payload, unsigned int length);
bool reconnectMQTT(PubSubClient &cln, String &clnName);//, const char* user, const char* psk);

String hostName; //used in MQTT publish
String mySSID = "MANIS"; //wifi parameter
String myPSK = "123IscA!"; //wifi parameter

//instantiate object client
WiFiClient myMqtt;
PubSubClient PSC(myMqtt);

//const IPAddress BROKER_IP(109,230,236,5); //MQTT broker's address
const char* BROKER_URL {"test.mosquitto.org"};
const uint16_t BROKER_PORT = 1883; //MQTT default port 
//const char* MQTT_USER = "JonSnow";
//const char* MQTT_PWD = "knowsnothing";

long prevTime;

//instantiate an object for DHT22
DHTesp myDht;
float Tp = 0.0, Hm = 0.0, pubT = 0.0, pubH = 0.0;

/*
//instantiate an object for HCSR04
HCSR04 myHC(TRIG_PIN, ECHO_PIN, 20, 4000);
long duration;
int distance = 0;
*/

//instantiate an object for HCSR04
NewPing sonar(TRIG_PIN, ECHO_PIN, 300);
int distance = 0, pubD=0;

void setup(){
  hostName = "1111angel";
  //WiFi.hostName(hostName);
  //init serial communication
  Serial.begin(115200);

  //connect with DHT22
  myDht.setup(4, DHTesp::DHT22);
  Serial.println("\n\n");

  //setup pins for the Ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  //configure WIFI
  WiFi.begin(mySSID, myPSK);
  int i = 0;
  while ((WiFi.status() != WL_CONNECTED) && i < 20){
    Serial.printf("Connection Status: %d\n", WiFi.status());
    delay(500);
    i++;
  }
  if (WiFi.isConnected()) Serial.println("Connected. IP Address: " + WiFi.localIP().toString());
  else Serial.println("[WiFi] Wifi connection failed.");

  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);

  PSC.setServer(BROKER_URL, BROKER_PORT);
  PSC.setCallback(cbMQTT);
 
}

void loop(){
  //variables to store instant readings
  float tpNew, hmNew;
  int distNew;

  if (prevTime + D_TIME <= millis()){
    tpNew = myDht.getTemperature();
    hmNew = myDht.getHumidity();
    distNew = sonar.ping_cm();
    /*
    //clear the trig-pin
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2); 
    //set the trig-pin on HIGH for 10µs
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    //read the echo-pin and return travel time in µs
    duration = pulseIn(ECHO_PIN, HIGH);
    distNew = duration * 0.034/2;
    */

    if (tpNew != Tp || hmNew != Hm || distNew != distance){
      Serial.println("Temperature = "+ (String)tpNew + " C");
      Serial.println("Humidity = "+ (String)hmNew + " %");  
      Serial.println("Distance = "+ (String)distNew + " cm\n");
    }
    Tp = tpNew;
    Hm = hmNew;
    distance = distNew;
    prevTime = millis();

    if (reconnectMQTT(PSC, hostName)){
      if(abs(Tp-pubT) >= 0.5){
        pubT = Tp;
        String topic = hostName + "/temp_celcius";
        String data_temp = (String)Tp;
        PSC.publish(topic.c_str(), data_temp.c_str());
      }
      if(abs(Hm-pubH) >= 0.5){
        pubH = Hm;
        String topic = hostName + "/humidity";
        String data_humid = (String)Hm;
        PSC.publish(topic.c_str(), data_humid.c_str());
      }
      if(abs(pubD-distance)>= 1){
        pubD = distance;
        String topic = hostName + "/distance";
        String data_distance = (String)distance;
        PSC.publish(topic.c_str(), data_distance.c_str());
      }
    }
    PSC.loop();
  }
  
}

//callback function 
void cbMQTT(char* topic, byte* payload, unsigned int length){
  String t = (String)topic, p;
  //save payload as string chars
  for (unsigned int i = 0; i < length; i++){
    p += (char)payload[i];
  }
  Serial.println("[cbMQTT] Message: " + t + " = " + p + " Length = " + (String)length);

}

//connect and reconnect 
bool reconnectMQTT(PubSubClient &cln, String &clnName){//, const char* user, const char* psk){
  //check wifi connection
  if(!WiFi.isConnected()) return false;
  //check broker connection
  int i = 0;
  while(!cln.connected() && i < 20){
    if(!cln.connect(clnName.c_str())){
      Serial.println("[reconnect] Broker connection failed! " + (String)cln.state());
      i++;
    }
    else {
      Serial.println("[reconnect] Broker connection established!");
      //String s = "angelina/temp/value";
      //cln.subscribe()
    }
  }
  return cln.connected(); //true or false
}









