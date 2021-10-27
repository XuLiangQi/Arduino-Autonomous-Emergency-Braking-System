#include <Servo.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_MQTT_FONA.h>
#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include <WiFiEspServer.h>
#include <WiFiEspUdp.h>
#include<Wire.h>

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME  // user name
#define AIO_KEY       //key

int currentSpeed = 60; //Speed for testing purpose
int Speed = 0; // initialize the speed to 0
char ssid[]=  // your wifi
char pass[] = // password to your wifi
int status = WL_IDLE_STATUS;
WiFiEspClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT,
AIO_USERNAME, AIO_KEY);
//const char SLIDER_FEED[] PROGMEM=AIO_USERNAME "/feeds/Velocity";
Adafruit_MQTT_Publish input1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Distance"); // publish distance value to adafruit
Adafruit_MQTT_Publish input2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Brake Indicator"); // publish the decision of brake indicator to adafruit
Adafruit_MQTT_Subscribe SLIDER_FEED = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Velocity"); // subscribe the velocity from a slider on adafruit
Adafruit_MQTT_Subscribe slider = Adafruit_MQTT_Subscribe(SLIDER_FEED);

//setting up variables
int trigPin = 12; // triger at pin 12
int echoPin = 11; // echo at pin 11
int pingTravelTime;
float acceleration, velocity, timeVar, realDuration, distance, distanceScaled, sumDistance, avgDistance, decision;
int v;
void MQTT_connect();

void setup() {
  // put your setup code here, to run once:
  //Initialize serial for debugging
  Serial.begin(9600);
  //Initialize serial for ESP module
  Serial1.begin(115200);
  WiFi.init(&Serial1);
  if (WiFi.status() == WL_NO_SHIELD) {
  Serial.println("WiFi shield not present");
  // don't continue
  while (true);
 }
 if( status != WL_CONNECTED) {
 Serial.print("Attempting to connect to WPA SSID: ");
 Serial.println(ssid);
 // Connect to WPA/WPA2 network
 // status =

 WiFi.begin(ssid, pass);
 while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
 Serial.println("Connected ");
 Serial.print(WiFi.localIP());
}

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  mqtt.subscribe(&slider);
}

void loop() {
  // acceleration reading
  sumDistance = 0;
  for(int i=0;i<10;i++){ //loops the program 10 times in order to calculate the aververage to provide a more accurate value
    digitalWrite(trigPin, LOW); // make sure that the sensor starts of in a low state
    delay(5); // delay for 5 microseconds before activating the trigger
    digitalWrite(trigPin, HIGH);// make sure that the sensorrecieves a high signal to start the recording sequence
    delay(10); // keep the trigger activated for 10 microseconds 
    digitalWrite(trigPin, LOW); //turn off the trigger
  
    pingTravelTime = pulseIn(echoPin, HIGH); // store the time for the wave to be sent and received
  
    realDuration = pingTravelTime/2; // have the time is required to find the correct distance
  
    distance = (realDuration)*0.034; // find the distance from the duration and the spped of sound (343m/s)
      
    sumDistance += distance;  //summs the distance for 10 times in the loop
    
    if (i==9){ // once the program has looped 10 times it will average the data
        avgDistance = sumDistance/10; // averageing the 10 distances
    }
  }
  distanceScaled = avgDistance * 10;
  Serial.println("Before MQTT");
  MQTT_connect();  
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(2000))) {
    // check if its the slider feed
    if (subscription == &slider) {
      Serial.print(F("Slider: "));
      Serial.println((char *)slider.lastread);
      uint16_t sliderval = atoi((char *)slider.lastread);  // convert to a number
      Speed = sliderval;
    }
  }
  Serial.println("After MQTT");
    
  Serial.println(Speed);
  velocity = Speed * 0.278; // convert the ultrasonic readings to unit of cm
  timeVar = distanceScaled / velocity; // calculate the time
  acceleration = velocity / timeVar; // calculate the acceleration based off the velocity
  acceleration = acceleration / 9.81;  // conver the unit of the acceleration to "g"
  Serial.println(distanceScaled);

  if (acceleration >= 0.7) {
    decision = 3; // 3 is stop, 2 is reduce the speed, 1 is go
  } else if (acceleration >= 0.4 && acceleration < 0.7){
    decision = 2;
  } else {
    decision = 1;
  }
  
  Serial.println(decision);
    
  Serial.print(F("\nSending values "));
  if (! input1.publish(distanceScaled)) {
  Serial.println(F("Failed"));
  } else {
  Serial.println(F("OK!"));
  }
  if (! input2.publish(decision)) {
  Serial.println(F("Failed"));
  } else {
  Serial.println(F("OK!"));
  }
  delay(5000);
  }

void MQTT_connect() {
 int8_t ret;
 // Stop if already connected.
 if (mqtt.connected()) {
 return;
 }
 Serial.print("Connecting to MQTT... ");
 uint8_t retries = 3;
 while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
 Serial.println(mqtt.connectErrorString(ret));
 Serial.println("Retrying MQTT connection in 5 seconds...");
 mqtt.disconnect();
 delay(5000); // wait 5 seconds
 retries--;
 mqtt.connect();
 if (retries == 0) {
 // basically die and wait for WDT to reset me
 break;
 //while (1);
 }
 }
 Serial.println("MQTT Connected!");
}
