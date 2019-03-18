//Vcc PIN1 (5V)
//Ground PIN2 (same ground as NodeMCU)
//Tx PIN5 ==> GPI010

// On Leonardo/Micro or others with hardware serial, use those!
// uncomment this line:
// #define pmsSerial Serial1
 
// For UNO and others without hardware serial, we must use software serial...
// pin #2 is IN from sensor (TX pin on sensor), leave pin #3 disconnected
// comment these two lines if using hardware serial
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include <DHT.h>
SoftwareSerial pmsSerial(2, 3);

const char* ssid = "YOUR WiFi SSID";
const char* password = "YOUR PASSWORD";
const char* mqtt_server = "broker.mqttdashboard.com";

const char* pubTopicT = "nodeREDmanisa/sensorC/temperature";
const char* pubTopicH ="nodeREDmanisa/sensorC/humidity";
const char* pubTopicPM ="nodeREDmanisa/sensorC/PM25";
//const char* subTopicLED = "nodeREDmanisa/sensorC/LED";
WiFiClient espClient;
PubSubClient client(espClient);
#define DHTPIN 5     
#define DHTTYPE DHT11   
DHT dht(DHTPIN, DHTTYPE);
//const int lamp = 4;

void setup() {
  Serial.begin(115200);
  dht.begin();
  pmsSerial.begin(9600);
  pinMode (lamp, OUTPUT);
  Serial.println("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
     {
            delay(250);
            Serial.print(".");
     }
  Serial.println("");
  Serial.println("WiFi connected");
  client.setServer(mqtt_server, 1883);
//  client.setCallback(callback);
}


//void callback(String topic, byte* message, unsigned int length) {
//  Serial.print("Message arrived on topic: ");
//  Serial.print(topic);
//  Serial.print(". Message: ");
//  String messageTemp;
  
//  for (int i = 0; i < length; i++) {
//    Serial.print((char)message[i]);
//    messageTemp += (char)message[i];
//  }
//  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic room/lamp, you check if the message is either on or off. Turns the lamp GPIO according to the message
//  if(topic==subTopicLED){
//      Serial.print("Changing Room lamp to ");
//      if(messageTemp == "on"){
//        digitalWrite(lamp, HIGH);
//        Serial.print("On");
//      }
//      else if(messageTemp == "off"){
//        digitalWrite(lamp, LOW);
//        Serial.print("Off");
//      }
//  }
//  Serial.println();
//}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
 //     client.subscribe(subTopicLED);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
 
struct pms5003data data;
    
void loop() {
    if (!client.connected()) {
      reconnect();
  }
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  if (readPMSdata(&pmsSerial)) {
    // reading data was successful!
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 2.5: "); Serial.print(data.pm25_standard);
    Serial.println("---------------------------------------");
    
    static char temperatureTemp[7];
    dtostrf(t,6,2,temperatureTemp);
    static char humidityTemp[7];
    dtostrf(h,6,2,humidityTemp);
    static char PM25[7];
    sprintf(PM25, "%d", data.pm25_standard);
    
    Serial.println("Publish message: ");
    Serial.print("t=");    Serial.println(temperatureTemp);
    Serial.print("h=");   Serial.println(humidityTemp);
    Serial.print("PM="); Serial.println(PM25);
    client.publish(pubTopicT, temperatureTemp);
    client.publish(pubTopicH, humidityTemp);
    client.publish(pubTopicPM, PM25);        
    delay(5000);
  }
  }  

 
boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }
 
  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);
 
  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }
 
  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }
 
  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);
 
  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  
  return true;
}
