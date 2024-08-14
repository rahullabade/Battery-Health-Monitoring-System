/*
 * 
 * This is sample code written for CDAC students of ACTS Batch to work with MQTT protocol
 * Install Library PubSubClient by Nick O'Leary
 * How to Install -> Go to Sketch -> Include Library -> Manage Libraries -> Filter your search -> PubSubClient
 * https://www.arduino.cc/reference/en/libraries/pubsubclient/
 * https://pubsubclient.knolleary.net/api
 * https://www.arduino.cc/reference/en/libraries/wifi/
 * https://arduinojson.org/
 * 
 */
 
#include <WiFi.h>
#include <PubSubClient.h>  

#include <esp_wifi.h>  
#include <ArduinoJson.h>

// Set the custom MAC address in case your ESP32 is not regsitered with the acts network - wifi spoofing
//uint8_t newMACAddress[] = {0xf4, 0x96, 0x34, 0x9d, 0xe2, 0x66};  // a8:6d:aa:0e:61:f9


char sensor_data_format_for_mqtt_publish[256];

const char* ssid =   "One";                          //ssid - service set Identifier (Replace it with your ssid name)

const char* password =  "12345678";                     // replace with ssid paasword


/*ThingsPeak MQTT Specific Details Start --
                         // broker port number


  GET it from the MQTT Device
  Publish the channel Feed
  Name: MyChanName
Client ID: ENTER_MQTT_DEVICE_CLIENT_ID
Host: mqtt://    mqtt3.thingspeak.com
Port: 1883
Username: ENTER_MQTT_DEVICE_USERNAME
Password: ENTER_MQTT_DEVICE_PASSWORD


In the Payload pane, use the following settings:

Topic: channels/<channelID>/publish/fields/field<fieldnumber>

Payload: field1=45
         field2=56
  Note : ThingsPEAK MQTT doesn't support JSON format
*/

const char* mqttBroker = "mqtt3.thingspeak.com";                  // broker address - replace it with your broker address/cloud broker - test.mosquitto.org

const int   mqttPort = 1883;   

const char* clientID = "DCk4NwwjJzYzLDo2OyYNKjE";                   // client-id --> replace it in case willing to connect with same broker

const char* mqttusername = "DCk4NwwjJzYzLDo2OyYNKjE";

const char* mqttpassword = "G+G5oPOFU/OCGqNof5hvWLeD";
//channels/<channelID>/publish/fields/field<fieldnumber>
const char* field1mqtttopic = "channels/2624043/publish/fields/field1"; 
const char* field2mqtttopic = "channels/2624043/publish/fields/field2";
const char* field3mqtttopic = "channels/2624043/publish/fields/field3";
float field1, field2;
int field3;

// 2384508 is channel ID

/*ThingsPeak MQTT Specific Details End --*/


WiFiClient MQTTclient;

PubSubClient client(MQTTclient);

long lastReconnectAttempt = 0;
boolean reconnect()
{
  //boolean connect (clientID, [username, password], [willTopic, willQoS, willRetain, willMessage], [cleanSession])
  if (client.connect(clientID,mqttusername,mqttpassword)) {

    Serial.println("Attempting to connect broker");
    
  }
  return client.connected();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Attempting to connect...");
  WiFi.mode(WIFI_STA);      
 // esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]); // for wifi spoofing
  WiFi.begin(ssid, password); // Connect to WiFi.
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Couldn't connect to WiFi.");
  }
  Serial.print("ESP32 IP ADDRESS : ");
  Serial.println(WiFi.localIP());
  //Add details for MQTT Broker
  client.setServer(mqttBroker, mqttPort); // Connect to broker
  lastReconnectAttempt = 0;
}
void loop() {
  if (!client.connected())
  {
    long now = millis();  // Returns the number of milliseconds passed since the Arduino board began running the current program
    if (now - lastReconnectAttempt > 5000) { // Try to reconnect.
      lastReconnectAttempt = now;
      if (reconnect())
      { 
        lastReconnectAttempt = 0;
      }
    }
  }
  else 
  { 
    Serial.println("Connected to Broker --- !!");
    client.loop();
    
    field1 = random(-16,56); // Voltage Sensor
    //delay(500);
    field2 = random(0,100);  // Current Sensor
    //delay(500);
    field3 = random(0,350);  // Temperature Sensor
    //delay(500);
    Serial.println(field1);
    Serial.println(field2);
    Serial.println(field3);
    // Pubish Data to specific Fields   
    client.publish(field1mqtttopic, String(field1).c_str());  //(topicname, payload)
    delay(1000);
    client.publish(field2mqtttopic, String(field2).c_str());
    delay(1000);
    client.publish(field3mqtttopic, String(field3).c_str());
    delay(1000);
    Serial.println("Sensor data sent to ThingsPeak");
  }
}
