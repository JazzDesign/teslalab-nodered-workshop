#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <BME280I2C.h>
#include <Wire.h>

// Network constants
#define TEAM_NAME "node2"
#define PUBLISH_PERIOD 1000

const char* ssid = "dlink-380E";
//const char* ssid = "IoTNetwork2.4GHz";
const char* password = "iot_ugal";
const char* mqtt_server = "192.168.8.115";

// I/O constants
#define TEMP_PIN A0
#define LED_PIN D5

int red=0;
int blue = 0;
int green = 0;

int redPin = D6;
int greenPin = D8;
int bluePin = D7;

//Button
int buttonPin = D1;

// Variables will change:
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;  //

// data variables
int lastReadingTime = 0;
double temp = 0;

char msg[50];
char cnt[50];
char msg_r[50];
char topic_name[250];

// network variables
WiFiClient espClient;
PubSubClient mqtt_client(espClient);


// temp
BME280I2C::Settings mysettings; 
//(
//  BME280I2C::OSR_X1,
//  BME280I2C::OSR_X1,
//  BME280I2C::OSR_X1,
//  BME280I2C::Mode_Forced,
//  BME280I2C::StandbyTime_1000ms,
//  BME280I2C::Filter_Off,
//  BME280I2C::SpiEnable_False,
//  0x77
//);

BME280I2C bme(mysettings);    // Default : forced mode, standby time = 1000 ms
                  // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,


void setupTempSensor() {
  Serial.println("Setting up BME280 Sensor");
  mysettings.bme280Addr = 0x77;
  bme = BME280I2C(mysettings);
  Wire.begin(D4, D3); // S
  while(!bme.begin())
  //while(!bme.begin(0x77))
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }
  
}
void setupMQTT() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.setCallback(callback);
}

void setup() {
  Serial.begin(115200);
  setupMQTT();
  setupTempSensor();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(buttonPin, INPUT);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    msg_r[i] = (char)payload[i];
  }
  msg_r[length] = 0;
  Serial.print("'");
  Serial.print(msg_r);
  Serial.println("'");
  if (strcmp("ON", msg_r) == 0) {
    Serial.println("LED ON");
    digitalWrite(LED_PIN, HIGH);
  } else if(strcmp("OFF", msg_r) == 0) {
    Serial.println("LED OFF");
    digitalWrite(LED_PIN, LOW);
  }else{
    colorConverter(msg_r);
  }
}

void colorConverter(String hexValue)
{
    int number = (int) strtol( &hexValue[1], NULL, 16);
    int r = number >> 16;
    int g = number >> 8 & 0xFF;
    int b = number & 0xFF;

    Serial.print("red is ");
    Serial.println(r);
    Serial.print("green is ");
    Serial.println(g);
    Serial.print("blue is ");
    Serial.println(b);
    setColor((r+50), g, b);

}

void setColor(int red, int green, int blue)
{
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
void reconnect() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect(TEAM_NAME)) {
      Serial.println("connected");
      mqtt_client.subscribe(getTopic("btn"));
      mqtt_client.subscribe(getTopic("rgb"));
    }else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void publish(char* topic, char* payload) {
  Serial.println(topic_name);
  mqtt_client.publish(topic_name, payload);
}

char* getTopic(char* topic) {
  sprintf(topic_name, "/%s/%s", TEAM_NAME, topic);
  return topic_name;
}

void loop() {
  if (!mqtt_client.connected()) {
    reconnect();
  }
  mqtt_client.loop();

  int actualTime = millis();
  if (actualTime - lastReadingTime > PUBLISH_PERIOD) {
    lastReadingTime = actualTime;

    float temp(NAN), hum(NAN), pres(NAN);
  
     BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
     BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  
     bme.read(pres, temp, hum, tempUnit, presUnit);
    
//    Serial.println(temp);
    String str(temp);
    str.toCharArray(msg, 50);
    mqtt_client.publish(getTopic("temp"), msg);     
  }
  buttonState = digitalRead(buttonPin);

    // compare the buttonState to its previous state
    if (buttonState != lastButtonState) {
      // if the state has changed, increment the counter
      if (buttonState == HIGH) {
        // if the current state is HIGH then the button went from off to on:
        buttonPushCounter++;
        Serial.println("on");
        Serial.print("number of button pushes: ");
        Serial.println(buttonPushCounter);
      } else {
        // if the current state is LOW then the button went from on to off:
        Serial.println("off");
      }
      // Delay a little bit to avoid bouncing
      delay(50);
    }
    // save the current state as the last state, for next time through the loop
    lastButtonState = buttonState;
    String str1(buttonPushCounter);
    str1.toCharArray(cnt, 50);
    mqtt_client.publish(getTopic("count"), cnt );
}
