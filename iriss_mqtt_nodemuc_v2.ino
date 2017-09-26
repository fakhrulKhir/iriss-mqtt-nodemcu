#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>
#include <OneWire.h>

OneWire  ds(D5);  // on pin 10 (a 4.7K resistor is necessary)

SoftwareSerial ss(D7,D8, false, 256); //connected to GPS module


int PERIOD = 10000; //5 saat  period utk baca sensor data

unsigned long previousMillis1=0;
long lastReconnectAttempt = 0;

static const uint32_t GPSBaud = 9600;

#define CHANNEL_ID "stat/IRISS_ESP"     //debug sensor2

#define TOPSZ  100 

// Offset hours from gps time (UTC)
const int offset = 8;   // Malaysia
//const int offset = -5;  // Eastern Standard Time (USA)
//const int offset = -4;  // Eastern Daylight Time (USA)
//const int offset = -8;  // Pacific Standard Time (USA)
//const int offset = -7;  // Pacific Daylight Time (USA)

int Hour,Minute,Second, Day, Month, Year;

TinyGPSPlus gps;
static const int MAX_SATELLITES = 40;


TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1); // $GPGSV sentence, first element
TinyGPSCustom messageNumber(gps, "GPGSV", 2);      // $GPGSV sentence, second element
TinyGPSCustom satsInView(gps, "GPGSV", 3);         // $GPGSV sentence, third element
TinyGPSCustom satNumber[4]; // to be initialized later
TinyGPSCustom elevation[4];
TinyGPSCustom azimuth[4];
TinyGPSCustom snr[4];

struct
{
  bool active;
  int elevation;
  int azimuth;
  int snr;
} sats[MAX_SATELLITES];

// Update these with values suitable for your network.

//const char* ssid = "xxxxx";
const char* ssid = "xxxx";
const char* password = "xxxxx";
const char* mqtt_server = "xxxxx";
//const char* mqtt_server = "xxxxx"; //ip default linkit smart duo

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;



void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  lastReconnectAttempt = 0;
}

void setup_wifi() {

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
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    }
    
    // Attempt to connect
    if (client.connect("espClient","xxxxx","xxxx")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish(CHANNEL_ID, "hello world");
      // ... and resubscribe
      client.subscribe("cmnd/IRISS_ESP");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  char stopic[TOPSZ], svalue[TOPSZ];

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  reconnect_mqtt();

  if (millis()-previousMillis1 > PERIOD){ 


    gps_data();
    getTemp();
 
   previousMillis1=millis();
 
  }
  //Serial.println(millis());
}





void gps_data(){

   // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}


void displayInfo(){
  char stopic[TOPSZ], svalue[TOPSZ];
  
if (gps.location.isValid()){
      Serial.print(F("Location: ")); 
      float lon = gps.location.lng();
      float latt = gps.location.lat();

      Serial.print(gps.location.lng(),6);  // 6 tempat perpuluhan
      Serial.print(",");
      Serial.println(gps.location.lat(),6);

      dtostrf(lon, 4, 6, svalue);
      snprintf_P(stopic, sizeof(stopic), PSTR("sensor/LON"));
      //snprintf_P(svalue, sizeof(svalue), PSTR("%s"),);
      client.publish(stopic,svalue);
      //
      dtostrf(latt, 4, 6, svalue);
      snprintf_P(stopic, sizeof(stopic), PSTR("sensor/LAT"));
      //snprintf_P(svalue, sizeof(svalue), PSTR("xxxx"));
      client.publish(stopic,svalue);
  }  
  if (gps.date.isValid() && gps.time.isValid()){
    Day=gps.date.day();
    Month=gps.date.month();
    Year=gps.date.year();
    Hour=gps.time.hour();
    Minute=gps.time.minute();
    Second=gps.time.second();

    

    snprintf_P(stopic, sizeof(stopic), PSTR("sensor/TIME"));
    snprintf_P(svalue, sizeof(svalue), PSTR("Date: %d-%d-%d Time: %d:%d:%d:"),Day,Month,Year,Hour,Minute,Second);
    client.publish(stopic,svalue);
    Serial.println(svalue);
    
    setTime(Hour, Minute, Second, Day, Month, Year);
    adjustTime(offset * SECS_PER_HOUR);
    
    long unixtime=now();
    Serial.print(F("Unixtime: "));

    snprintf_P(stopic, sizeof(stopic), PSTR("sensor/EPOCH"));
    snprintf_P(svalue, sizeof(svalue), PSTR("%ld"),unixtime);
    client.publish(stopic,svalue);
    Serial.println(unixtime);

  }
 Serial.println();
}


void reconnect_mqtt(){
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
//  long now = millis();
//  if (now - lastMsg > 2000) {
//    lastMsg = now;
//    ++value;
//    snprintf (msg, 75, "hello world #%ld", value);
//    Serial.print("Publish message: ");
//    Serial.println(msg);
//    client.publish(CHANNEL_ID, msg);
//  }
}


void getTemp(){
  char stopic[TOPSZ], svalue[TOPSZ];
  
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;
  
  if ( !ds.search(addr)) {
//    Serial.println("No more addresses.");
//    Serial.println();
    ds.reset_search();
    delay(250);
    return;
  }
  
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");

  reconnect_mqtt();


  dtostrf(celsius, 4, 2, svalue);
  snprintf_P(stopic, sizeof(stopic), PSTR("sensor/W_TEMP"));
  //snprintf_P(svalue, sizeof(svalue), PSTR("%ld"),unixtime);
  client.publish(stopic,svalue);

  
}
