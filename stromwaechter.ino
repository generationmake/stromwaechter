/*
 *
 * Software for the stromwaechter pcb
 * http://stromwaechter.generationmake.de
 * Bernhard Mayer, DL1MAB, bernhard@generationmake.de
 * 2019-02-24 - initial version
 * 2019-05-27 - added mqtt topics from earlier versions: version, mac, ip, wifi quality
 * 2019-05-30 - cleaned up code and comments
 * 2019-06-01 - made mqtt messages more modular; included mac address in mqtt message
 * 2019-06-01 - added state to mqtt messages
 * 2019-06-03 - check bus voltage and turn channels on and off
 * 2019-06-03 - support up to 4 boards

 Based on the Basic ESP8266 MQTT example of the PubSubClient library.

it reads all the sensors and sends the status every 10 seconds.

MQTT messages:

<mac>/vbus                 - bus voltage in volt
<mac>/temperature          - temperature of board
<mac>/<numsensor>/voltage  - voltage in volt of channel
<mac>/<numsensor>/current  - current in ampere of channel
<mac>/<numsensor>/state    - state of channel (1=on; 0=off)
<mac>/version              - firmware version
<mac>/ip                   - ip address of board
<mac>/mac                  - mac address of board
<mac>/wlan                 - wifi quality

<mac> is like b4-e6-2d-3f-62-05
<numsensor> is 1 for the first channel, 2 for the second and so on

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h> // i2c-bib
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ADS1115_WE.h>
#include "i2c_helper.h"

#define ledPin 12       // the blue LED
#define NUM_SENSORS 8   // define number of sensors (max 16)
#define NUM_CURRENT 4   // define number of current sensors (max 4)

const float onoff[NUM_SENSORS][2]={
  {13.0,12.8},    // channel 1 start at 13.0 V, stop at 12.8 V
  {12.7,12.5},    // channel 2 start at 12.7 V, stop at 12.5 V
  {12.4,12.2},    // channel 3 start at 12.4 V, stop at 12.2 V
  {12.1,11.8},    // channel 4 start at 12.1 V, stop at 11.8 V
  {0,0},          // channel 5 start at 0.0 V, stop at 0.0 V
  {0,0},          // channel 6 start at 0.0 V, stop at 0.0 V
  {0,0},          // channel 7 start at 0.0 V, stop at 0.0 V
  {0,0}           // channel 8 start at 0.0 V, stop at 0.0 V
};

#define ONE_WIRE_BUS 2  // DS18B20 pin
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
ADS1115_WE ads1115 = ADS1115_WE();

// Update these with values suitable for your network.

const char* versionstring = "esp8266_stromwaechter_vx.y_20250822";   //is sent to MQTT broker
const char* ssid = "openhab";
const char* password = "openhabopenhab";
const char* mqtt_server = "192.168.35.1";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];         //buffer for mqtt messages
char esp_mac[18];     //buffer for MAC address
char esp_ip[17];      //buffer for IP address
IPAddress ip;         //the IP address of your shield

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

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(ledPin, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(ledPin, HIGH);  // Turn the LED off by making the voltage HIGH
  }
}

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
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  ads1115.setCompareChannels(channel);
  voltage = ads1115.getResult_mV(); // alternative: getResult_mV for Millivolt
  return voltage;
}
int readChannelraw(ADS1115_MUX channel) {
  int value = 0.0;
  ads1115.setCompareChannels(channel);
  value = ads1115.getRawResult(); // read raw value
  return value;
}

void setup() {
  byte error,address,i;
  byte temp[6];
  int nDevices;
  pinMode(ledPin, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);

  Wire.begin(4,5); // 4-data, 5-clk for nodeESP
  Wire.setClock(1000);
  delay(10);
  Serial.print("ChipID ");
  Serial.println(ESP.getChipId());

  DS18B20.begin();
  Serial.print("Number DS18B20. ");
  Serial.println(DS18B20.getDeviceCount());
  Serial.print("Parasite power mode: ");
  Serial.println(DS18B20.isParasitePowerMode());

  Serial.println("Scanning I2C...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  i2c_write_byte(0x20,0x03,0);
  i2c_write_byte(0x20,0x01,0);   // disable all outputs
  if(NUM_SENSORS>4)
  {
    i2c_write_byte(0x21,0x03,0);
    i2c_write_byte(0x21,0x01,0);   // disable all outputs
  }
  if(NUM_SENSORS>8)
  {
    i2c_write_byte(0x22,0x03,0);
    i2c_write_byte(0x22,0x01,0);   // disable all outputs
  }
  if(NUM_SENSORS>12)
  {
    i2c_write_byte(0x23,0x03,0);
    i2c_write_byte(0x23,0x01,0);   // disable all outputs
  }
  for(i=0;i<NUM_SENSORS;i++)
  {
    i2c_write_word(0x40+i,0x05,0x0400); // value for shunt 20 mOhm - max current=4.096 A
  }

  if(NUM_CURRENT>0)
  {
    if(!ads1115.init()){
      Serial.println("ADS1115 not connected!");
    }
    ads1115.setVoltageRange_mV(ADS1115_RANGE_4096); //comment line/change parameter to change range
    ads1115.setCompareChannels(ADS1115_COMP_0_GND); //comment line/change parameter to change channel
    ads1115.setMeasureMode(ADS1115_CONTINUOUS); //comment line/change parameter to change mode
  }

  setup_wifi();

  //get IP address, format and print
  ip = WiFi.localIP();
  sprintf(esp_ip, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  Serial.print("IP address:  ");
  Serial.println(esp_ip);

  //get MAC address, format and print
  WiFi.macAddress(temp);
  sprintf(esp_mac, "%02x-%02x-%02x-%02x-%02x-%02x", temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]);
  Serial.print("MAC address: ");
  Serial.println(esp_mac);

  // start MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
//  char esp_sub[50];     //buffer for subscribe string
  char esp_pub[80];     //buffer for publish string
  int rssi;    //wifi signal strength
  int quali;   //wifi signal quality
  int temp_count=0;
  static int delay_count=0;
  float input_voltage;
  float temp_ds=0;
  float current=0, voltage=0;
  static int state=0;
  int i=0;

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 10000)  // send message every 10 seconds
  {
    if(DS18B20.getDeviceCount()>0)
    {
      do {
        DS18B20.requestTemperatures();
  //      DS18B20.requestTemperaturesByIndex(0);
        delay(1000);  // delay for parasite power
        temp_ds = DS18B20.getTempCByIndex(0);
        Serial.print("Temperature: ");
        Serial.println(temp_ds);
        temp_count++;
        yield();
      } while ((temp_ds == 85.0 || temp_ds == (-127.0)) && temp_count < 20);
      if(temp_count==20) temp_ds=0;
    }

    // input voltage:
  //  input_voltage=(analogRead(A0))*17.5/1024.0; // for voltage divider 330k|20k
    input_voltage=(analogRead(A0))*18.37/1024.0; // for voltage divider 330k|19k
    Serial.print("Input voltage: ");
    Serial.println(input_voltage);

    lastMsg = now;
    if ( delay_count > 0 ) delay_count--;
    if ( delay_count == 0 ) {         //send these messages every 600 seconds
      delay_count = 6;
//      delay_count = 60;
      // send version
      snprintf (esp_pub, 50, "%s/version", esp_mac); // create topic with mac address
      client.publish(esp_pub, versionstring );  //module sends to this topic
      Serial.println("send - version");

      // send MAC
      snprintf (esp_pub, 50, "%s/mac", esp_mac); // create topic with mac address
      client.publish(esp_pub, esp_mac );        //module sends to this topic
      Serial.println("send - mac");

      // send IP
      snprintf (esp_pub, 50, "%s/ip", esp_mac); // create topic with mac address
      client.publish(esp_pub, esp_ip );         //module sends to this topic
      Serial.println("send - IP");

      // send wifi quality
      rssi = WiFi.RSSI();
      quali = 2*(rssi +100);
      if (quali > 100) { quali = 100; }
      if (quali < 0 ) { quali = 0; }
      sprintf(msg, "%ddBm / %d%%", rssi, quali);
      snprintf (esp_pub, 50, "%s/wlan", esp_mac); // create topic with mac address
      client.publish(esp_pub, msg);
      Serial.print("send - wifi quality: ");
      Serial.print(esp_pub);
      Serial.println(msg);
    }

    Serial.println("Publish mqtt messages");
    snprintf (msg, 50, "%f", input_voltage);
    snprintf (esp_pub, 50, "%s/vbus", esp_mac); // create topic with mac address
    client.publish(esp_pub, msg);
    snprintf (msg, 50, "%f", temp_ds);
    snprintf (esp_pub, 50, "%s/temperature", esp_mac); // create topic with mac address
    client.publish(esp_pub, msg);
    for(i=0;i<NUM_SENSORS;i++)
    {
//check and set state
      if(!(state&(1<<i))) // if state off
      {
        if(input_voltage>=onoff[i][0])
        {
          state|=(1<<i);  // bus voltage higher than defined
          Serial.println("turn channel on");
        }
      }
      else if(state&(1<<i)) // if state on
      {
        if(input_voltage<onoff[i][1])
        {
          state&=~(1<<i);  // bus voltage lower than defined
          Serial.println("turn channel off");
        }
      }

//enable/disable channels
      if(i<4) i2c_write_byte(0x20,0x01,state&0x000F);   // set outputs on first pcb
      else if(i<8) i2c_write_byte(0x21,0x01,(state>>4)&0x000F);   // set outputs on second pcb
      else if(i<12) i2c_write_byte(0x22,0x01,(state>>8)&0x000F);   // set outputs on second pcb
      else if(i<16) i2c_write_byte(0x23,0x01,(state>>12)&0x000F);   // set outputs on second pcb

// read sensor values
      voltage=i2c_read_word(0x40+i, 0x02)*0.00125;
      Serial.print("   Voltage: (V)");
      Serial.println(voltage);
      current=i2c_read_word(0x40+i, 0x04)*0.00025;
      Serial.print("   Current: (A)");
      Serial.println(current);

// publish mqtt messages
      snprintf (msg, 50, "%f", voltage);
      snprintf (esp_pub, 50, "%s/%i/voltage", esp_mac, i+1); // create topic with mac address
      client.publish(esp_pub, msg);
      snprintf (msg, 50, "%f", current);
      snprintf (esp_pub, 50, "%s/%i/current", esp_mac, i+1); // create topic with mac address
      client.publish(esp_pub, msg);
      if(!(state&(1<<i))) snprintf (msg, 50, "0");   // state off
      else snprintf (msg, 50, "1");
      snprintf (esp_pub, 50, "%s/%i/state", esp_mac, i+1); // create topic with mac address
      client.publish(esp_pub, msg);
    }
    for(i=0;i<NUM_CURRENT;i++)
    {

      float ads_voltage = 0.0;
      int ads_raw = 0;

      Serial.print("ADS1115 ");
      Serial.print(i);
      Serial.print(": ");
      if(i==0) ads_voltage = readChannel(ADS1115_COMP_0_GND);
      else if(i==1) ads_voltage = readChannel(ADS1115_COMP_1_GND);
      else if(i==2) ads_voltage = readChannel(ADS1115_COMP_2_GND);
      else ads_voltage = readChannel(ADS1115_COMP_3_GND);
      Serial.println(ads_voltage);
      snprintf (msg, 50, "%f", ads_voltage);
      // if(i==0) ads_raw = readChannelraw(ADS1115_COMP_0_GND);
      // else if(i==1) ads_raw = readChannelraw(ADS1115_COMP_1_GND);
      // else if(i==2) ads_raw = readChannelraw(ADS1115_COMP_2_GND);
      // else ads_raw = readChannelraw(ADS1115_COMP_3_GND);
      // Serial.println(ads_raw);
      // snprintf (msg, 50, "%i", ads_raw);
      snprintf (esp_pub, 50, "%s/c%i/voltage", esp_mac, i+1); // create topic with mac address
      client.publish(esp_pub, msg);
    }
  }
}
