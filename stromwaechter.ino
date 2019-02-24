/*
 * 
 * Software for the stromwaechter pcb
 * http://stromwaechter.generationmake.de
 * Bernhard Mayer, DL1MAB, bernhard@generationmake.de
 * 2019-02-24

 
 Based on the Basic ESP8266 MQTT example of the PubSubClient library.

it reads all the sensors and sends the status every 10 seconds.

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h> // i2c-bib
#include <OneWire.h>
#include <DallasTemperature.h>

float input_voltage;
float temp_ds=0;
float voltage1;
float current1;
float voltage2;
float current2;
float voltage3;
float current3;
float voltage4;
float current4;

#define ONE_WIRE_BUS 2  // DS18B20 pin
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

// Update these with values suitable for your network.

const char* ssid = "openhab";
const char* password = "openhabopenhab";
const char* mqtt_server = "192.168.35.1";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

unsigned short int swap_bytes(unsigned short int input)
{
  return (((input & 0xff) << 8) | ((input >> 8) & 0xff));
}

void i2c_write(char address, char value)
{
  Wire.beginTransmission(address);
  Wire.write(value);
  Wire.endTransmission();
}
void i2c_write_byte(char address, char reg, char value)
{
  Wire.beginTransmission(address);
  Wire.write(reg); // power up continous mode
  Wire.write(value);
  Wire.endTransmission();
}
void i2c_write_word(char address, char reg, short int value)
{
  Wire.beginTransmission(address);
  Wire.write(reg); // power up continous mode
  Wire.write((value&0xff00)>>8);
  Wire.write(value&0x00ff);
  Wire.endTransmission();
}
char i2c_read_byte(char address, char reg)
{
  char x=0;
  Wire.beginTransmission(address);
  Wire.write(reg); // power up state
  Wire.endTransmission();
  Wire.requestFrom(address,1);
  if(Wire.available()) x=Wire.read();
  Wire.endTransmission();
  return x;
}
char i2c_smbus_read_byte_data(char address, char reg)
{
  return i2c_read_byte(address, reg);
}
short int i2c_read_word(char address, char reg)
{
  short int x=0;
  Wire.beginTransmission(address);
  Wire.write(reg); // power up state
  Wire.endTransmission();
  Wire.requestFrom(address,2);
  if(Wire.available()) x=Wire.read();
  x=x<<8;
  if(Wire.available()) x=x|Wire.read();
  Wire.endTransmission();
  return x;
}
int i2c_read_wordp(char address, char reg)
{
  int x=0;
  Wire.beginTransmission(address);
  Wire.write(reg); // power up state
  Wire.endTransmission();
  Wire.requestFrom(address,3);
  if(Wire.available()) x=Wire.read();
  x=x<<8;
  if(Wire.available()) x=x|Wire.read();
  x=x<<4;
  if(Wire.available()) x=x|(Wire.read()>>4);
  Wire.endTransmission();
  return x;
}
short int i2c_read_word_rev(char address, char reg)
{
  short int x=0;
  Wire.beginTransmission(address);
  Wire.write(reg); // power up state
  Wire.endTransmission();
  Wire.requestFrom(address,2);
  if(Wire.available()) x=Wire.read();
  if(Wire.available()) x=x|(Wire.read()<<8);
  Wire.endTransmission();
  return x;
}
short int i2c_smbus_read_word_data(char address, char reg)
{
  return i2c_read_word_rev(address, reg);
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

  randomSeed(micros());

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
    digitalWrite(12, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(12, HIGH);  // Turn the LED off by making the voltage HIGH
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
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
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

void setup() {
  byte error,address,x;
  short int sx=0;
  int nDevices;
  pinMode(12, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
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
// variables for DS18B20
  int temp_count=0;

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

  Serial.println("Scanning...");
 
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
  i2c_write_byte(0x20,0x01,0xFF);
  i2c_write_word(0x40,0x05,swap_bytes(0x0800));
  i2c_write_word(0x41,0x05,swap_bytes(0x0800));
  i2c_write_word(0x42,0x05,swap_bytes(0x0800));
  i2c_write_word(0x43,0x05,swap_bytes(0x0800));
  
  // input voltage:
  input_voltage=(analogRead(A0))*17.5/1024.0;
  Serial.print("Input voltage: ");
  Serial.println(input_voltage);
  
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {

  int temp_count=0;
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 10000) {
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
  input_voltage=(analogRead(A0))*17.5/1024.0;
  Serial.print("Input voltage: ");
  Serial.println(input_voltage);
  

  voltage1=i2c_read_word(0x40, 0x02)*0.00125;
  Serial.print("   Voltage 1: (V)");
  Serial.println(voltage1);
  current1=i2c_read_word(0x40, 0x04)*0.00025;
  Serial.print("   Current 1: (A)");
  Serial.println(current1);
  voltage2=i2c_read_word(0x41, 0x02)*0.00125;
  Serial.print("   Voltage 2: (V)");
  Serial.println(voltage2);
  current2=i2c_read_word(0x41, 0x04)*0.00025;
  Serial.print("   Current 2: (A)");
  Serial.println(current2);
  voltage3=i2c_read_word(0x42, 0x02)*0.00125;
  Serial.print("   Voltage 3: (V)");
  Serial.println(voltage3);
  current3=i2c_read_word(0x42, 0x04)*0.00025;
  Serial.print("   Current 3: (A)");
  Serial.println(current3);
  voltage4=i2c_read_word(0x43, 0x02)*0.00125;
  Serial.print("   Voltage 4: (V)");
  Serial.println(voltage4);
  current4=i2c_read_word(0x43, 0x04)*0.00025;
  Serial.print("   Current 4: (A)");
  Serial.println(current4);

    lastMsg = now;
    ++value;
    snprintf (msg, 50, "%f", input_voltage);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish("stromwaechter/vbus", msg);
    snprintf (msg, 50, "%f", temp_ds);
    client.publish("stromwaechter/temperature", msg);
    snprintf (msg, 50, "%f", voltage1);
    client.publish("stromwaechter/1/voltage", msg);
    snprintf (msg, 50, "%f", current1);
    client.publish("stromwaechter/1/current", msg);
    snprintf (msg, 50, "%f", voltage2);
    client.publish("stromwaechter/2/voltage", msg);
    snprintf (msg, 50, "%f", current2);
    client.publish("stromwaechter/2/current", msg);
    snprintf (msg, 50, "%f", voltage3);
    client.publish("stromwaechter/3/voltage", msg);
    snprintf (msg, 50, "%f", current3);
    client.publish("stromwaechter/3/current", msg);
    snprintf (msg, 50, "%f", voltage4);
    client.publish("stromwaechter/4/voltage", msg);
    snprintf (msg, 50, "%f", current4);
    client.publish("stromwaechter/4/current", msg);
  }
}
