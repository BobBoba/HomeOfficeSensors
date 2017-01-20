#include <TaskScheduler.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>

#include "Adafruit_BMP085.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <deque>
#include <queue>

#include "passwords.h"

#define MILLISECONDS_IN_SECOND 1000

template<typename T, typename Container = std::deque<T>>
class iterable_queue : public std::queue<T, Container>
{
public:
	typedef typename Container::iterator iterator;
	typedef typename Container::const_iterator const_iterator;

	iterator begin() { return this->c.begin(); }
	iterator end() { return this->c.end(); }
	const_iterator begin() const { return this->c.begin(); }
	const_iterator end() const { return this->c.end(); }
};

#define DISPLAY_I2C_ADDR 0x27
#define DISPLAY_WIDTH 16
#define DISPLAY_LINES 2

//LiquidCrystal_I2C lcd(0x27, 16, 2); // Устанавливаем дисплей
LiquidCrystal_I2C lcd(DISPLAY_I2C_ADDR, DISPLAY_WIDTH, DISPLAY_LINES); // Устанавливаем дисплей

//#define MAC "18FE34D3F6C5"

//IPAddress server(192, 168, 0, 10); // mosquitto address
WiFiClientSecure wclient;
//PubSubClient client(server, 1883, wclient);
//mac: 18fe34d3f6c5
int counter = 0;
int ms = 0;
String mac = "";
//String pubTopic;// = "/ESP8266/"MAC"/DATA";
//String pubTopic2;// = "/ESP8266/"MAC;
//String controlTopic;// = "/ESP8266/"MAC"/CONTROL/#";
//String statTopic;// = "/ESP8266/"MAC"/STATUS/GPIO/4";
int GPIO = 4;
const char* mqtt_user = "";
const char* mqtt_pass = "";
const char* mqtt_client = "ESP8266";
int heap = 0;

int lastSend1 = 0;
int lastSend2 = 0;

#define AM2302PIN 12 //     // what digital pin we're connected to
#define AM2301PIN 14 // out    // what digital pin we're connected to
//#define PIRPIN 16

// Uncomment whatever type you're using!
// DHT 11 : 
// DHT 22 : (AM2302), AM2321
// DHT 21 : (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dhtIn(AM2302PIN, DHT22);
DHT dhtOut(AM2301PIN, DHT21);

// Callback methods prototypes
void MainTask();
//void SendToMQTT();
//void MQPinger();
void SendToIoTHub();

Scheduler runner;

//Tasks
//Task t4();
Task EverySecondTask(3 * MILLISECONDS_IN_SECOND, TASK_FOREVER, &MainTask, &runner);
Task IoTSenderTask(60 * MILLISECONDS_IN_SECOND, TASK_FOREVER, &SendToIoTHub, &runner);
//Task MQPingerTask(3 * MILLISECONDS_IN_SECOND, TASK_FOREVER, &MQPinger, &runner);
//Task MQSenderTask(60 * MILLISECONDS_IN_SECOND, TASK_FOREVER, &SendToMQTT, &runner);
//Task TemperatureProcecessorTask(60000, TASK_FOREVER, &TemperatureProcessor);

byte degree[8] = {
	B00010,
	B00101,
	B00010,
	B00000,
	B00000,
	B00000,
	B00000,
};

iterable_queue<double> valuesQueue;

/***********************Software Related Macros************************************/
#define         READ_SAMPLE_INTERVAL         (10)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (10)     //define the time interval(in milisecond) between each samples in 

// усредненное за n-замеров значение датчика
double co2volts = 0;

// усредненное значение за последние измерения, взятые из истории
double co2volts2 = 0;

double tempIn = 0;
double humidityIn = 0;
double tempOut = 0;
double humidityOut = 0;

int lastSentPirState = -1;
int pirState = 0;

#define AVERAGE_QUEUE_SIZE (1 * 60) // n-minutes

//ADC
Adafruit_ADS1115 ads(0x48);

//Pressure
Adafruit_BMP085 bmp;
double bmpTemperature = 0.0, bmpPressure = 0.0, bmpAltitude = 0.0;


void httpPost(String content)
{
	wclient.stop(); // закрываем подключение, если вдруг оно открыто

	if (wclient.connect(azureHost, azurePort)) {
		wclient.print("POST ");
		wclient.print(azureUri);
		wclient.println(" HTTP/1.1");
		wclient.print("Host: ");
		wclient.println(azureHost);
		wclient.print("Authorization: ");
		wclient.println(authSAS);
		wclient.println("Connection: close");
		wclient.print("Content-Type: ");
		wclient.println("text/plain");
		wclient.print("Content-Length: ");
		wclient.println(content.length());
		wclient.println();
		wclient.println(content);
		delay(200);
	}
	else {
		Serial.println("HTTP POST send failed");
	}
}

JsonObject& prepareJson()
{
	StaticJsonBuffer<1024> jsonBuffer;
	JsonObject& jsonObj = jsonBuffer.createObject();

	jsonObj["MAC"] = mac;

	JsonObject& jsonBMP = jsonObj.createNestedObject("BMP085");

	jsonBMP["temperature"] = false == isnan(bmpTemperature) ? bmpTemperature : 0.0;
	jsonBMP["altitude"] = false == isnan(bmpAltitude) ? bmpAltitude : 0.0;
	jsonBMP["pressure"] = false == isnan(bmpPressure) ? bmpPressure : 0.0;

	JsonObject& jsonDHT22in = jsonObj.createNestedObject("DHT22in"); // IN
	jsonDHT22in["temperature"] = false == isnan(tempIn) ? tempIn : 0.0;
	jsonDHT22in["humidity"] = false == isnan(humidityIn) ? humidityIn : 0.0;

	JsonObject& jsonDHT21out = jsonObj.createNestedObject("DHT21out"); // OUT
	jsonDHT21out["temperature"] = false == isnan(tempOut) ? tempOut : 0.0;
	jsonDHT21out["humidity"] = false == isnan(humidityOut) ? humidityOut : 0.0;

	//if (DS18x20s.size() > 0)
	//{
	//	JsonArray& jsonDS18x20s = jsonObj.createNestedArray("DS18x20");

	//	for (int p = 0, pc = DS18x20s.size(); p < pc; ++p)
	//	{
	//		JsonObject& jsonDS18x20 = jsonDS18x20s.createNestedObject();
	//		jsonDS18x20["address"] = DS18x20s[p].saddr;
	//		jsonDS18x20["temperature"] = DS18x20s[p].celsius;
	//	}
	//}

	return jsonObj;// .printTo(buffer);

}

void SendToIoTHub()
{
	JsonObject& json = prepareJson();

	String buffer;
	json.printTo(buffer);

	Serial.println("Send updated data to IoT Hub");
	Serial.println(buffer);

	//return;//temporary

	httpPost(buffer);

	String response = "";
	char c;
	while (wclient.available()) {
		c = wclient.read();
		response.concat(c);
	}
	if (response.equals(""))
	{
		Serial.println("empty response");
	}
	else
	{
		if (response.startsWith("HTTP/1.1 204")) {
			Serial.println("String has been successfully send to Azure IoT Hub");
		}
		else {
			Serial.println("Error");
			Serial.println(response);
		}
	}

}

void restart() {
	Serial.println("Will reset and try again...");
	abort();
}
void setup()
{
	// Setup console
	Serial.begin(74880);
	Serial.println();
	Serial.println();

	uint8_t macAddr[6];
	WiFi.macAddress(macAddr);
	//MAC2STR()
	for (int i = 0; i < 6; ++i)
		mac += String(macAddr[i], 16);
	mac.toUpperCase();

	if (!bmp.begin()) {
		Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	}

	lcd.init();
	lcd.createChar(0, degree);
	lcd.noAutoscroll();
	lcd.backlight();// Включаем подсветку дисплея

	lcd.print("Welcome!!!");

	lcd.setCursor(0, 1);
	lcd.print("MAC:");
	lcd.print(mac);

	delay(2000);


	lcd.clear();

	lcd.setCursor(0, 0);
	lcd.print("WiFi...");


	WiFi.begin(ssid, pass);

	Serial.print("Connecting to WiFi...");
	int counter = 0;
	while (WiFi.status() != WL_CONNECTED) {
		delay(1000);
		Serial.print(".");
		lcd.print(".");
		//display.clear();
		//display.drawString(64, 10, "Connecting to WiFi");
		//display.drawXbm(46, 30, 8, 8, counter % 3 == 0 ? activeSymbole : inactiveSymbole);
		//display.drawXbm(60, 30, 8, 8, counter % 3 == 1 ? activeSymbole : inactiveSymbole);
		//display.drawXbm(74, 30, 8, 8, counter % 3 == 2 ? activeSymbole : inactiveSymbole);
		//display.display();

		counter++;
	}
	Serial.println(" done!!!");
	lcd.print(" OK");


	lcd.setCursor(0, 1);
	lcd.print("Azure...");

	Serial.print("Connecting to ");
	Serial.print(azureHost);
	while (!wclient.connect(azureHost, azurePort)) {
		Serial.println(" failed");
		lcd.print(" Error");
		delay(1000);
		//return;
	}
	Serial.println(" done!!!");
	lcd.print(" OK");
	delay(500);


	EverySecondTask.enable();

	IoTSenderTask.enableDelayed(5 * MILLISECONDS_IN_SECOND);
}

void MainTask()
{
	//bool valid = true;
	static int counter = 0;

	delay(10);
	while (isnan(bmpTemperature = bmp.readTemperature()));
	{
		delay(1);
	}
	Serial.print("BMP Temp(C): ");
	Serial.print(bmpTemperature, 1);

	delay(10);
	while (isnan(bmpPressure = bmp.readPressure() / 133.3));
	{
		delay(1);
	}
	Serial.print(" | ");
	Serial.print("Pressure(mm): ");
	Serial.print(bmpPressure, 1);

	delay(10);
	while (isnan(bmpAltitude = bmp.readAltitude()));
	{
		delay(1);
	}
	Serial.print(" | ");
	Serial.print("Alt(m): ");
	Serial.print(bmpAltitude);

	Serial.println();

	// Reading temperature or humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

	delay(10);
	while (isnan(tempIn = dhtIn.readTemperature()))
	{
		delay(1);
	}
	Serial.print("tempIn(C): ");
	Serial.print(tempIn, 1);

	delay(10);
	while (isnan(humidityIn = dhtIn.readHumidity()))
	{
		delay(1);
	}
	Serial.print(" | ");
	Serial.print("humidityIn(%): ");
	Serial.print(humidityIn, 1);


	// Read temperature as Celsius (the default)
	delay(10);
	while (isnan(tempOut = dhtOut.readTemperature()))
	{
		delay(1);
	}
	Serial.print(" | ");
	Serial.print("tempOut(C): ");
	Serial.print(tempOut, 1);


	delay(10);
	while (isnan(humidityOut = dhtOut.readHumidity()))
	{
		delay(1);
	}

	Serial.print(" | ");
	Serial.print("humidityOut(%): ");
	Serial.print(humidityOut, 1);


	//Serial.print(" Out:");
	//Serial.print(tempOut, 1);

	//Serial.print(" | humidity: ");
	//Serial.print(humidityIn, 1);
	//Serial.print(" Out: ");
	//Serial.print(humidityOut, 1);

	Serial.println();

	delay(10);

	lcd.setCursor(0, 0);

	if (counter++ % 5 == 0)
	{
		//lcd.clear();
		//lcd.setCursor(0, 0);
		//String lt = "Temp: ";
		//lt += t;

		//lcd.print("Pressure: ");
		//lcd.print(bmpPressure / 133.3, 1);
		//lcd.print("mm");

		int w = 0;
		w += lcd.print("T ");
		w += lcd.print(tempIn, 1);
		w += lcd.write(byte(0)); // degrees symbol
		w += lcd.print(" / Pr(mm)");

		for (int i = w; i < DISPLAY_WIDTH; ++i)
			lcd.print(" ");

		w = 0;
		lcd.setCursor(0, 1);
		w += lcd.print("H ");
		w += lcd.print(humidityIn, 1);
		w += lcd.print("%");
		w += lcd.print(" / ");
		w += lcd.print(bmpPressure, 1);
		w += lcd.print("");
		for (int i = w; i < DISPLAY_WIDTH; ++i)
			lcd.print(" ");

	}
	else {

		int w = 0;
		w += lcd.print("T ");
		w += lcd.print(tempIn, 1);
		w += lcd.write(byte(0)); // degrees symbol
		w += lcd.print(" / ");
		w += lcd.print(tempOut, 1);
		w += lcd.write(byte(0)); // degrees symbol
		for (int i = w; i < DISPLAY_WIDTH; ++i)
			lcd.print(" ");

		w = 0;
		lcd.setCursor(0, 1);
		w += lcd.print("H ");
		w += lcd.print(humidityIn, 1);
		w += lcd.print("%");
		w += lcd.print(" / ");
		w += lcd.print(humidityOut, 1);
		w += lcd.print("%");
		for (int i = w; i < DISPLAY_WIDTH; ++i)
			lcd.print(" ");
	}

	if (ESP.getFreeHeap() < 1000) {
		Serial.println("Detect mem leak!");

		lcd.clear();
		lcd.print("Memory Leak !!!");
		delay(3000);
		restart();
	}
}

void loop()
{
	runner.execute();
}
