#include <TaskScheduler.h>
#include <Adafruit_Sensor.h>
#include "passwords.h"
#include <Wire.h>
#include <Adafruit_ADS1015.h>
//#include <MQTT.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <ESP8266WiFi.h>

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

IPAddress server(192, 168, 0, 10); // mosquitto address
WiFiClient wclient;
PubSubClient client(server, 1883, wclient);
//mac: 18fe34d3f6c5
int counter = 0;
int ms = 0;
String mac = "";
String pubTopic;// = "/ESP8266/"MAC"/DATA";
String pubTopic2;// = "/ESP8266/"MAC;
String controlTopic;// = "/ESP8266/"MAC"/CONTROL/#";
String statTopic;// = "/ESP8266/"MAC"/STATUS/GPIO/4";
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
void SendToMQTT();
void MQPinger();

Scheduler runner;

//Tasks
//Task t4();
Task EverySecondTask(3 * MILLISECONDS_IN_SECOND, TASK_FOREVER, &MainTask, &runner);
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
long bmpTemperature = 0, bmpPressure = 0, bmpAltitude = 0;


//void callback(const MQTT::Publish& sub) {
void callback(const char* topic, byte* payload, unsigned int length)
{
	Serial.print("Get data from subscribed topic ");
	Serial.print(topic);
	Serial.print(" => ");
	Serial.println(reinterpret_cast<char*>(payload));
	String expected = "/ESP8266/" + mac + "/GPIO/";
	expected += GPIO;
	if (expected == topic) {
		String s = "ESP8266 GPIO_";
		s += GPIO;
		String stat = "{\"status\":";
		if (strcmp(reinterpret_cast<char*>(payload), "true") == 0) {
			digitalWrite(GPIO, 0);
			s += " set LOW";
			stat += "\"LOW\"}";
		}
		else {
			digitalWrite(GPIO, 1);
			s += " set HIGH";
			stat += "\"HIGH\"}";
		}
		client.publish(statTopic.c_str(), stat.c_str());
		Serial.println(s);
	}
	// echo
	//MQTT::Publish newpub(pubTopic, sub.payload(), sub.payload_len());
	//client.publish(newpub);
	client.publish(pubTopic.c_str(), payload, length);
}

void restart() {
	Serial.println("Will reset and try again...");
	abort();
}

float getMGValue()
{
	int32_t adc = ads.readADC_SingleEnded(0);

	float Voltage = (adc * 7.812738425855281e-6) * 1000.0; // x16
	//float Voltage = (adc * 1.5625476851710562456129642628254e-5) * 1000.0; // x8
	return Voltage;
}

float GetAverageVoltage()
{
	//int16_t adc0;  // we read from the ADC, we have a sixteen bit integer as a result
	//Voltage /= 16;
	//int32_t adc = 0;

	float Voltage = 0;

	int i;
	//float v = 0;

	for (i = 0; i < READ_SAMPLE_TIMES; i++) {
		Voltage += getMGValue();// ads.readADC_SingleEnded(0);
		//v += analogRead(mg_pin);
		delay(READ_SAMPLE_INTERVAL);
	}
	//v = (v / READ_SAMPLE_TIMES) * 5 / 1024;
	Voltage = Voltage / READ_SAMPLE_TIMES;
	//float Voltage = (adc * 0.1875) / 1000;
	//float Voltage = (adc * 7.812738425855281e-6) * 1000.0;
	//float Voltage = (adc * 1.5625476851710562456129642628254e-5) * 1000.0;

	//Serial.print("     ");
	//Serial.print("MG811 ADC: ");
	//Serial.print(adc);
	//Serial.print("MG811: ");
	//Serial.print(Voltage, 2);
	//Serial.println(" mV");

	return Voltage;
}

float GetAverageVoltage2()
{
	auto size = valuesQueue.size();

	//if (size < AVERAGE_QUEUE_SIZE)
	if (size <= 0) //paranoid check
		return 0;

	float Voltage = 0;
	for (auto begin = valuesQueue.begin(), end = valuesQueue.end(); begin != end; ++begin) {
		Voltage += *begin;

	}
	//v = (v / READ_SAMPLE_TIMES) * 5 / 1024;
	Voltage = Voltage / size;

	return Voltage;
}


void SendToMQTT()
{
	//*
	String payload = "{";

	bool needComma = false;

	if (humidityIn > 0)
	{
		payload += "\"Humidity\":"; payload += humidityIn;
		needComma = true;
	}

	if (tempIn > 0)
	{
		if (needComma)
			payload += ",";
		payload += "\"Temperature\":"; payload += tempIn;
		needComma = true;
	}

	if (valuesQueue.size() >= AVERAGE_QUEUE_SIZE && co2volts2 > 0)
	{
		if (needComma)
			payload += ",";
		payload += "\"MG811\":"; payload += String(co2volts2, 3);
		needComma = true;
	}

	// чтобы не слать одно и тоже по тсо раз, экономим трафик
	//if (lastSentPirState != pirState)
	//{
	lastSentPirState = pirState;

	if (needComma)
		payload += ",";
	payload += "\"PIR\":"; payload += pirState;
	needComma = true;
	//}
	//payload += ",\"HeatIndex\":"; payload += hic;
	payload += "}";

	Serial.print("Publish JSON: ");
	Serial.print(payload);
	if (client.publish(pubTopic2.c_str(), payload.c_str())) {
		Serial.println(" - Ok");
	}
	else {
		Serial.println(" - Failed");
		//restart();
	}//*/
}

void setup()
{
	uint8_t macAddr[6];
	WiFi.macAddress(macAddr);
	//MAC2STR()
	for (int i = 0; i < 6; ++i)
		mac += String(macAddr[i], 16);

	pubTopic = "/ESP8266/" + mac + "/DATA";
	pubTopic2 = "/ESP8266/" + mac;
	controlTopic = "/ESP8266/" + mac + "/CONTROL/#";
	statTopic = "/ESP8266/" + mac + "/STATUS/GPIO/4";


	//runner.init();

	//runner.addTask(EverySecondTask);
	//runner.addTask(MQSenderTask);
	//runner.addTask(TemperatureProcecessorTask);

	//ads.setGain(GAIN_SIXTEEN);
	//ads.begin();

	if (!bmp.begin()) {
		Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	}

	lcd.init();
	lcd.noAutoscroll();
	lcd.backlight();// Включаем подсветку дисплея

	lcd.print("Welcome!!!");

	lcd.setCursor(0, 1);
	lcd.print("MAC:");
	lcd.print(mac);

	delay(10);

	lcd.createChar(0, degree);

	// Setup console
	Serial.begin(115200);
	delay(2000);
	Serial.println();
	Serial.println();

	//pinMode(GPIO, OUTPUT);
	//digitalWrite(GPIO, 1);

	client.setCallback(callback);

	//Serial.print("Connecting to ");
	//Serial.println(ssid);

	/*
	lcd.setCursor(0, 1);
	lcd.print("Connecting");

	WiFi.begin(ssid, pass);

	char progress[] = "/-\|";
	int idx = 0;
	int retries = 0;
	while ((WiFi.status() != WL_CONNECTED) && (retries < 20)) {
	retries++;
	delay(500);
	Serial.print(".");
	lcd.setCursor(15, 1);
	lcd.print(progress[idx]);
	++idx;
	if (idx >= 3)
	idx = 0;
	}
	if (WiFi.status() == WL_CONNECTED) {
	Serial.println("");
	Serial.println("WiFi connected");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());
	WiFi.printDiag(Serial);

	lcd.setCursor(0, 1);
	lcd.print("                ");
	lcd.setCursor(0, 1);
	lcd.print(WiFi.localIP());
	}
	else {
	restart();
	}

	pinMode(PIRPIN, INPUT);
	*/
	//Serial.println("Connecting to MQTT broker ");
	//if (client.connect(MQTT::Connect(mqtt_client).set_auth(mqtt_user, mqtt_pass))) {
	//	Serial.println("Connected to MQTT broker");
	//	client.subscribe(controlTopic);
	//}
	//else {
	//	//restart();
	//}

	//delay(2000);
	//TemperatureProcessor();

	//lastSend2 = secondsSinceStart;


	EverySecondTask.enable();
	//MQPingerTask.enable();
	//MQSenderTask.enableDelayed(3 * MILLISECONDS_IN_SECOND);

	//TemperatureProcecessorTask.enableDelayed(2000);

	//heap = ESP.getFreeHeap();
}

void MainTask()
{

	static int counter = 0;


	bmpTemperature = bmp.readTemperature();
	bmpPressure = bmp.readPressure();
	bmpAltitude = bmp.readAltitude();

	Serial.print("BMP Temp(C):");
	Serial.print(bmpTemperature*0.1, 1);
	Serial.print("  Alt(m):");
	Serial.print(bmpAltitude*0.01);
	Serial.print("  Pressure(mm):");
	Serial.println(bmpPressure / 133.3, 1);




	// Reading temperature or humidity takes about 250 milliseconds!
	// Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

	//tempIn = dhtIn.readTemperature();
	tempIn = dhtIn.readTemperature();
	humidityIn = dhtIn.readHumidity();

	// Read temperature as Celsius (the default)
	tempOut = dhtOut.readTemperature();
	humidityOut = dhtOut.readHumidity();
	if (isnan(humidityIn) || isnan(tempIn)
		||
		isnan(humidityOut) || isnan(tempOut)
		)
	{
		Serial.println("Failed to read from Internal DHT sensor!");

		return;
	}

	// Read temperature as Fahrenheit (isFahrenheit = true)
	//float f = dht.readTemperature(true);
	// Compute heat index in Fahrenheit (the default)
	// float hif = dht.computeHeatIndex(f, h);
	// Compute heat index in Celsius (isFahreheit = false)
	//float hic = dht.computeHeatIndex(t, h, false);
	//*
	// Check if any reads failed and exit early (to try again).



	//auto co2volts = GetAverageVoltage2();

	/*pirState = digitalRead(PIRPIN);


	co2volts = GetAverageVoltage();

	if (valuesQueue.size() >= AVERAGE_QUEUE_SIZE)
	valuesQueue.pop();
	valuesQueue.push(co2volts);

	co2volts2 = GetAverageVoltage2();
	*/

	Serial.print("temperature: In:");
	Serial.print(tempIn, 1);
	Serial.print(" Out:");
	Serial.print(tempOut, 1);

	Serial.print(" | humidity: ");
	Serial.print(humidityIn, 1);
	Serial.print(" Out: ");
	Serial.print(humidityOut, 1);

	Serial.println();

	//Serial.print("  co2volts: ");
	//Serial.print(co2volts, 3);
	//
	//Serial.print("  co2volts2[");
	//Serial.print(valuesQueue.size());
	//Serial.print("/");
	//Serial.print(AVERAGE_QUEUE_SIZE);
	//Serial.print("]: ");
	//Serial.println(co2volts2, 3);


	//lcd.clear();
	//lcd.autoscroll();
	lcd.setCursor(0, 0);
	//String lt = "Temp: ";
	//lt += t;
	//lcd.clear();

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
		w += lcd.print(bmpPressure / 133.3, 1);
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

	/*
	lcd.setCursor(0, 2);
	lcd.print("MG811: ");
	lcd.print(co2volts2, 3);
	lcd.print(" mV    ");

	lcd.setCursor(0, 3);
	auto pir = String();
	pir.reserve(DISPLAY_WIDTH);
	pir += "PIR: ";
	pir += pirState == 0 ? "off" : "on";
	//lcd.print("PIR: ");
	//lcd.print(pirState == 0 ? "off                 " : "on                  ");
	lcd.print(pir.c_str());
	*/
	//if (micros() / 1000 - ms > 1000 || ms > micros() / 1000) {
	//Serial.print("Free heap: ");
	//Serial.print(ESP.getFreeHeap());
	//Serial.println(" bytes");

	//if (heap > ESP.getFreeHeap()) {
	if (ESP.getFreeHeap() < 1000) {
		Serial.println("Detect mem leak!");

		lcd.clear();
		lcd.print("Memory Leak !!!");
		delay(3000);
		restart();
		//heap = ESP.getFreeHeap();
	}
	//ms = micros() / 1000;
	//}

	//heap = ESP.getFreeHeap();
}

void loop()
{
	runner.execute();
}


/*if (micros() / 1000 - ms > 1000 || ms > micros() / 1000) {
		//Serial.print("HEAP: ");
		//Serial.println(ESP.getFreeHeap());
		if (heap > ESP.getFreeHeap()) {
		Serial.println("Detect mem leak!");
		heap = ESP.getFreeHeap();
		}
		ms = micros() / 1000;
		++secondsSinceStart;
		}
		if (seconds % 10 == 0 && lastSend1 != seconds) {
		lastSend1 = seconds;

		//seconds = 0;
		++counter;
		String payload = "{\"micros\":";
		payload += ms;
		payload += ",\"counter\":";
		payload += counter;
		payload += "}";


		if (client.connected()) {
		Serial.print("Sending payload: ");
		Serial.print(payload);
		if (client.publish(pubTopic, (char*) payload.c_str())) {
		Serial.println(" Publish OK");
		} else {
		restart();
		}
		} else {
		restart();
		}
		}

		if (secondsSinceStart % 60 == 0 && lastSend2 != secondsSinceStart) {
		lastSend2 = secondsSinceStart;
		proceedTemperature();
		}//*/

void MQPinger()
{
	if (!client.loop())
	{
		Serial.println("client.loop() failed");

		if (!client.connected())
		{
			Serial.print("client.connected() failed, state: ");
			Serial.println(client.state());

			Serial.println("Connecting to MQTT broker...");
			//Serial.println("!client.loop()");

			if (client.connect(mqtt_client, mqtt_user, mqtt_pass))
			{
				Serial.println("Connected to MQTT broker");
				client.subscribe(controlTopic.c_str());


				//String payload = "{";
				//payload += "\"Online\":"; payload += "true";
				////payload += ",\"Temperature\":"; payload += t;
				////payload += ",\"HeatIndex\":"; payload += hic;
				//payload += "}";

				//Serial.print("Sending payload: ");
				//Serial.print(payload);
				//if (client.publish(pubTopic2.c_str(), payload.c_str())) {
				//	Serial.println(" - Publish OK");
				//}
				//else {
				//	//restart();
				//}

			}
		}
		//restart();
	}


}