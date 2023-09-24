#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// constants
const int  reedPin = 14;    // the pin that the pushbutton is attached to
const int ledPin = 2;       // the pin that the LED is attached to
const char* ssid = "WiFi";
const char* password = "";
#define MAX_SRV_CLIENTS 1;  // how many clients should be able to telnet
const int port = 23;        // telnet server port
const long reedInterval = 5000; // 5 Seconds
const char* mqtt_server = "broker.hivemq.com";

// variables
int reedCounter = 0;   // counter for the number of button presses
int reedState = 0;         // current state of the button
int lastReedState = 0;     // previous state of the button
unsigned long reedPreviousMillis = 0;
bool lastMeasurementRetrieved = false;
double initialMeterReading = 0.0;

// telnet server
WiFiServer server(port);
WiFiClient wifi_1;

// mqtt client
WiFiClient wifi_2;
PubSubClient mqtt_client(wifi_2);
long lastMsg = 0;
char msg[50];
int value = 0;

/**
 * Class to handle serial and telnet printing
 */
class DualPrint : public Print
{
  public:
    DualPrint() : use_Serial(false), use_Telnet(false) {} // Initialization list
    virtual size_t write(uint8_t c) {
      if (use_Serial) Serial.write(c);
      if (use_Telnet) {
        if (!wifi_1.connected()) {
          // try to connect to a new client
          wifi_1 = server.available();
        } else {
          // write data to the connected client
          if (wifi_1.available() > 0) {
            wifi_1.write(c);
          }
        }
      }
      return 1;
    }
    bool use_Serial, use_Telnet;
} out;

void setup() {
  out.use_Serial = true;
  out.use_Telnet = true;
  Serial.begin(115200);
  out.println("Booting");
  pinMode(reedPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    out.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      out.println("Start updating " + type);
    })
    .onEnd([]() {
      out.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      out.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      out.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) out.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) out.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) out.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) out.println("Receive Failed");
      else if (error == OTA_END_ERROR) out.println("End Failed");
    });

  ArduinoOTA.begin();

  out.println("Ready");
  out.print("IP address: ");
  out.println(WiFi.localIP());

  server.begin(); // start telnet

  // MQTT
  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.setCallback(callback);
}

// MQTT on message recieved
void callback(char* topic, byte* message, unsigned int length) {
  out.print("Message arrived on topic: ");
  out.print(topic);
  out.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    out.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  out.println();

  // Do someting with the MQTT message
  if(String(topic) == "esp32/gasVolume") {
    if (!lastMeasurementRetrieved) {
      double total = messageTemp.toDouble() + (reedCounter * 0.01);
      if (total > initialMeterReading) {
        out.print("Setting total consumption to ");
        out.println(total);
        initialMeterReading = total;
      }
      lastMeasurementRetrieved = true;
      mqtt_client.unsubscribe("esp32/gasVolume");
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqtt_client.connected()) {
    out.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt_client.connect("ESP32Client509")) {
      out.println("connected");
      // Subscribe
      if(!lastMeasurementRetrieved) {
        mqtt_client.subscribe("esp32/gasVolume");
      }
    } else {
      out.print("failed, rc=");
      out.print(mqtt_client.state());
      out.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop() {
  ArduinoOTA.handle();

  unsigned long currentMillis = millis();

  // MQTT
  if (!mqtt_client.connected()) {
    reconnect();
  }
  mqtt_client.loop();

  // read the reed input pin:
  reedState = digitalRead(reedPin);

  // compare the reedState to its previous state
  if (reedState != lastReedState) {
    // if the state has changed, increment the counter
    if (reedState == HIGH) {
      // if the current state is HIGH then the button went from off to on:
      reedCounter++;
      out.println("open");
      out.print("number of rotations (0.01m3): ");
      out.println(reedCounter);
      out.print("total gas consumption (m3): ");
      double totalConsumption = initialMeterReading + (reedCounter * 0.01);
      out.println(totalConsumption);

      // Convert the value to a char array
      char tempString[9];
      dtostrf(totalConsumption, 3, 2, tempString);
      out.println(tempString);

      if(lastMeasurementRetrieved) {
        mqtt_client.publish("esp32/gasVolume", tempString, true);
      }
      digitalWrite(ledPin, LOW);
      
    } else {
      // if the current state is LOW then the button went from on to off:
      out.println("closed");
      digitalWrite(ledPin, HIGH);
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastReedState = reedState;

  // display reed current state at regular intervals
  if (currentMillis - reedPreviousMillis >= reedInterval) {
    reedPreviousMillis = currentMillis;
    if (reedState == HIGH) {
      out.println("open");
    } else {
      out.println("closed");
    }
  }
}
