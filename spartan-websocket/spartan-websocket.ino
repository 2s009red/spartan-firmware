#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>

#include "Wire.h"
#include <MPU6050_light.h>


#include "MotionPlanner.h"

WebSocketsClient webSocket;

#define USE_SERIAL Serial

//const int WITHDRAW_DELAY = 200;

const int MAX_EXTENSION = 180;

int extension = MAX_EXTENSION;  // degrees of furthest extension

uint32_t spar_delay_minimum = 2000;  // delay, in ms, between punches
uint32_t spar_delay_maximum = 4000;  // delay, in ms, between punches

bool sparring = false;  // are we sparring right now?

uint32_t last_punch_millis;

const uint8_t SERVO_PIN = 14;
const uint32_t SERVO_FREQUENCY = 333;

uint32_t current_strike_delay;

bool extended = false;

MPU6050 mpu(Wire);
MotionPlanner motionPlanner;

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
	switch(type) {
		case WStype_TEXT:
            switch(payload[0]) {
                case 'p':
                  motionPlanner.clear();
                  motionPlanner.addMotion(millis(), extension);
                  motionPlanner.addMotion(millis() + WITHDRAW_DELAY, 0);

                  Serial.println("Punch");
                  Serial.println(extension);
                  last_punch_millis = millis();
                  break;
                case 's':   // TODO speed
                  Serial.println(atoi((const char*) payload + 1));
                  break;
                case 'e':   // extension
                  // TODO we should have this update motionPlanner; right now, it only reads these values once, during startSparring
                  extension = atoi((const char*) payload + 1);
                  break;
                case 'f':   // frequency
                  if (payload[1] == 'n') {  // miN
                    spar_delay_minimum = atoi((const char*) payload + 2);
                  } else if (payload[1] == 'x') {   // maX
                    spar_delay_maximum = atoi((const char*) payload + 2);
                  }
                  break;
                case 'r':
                  // Reset or pReamble--stop all punches, reset to 0, and get ready to start sparring mode
                  break;
                case '0':
                  motionPlanner.stopSparring();
                  motionPlanner.clearAndReturnToZero();
                  break;
                case '1':
                // reset to 0
                  motionPlanner.clearAndReturnToZero();
                  motionPlanner.startSparring(spar_delay_minimum, spar_delay_maximum, extension);
                  break;
                case '2':
                  motionPlanner.addMotion(millis(), extension);
                  break;
                case '3':
                  motionPlanner.clear();
                  break;
            }
			break;
		case WStype_BIN:
		case WStype_ERROR:			
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
    case WStype_DISCONNECTED:
    case WStype_CONNECTED:
			break;
	}
}


void setup() {
    pinMode(SERVO_PIN, OUTPUT);
    
	  Serial.begin(115200);
	  Serial.setDebugOutput(true);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    int status = mpu.begin();

    if (status) ESP.restart(); // couldn't connect to MPU

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true); // gyro and accelero
 
    WiFi.mode(WIFI_STA);
    WiFi.begin("MIT", "");

    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
  	}

    Serial.println("Connected to WiFi");

  	webSocket.begin("35.173.125.185", 80, "/");
  	webSocket.onEvent(webSocketEvent);
  	webSocket.setReconnectInterval(5000);

    // set up PWM
    analogWriteFreq(SERVO_FREQUENCY);

    motionPlanner.clear();
    motionPlanner.attachServo(SERVO_PIN);
}

void loop() {
    webSocket.loop();
//    const uint32_t SPEED_DELAY = 20; // smear withdraw over this time period so we get slower motion

    motionPlanner.update();
}
