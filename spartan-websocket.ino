/*
 * WebSocketClient.ino
 *
 *  Created on: 24.05.2015
 *
 */

#include <Arduino.h>
#include <WiFiMulti.h>
#include <WebSocketsClient.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

WiFiMulti WiFiMulti;
WebSocketsClient webSocket;

#define USE_SERIAL Serial

const int WITHDRAW_DELAY = 200;

// TODO change this?
const int MAX_EXTENSION = 180;

int extension = 180;  // degrees of furthest extension
//int frequency = 1000;  // delay, in ms, between punches
uint32_t spar_delay_minimum = 2000;  // delay, in ms, between punches
uint32_t spar_delay_maximum = 4000;  // delay, in ms, between punches

bool sparring = false;  // are we sparring right now?
uint32_t last_punch_millis;

const uint8_t SERVO_PIN = 17;

uint32_t current_strike_delay;

bool extended = false;

// TODO clean this file up

MPU6050 accelerometer;

/*
struct MotionPlan {
    uint32_t destination,
    uint32_t timestamp
};

// TODO enforce singleton
class MotionController {
  private:
    bool isSparring;
    uint32_t motionPlanLength;
    MotionPlan motions[128];

  public:
    MotionController() {
        this->isSparring = false;
    }

    void updateMotion() {
        
    }

    void addMotion() {
        
    }
} motionController;
*/

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

	switch(type) {
		case WStype_DISCONNECTED:
			USE_SERIAL.printf("[WSc] Disconnected!\n");
			break;
		case WStype_CONNECTED:
			USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);

			// send message to server when Connected
			webSocket.sendTXT("Connected");
            
			break;
		case WStype_TEXT:
            switch(payload[0]) {
                case 'p':
                  servo_write(extension);
                  Serial.println("Punch");
                  Serial.println(extension);
                  last_punch_millis = millis();
                  break;
                case 's':
                  Serial.println(atoi((const char*) payload + 1));
                  break;
                case 'e':   // extension
                  extension = atoi((const char*) payload + 1);
//                  sscanf(payload + 1, "%f", 
                  break;
                case 'f':   // frequency
                  if (payload[1] == 'n') {  // miN
                    spar_delay_minimum = atoi((const char*) payload + 2);
                  } else if (payload[1] == 'x') {   // maX
                    spar_delay_maximum = atoi((const char*) payload + 2);
                  }
//                  frequency = atoi((const char*) payload + 1);
                  break;
                  // use '1' and '0' to start sparring
                case 'r':
                  // Reset or pReamble--stop all punches, reset to 0, and get ready to start sparring mode
//                  servo_wri
                  break;
                case '1':
                // reset to 0
                  servo_write(0);
                  last_punch_millis = millis();
                  sparring = true;
                  current_strike_delay = random(spar_delay_minimum, spar_delay_maximum);
                  break;
                case '0':
                  servo_write(0);
                  sparring = false;
                  last_punch_millis = 0;    // TODO kinda a hack
                  break;
                case '2':
                  extended = true;
                  break;
                case '3':
                  extended = false;
                  servo_write(0);
                  break;
            }
			// send message to server
			// webSocket.sendTXT("message here");
			break;
		case WStype_BIN:
			USE_SERIAL.printf("[WSc] get binary length: %u\n", length);

			// send data to server
			// webSocket.sendBIN(payload, length);
			break;
		case WStype_ERROR:			
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
	}

}


void setup() {
    pinMode(SERVO_PIN, OUTPUT);
    pinMode(2, OUTPUT);
//    pinMode(1, OUTPUT);
    
	USE_SERIAL.begin(115200);

	USE_SERIAL.setDebugOutput(true);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    accelerometer.initialize();

    if (accelerometer.testConnection()) {
        Serial.println("good");
    }


	for(uint8_t t = 2; t > 0; t--) {
		USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
		USE_SERIAL.flush();
		delay(1000);
	}
   
	WiFiMulti.addAP("MIT GUEST", "");

	while(WiFiMulti.run() != WL_CONNECTED) {
		delay(100);
	}

   USE_SERIAL.println("Connected to WiFi");

	// server address, port and URL
	webSocket.begin("35.173.125.185", 80, "/");

	// event handler
	webSocket.onEvent(webSocketEvent);

	// try ever 5000 again if connection has failed
	webSocket.setReconnectInterval(5000);

//    digitalWrite(SERVO_PIN, LOW);

// set up PWM
    const int freq = 333;
    const int ledChannel = 0;
    const int resolution = 10;

//    ledcSetup(ledChannel, freq, resolution);
//    ledcAttachPin(SERVO_PIN, ledChannel);
}

int last_move_millis = 0;

// TODO this is horrible. why do we even have this
void servo_write(int pos) {
//    Serial.println(pos);
    // TODO delete this later
    int delay = map(pos, 0, 180, 2050, 1480);
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(delay);
    digitalWrite(SERVO_PIN, LOW);
    
    
//    int pwm_value = map(pos, 0, 180, 2796, 2018);

//    int pwm_value = map(pos, 0, 180, 699, 504);
//    ledcWrite(0, pwm_value);
}

// TODO make a motion controller class
// allow user to schedule a punch
// allows user to cancel all scheduled punches
void loop() {
    webSocket.loop();
    
//    digitalWrite(2, HIGH);
//    digitalWrite(1, HIGH);
    
    // this is a terrible hack
//    uint32_t milliseconds = millis();

    uint32_t milliseconds_passed = millis() - last_punch_millis;
// TODO TODO TODO add function that keeps the arm out
    const uint32_t SPEED_DELAY = 20; // smear withdraw over this time period so we get slower motion

    if (extended) {
        servo_write(180);
    } else if (!sparring) {
        if (milliseconds_passed < WITHDRAW_DELAY) {
            servo_write(extension);
         } else if (milliseconds_passed < WITHDRAW_DELAY + SPEED_DELAY) {   
            // FIXME wtf this expression
            servo_write(extension - extension * (milliseconds_passed - WITHDRAW_DELAY) / SPEED_DELAY);
        } else {
            servo_write(0);
        }
    } else {
        if (milliseconds_passed < WITHDRAW_DELAY) {
            servo_write(extension);
        } else if (milliseconds_passed < WITHDRAW_DELAY + current_strike_delay) {
            servo_write(extension - extension * (milliseconds_passed - WITHDRAW_DELAY) / SPEED_DELAY);
        } else {
            servo_write(0);
            last_punch_millis = millis();
            current_strike_delay = random(spar_delay_minimum, spar_delay_maximum);
        }
    }
}
