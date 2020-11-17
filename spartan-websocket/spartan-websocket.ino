#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

WebSocketsClient webSocket;

#define USE_SERIAL Serial

const int WITHDRAW_DELAY = 200;

const int MAX_EXTENSION = 180;

int extension = 180;  // degrees of furthest extension
//int frequency = 1000;  // delay, in ms, between punches
uint32_t spar_delay_minimum = 2000;  // delay, in ms, between punches
uint32_t spar_delay_maximum = 4000;  // delay, in ms, between punches

bool sparring = false;  // are we sparring right now?
uint32_t last_punch_millis;

const uint8_t SERVO_PIN = 14;
const uint32_t SERVO_FREQUENCY = 333;

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
    accelerometer.initialize();

    if (accelerometer.testConnection()) {
        Serial.println("good");
    }
 
    WiFi.mode(WIFI_STA);
    WiFi.begin("MIT GUEST", "");

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
}

int last_move_millis = 0;

void servo_write(int pos) {
    uint32_t pwm_value = map(pos, 0, 180, 699, 504);
    analogWrite(SERVO_PIN, pwm_value);
}

// TODO make a motion controller class
// allow user to schedule a punch
// allows user to cancel all scheduled punches
void loop() {
    webSocket.loop();
    

    uint32_t milliseconds_passed = millis() - last_punch_millis;
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
