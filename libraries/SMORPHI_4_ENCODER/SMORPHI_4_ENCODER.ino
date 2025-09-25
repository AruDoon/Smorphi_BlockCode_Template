#include <ESP32Encoder.h>
ESP32Encoder encoder1, encoder2, encoder3, encoder4;

// #define ENCODER_FR_A 19
// #define ENCODER_FR_B 18
// #define ENCODER_FL_A 16
// #define ENCODER_FL_B 17

// #define ENCODER_RR_A 25
// #define ENCODER_RR_B 23
// #define ENCODER_RL_A 26
// #define ENCODER_RL_B 27


#define ENCODER_FL_A 4
#define ENCODER_FL_B 5
#define ENCODER_FR_A 19
#define ENCODER_FR_B 18
#define ENCODER_RR_A 25
#define ENCODER_RR_B 23
#define ENCODER_RL_A 26
#define ENCODER_RL_B 27


void setup(){
	
	Serial.begin(115200);
	// Enable the weak pull down resistors

	//ESP32Encoder::useInternalWeakPullResistors = puType::down;
	// Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors = puType::up;

	// use pin 19 and 18 for the first encoder
	encoder1.attachHalfQuad(ENCODER_FL_A, ENCODER_FL_B);
	// use pin 17 and 16 for the second encoder
	encoder2.attachHalfQuad(ENCODER_FR_A, ENCODER_FR_B);
	encoder3.attachHalfQuad(ENCODER_RR_A, ENCODER_RR_B);
	encoder4.attachHalfQuad(ENCODER_RL_A, ENCODER_RL_B);

		
	// set starting count value after attaching
	// encoder1.setCount(0);
	// encoder2.setCount(0);

	// clear the encoder's raw count and set the tracked count to zero
	encoder1.clearCount();
	encoder2.clearCount();
	encoder3.clearCount();
	encoder4.clearCount();
	// set the lastToggle
}

void loop(){
	// Loop and read the count
	Serial.println("Encoder count = " + String((int32_t)encoder1.getCount()) + " " + String((int32_t)encoder2.getCount()) + " " + String((int32_t)encoder3.getCount()) + " " + String((int32_t)encoder4.getCount()));
	delay(100);

	// // every 5 seconds toggle encoder 2
	// if (millis() - encoder2lastToggled >= 5000) {
	// 	if(encoder2Paused) {
	// 		Serial.println("Resuming Encoder 2");
	// 		encoder2.resumeCount();
	// 	} else {
	// 		Serial.println("Paused Encoder 2");
	// 		encoder2.pauseCount();
	// 	}

	// 	encoder2Paused = !encoder2Paused;
	// 	encoder2lastToggled = millis();
	// }
}

