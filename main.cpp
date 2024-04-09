/*
	Embedded Project: Gesture Lock System
	Students: Minh Quan Nham, Olutamilore Sojinrin

	Usage:
	- This program uses the two buttons on the Circuit Playground
	  board to navigate - the LEFTBUTTON (LB) and the RIGHTBUTTON
	  (RB).
	- When board is reset, the board is in the UNLOCKED state
	  (solid green).
	- Clicking RB in UNLOCKED state will begin the password
	  RECORDING state (loading green), and any motion during this
	  will be recorded as part of the gesture. Clicking RB again
	  will end the RECORDING state and return to UNLOCKED state.
	- Clicking LB in UNLOCKED state will change state to the
	  LOCKED state (solid red).
	- Clicking RB in LOCKED state will begin the password
	  ENTERING state (loading red), and any motion will be
	  interpreted as part of the gesture password. Clicking RB
	  again will end the ENTERING state. If the password matches
	  the saved password, the board will move to UNLOCKED state.
	  Otherwise it will return to LOCKED state.
	- During ENTERING, RECORDING and LOCKED the LB is blocked,
	  and clicking it will show a blinking error on that side.

	Tuning:
	- The project uses DTW to compare passwords, and compares
	  both magnitudes of the acceleration vector as well as the
	  angle between the vector and the x-axis. The larger the
	  DTW score is, the larger the differences between the saved
	  and entered passwords are.
	- The constants ACCEL_WEIGHT and ANGLE_WEIGHT determine the
	  multipliers for the acceleration magnitude and the angle,
	  and their scores are added together and compared with
	  SCORE_THRESH. The password is deemed correct if the total
	  score is < SCORE_THRESH. These constants can be changed
	  (increasing SCORE_THRESH, decreasing ACCEL_WEIGHT and
	  ANGLE_WEIGHT) to make the comparison more lenient.
	- The LOG_ENABLED is defined to print the states and the DTW
	  scores to the Serial monitor.
*/

#include <Arduino.h>
#include <SPI.h>
#include <LightNeoPixels.h>

// Sampling constants
#define DEFAULT_FREQ 50
#define RESAMPLE_FACTOR 0.75
#define AVG_WINDOW 5

// DTW constants
#define MAX_SIZE 28
#define DTW_WINDOW 4

// Scoring constants
#define SCORE_THRESH 0.8
#define ACCEL_WEIGHT 0.5
#define ANGLE_WEIGHT 1.6

// Serial logging
#define LOG_ENABLED

// SPI read function
// Read "length" bytes into output location
void SPIread(uint8_t cs, uint8_t addr, uint8_t length, void* output) {
	uint8_t* output_int = (uint8_t*) output;
	digitalWrite(cs, LOW);						// Assert CS
	SPI.transfer(0xC0 | (addr & 0x3F));			// Set RW and MS
	for (uint8_t i = 0; i < length; i++)
		*(output_int++) = SPI.transfer(0x00);
	digitalWrite(cs, HIGH);						// Deassert CS
}

// SPI write function
// Write "length" bytes from input location
void SPIwrite(uint8_t cs, uint8_t addr, uint8_t length, const void* input) {
	uint8_t* input_int = (uint8_t*) input;
	digitalWrite(cs, LOW);					// Assert CS
	SPI.transfer(0x40 | (addr & 0x3F));		// Unset RW, set MS
	for (uint8_t i = 0; i < length; i++)
		SPI.transfer(*(input_int++));
	digitalWrite(cs, HIGH);					// Deassert CS
}

// Vector3Int struct
struct Vector3Int {
	int16_t xyz[3] = {};
	Vector3Int(int16_t x = 0, int16_t y = 0, int16_t z = 0) {
		xyz[0] = x; xyz[1] = y; xyz[2] = z;
	}
	int32_t mag_sqr() const {
		return xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2];
	}
	double mag() const {return sqrt(mag_sqr());}
	int32_t dot(const Vector3Int& other) const {
		return
			xyz[0] * other.xyz[0] +
			xyz[1] * other.xyz[1] +
			xyz[2] * other.xyz[2];
	}
};

// Windowed dynamic time warping courtesy of:
// https://en.wikipedia.org/wiki/Dynamic_time_warping
double DTW(
	double* source, int16_t source_size,
	double* target, int16_t target_size
) {
	static double DTW_cost[MAX_SIZE][DTW_WINDOW * 2];
	for (int16_t i = 0; i < source_size; i++)
		for (int16_t j = 0; j < DTW_WINDOW * 2; j++)
			DTW_cost[i][j] = __DBL_MAX__;
			
	DTW_cost[0][0] = 0;
	// for (int16_t i = 1; i < source_size; i++)
	// 	for (int16_t j = max(1, i - DTW_WINDOW);
	// 		j < min(target_size, i + DTW_WINDOW); j++)
	// 		DTW_cost[i][j] = 0;
	
	for (int16_t i = 1; i < source_size; i++)
		for (int16_t j = max(1, i - DTW_WINDOW);
			j < min(target_size, i + DTW_WINDOW); j++) {
			int16_t j_p = j - max(1, i - DTW_WINDOW) + 1;
			double cost = abs(source[i] - target[j]);
			DTW_cost[i][j_p] = cost + min(
				min(DTW_cost[i - 1][j_p], DTW_cost[i][j_p - 1]),
				DTW_cost[i - 1][j_p - 1]
			);
		}
	return DTW_cost[source_size - 1][
		min(target_size, source_size - 1 + DTW_WINDOW) -
		max(1, source_size - 1 - DTW_WINDOW)
	];
}

// Sequence struct
struct Sequence {
	static const int16_t SEQ_MAX_SIZE = MAX_SIZE * 2;
	double data[SEQ_MAX_SIZE] = {};
	int16_t length = 0;
	void reset() {length = 0;}
	uint8_t write(double value) {
		if (length >= SEQ_MAX_SIZE) return 0;
		data[length++] = value;
		return 1;
	}
	uint8_t resample(double factor) {
		static double buffer[SEQ_MAX_SIZE] = {};
		int16_t i = 0;
		while (i < length) {
			buffer[i] = data[i];
			i++;
		}
		i = 0;
		while (i < min(((int) length - 1) * factor + __DBL_EPSILON__,
			SEQ_MAX_SIZE)) {
			double lower = floor(i / factor),
				fraction = i / factor - lower;
			data[i] = buffer[(int16_t) lower] * (1 - fraction) +
				buffer[(int16_t) lower + 1] * fraction;
			i++;
		}
		length = i;
		return 1;
	}
	uint8_t shiftleft(int16_t amount) {
		if (amount <= 0) return 0;
		if (amount >= length) {
			length = 0;
			return 1;
		}
		for (int16_t i = 0; i < length - amount; i++)
			data[i] = data[i + amount];
		length = length - amount;
		return 1;
	}
};

void setup() {
	// Setup buttons
	// Set PD4 & PF6 to be input pins with no pullup resistors
	DDRD &= ~(1 << 4); PORTD &= ~(1 << 4);
	DDRF &= ~(1 << 6); PORTF &= ~(1 << 6);

	// Setup SPI
	SPI.begin();
	SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
	pinMode(8, OUTPUT);						// Enable CS

	// Setup accelerometer
	SPIwrite(8, 0x20, 1,	// CTRL_REG1 (0x20)
		&(const uint8_t&) 0b01110111);
	/* 						xxxx 400Hz
								x Normal mode
								 xxx Enable all 3 axes
	*/
	SPIwrite(8, 0x23, 1,	// CTRL_REG4 (0x23)
		&(const uint8_t&) 0b00110000);
	/*						xx Default
							  xx Full range +-16g selection
								x Normal mode
								 xxx Default
	*/

	// Setup NeoPixels
	DDRB |= 1;

#ifdef LOG_ENABLED
	Serial.begin(9600);
#endif
}

// State transition function
// Available states:
// 0x00: Locked		0x01: Entering pw
// 0x10: Unlocked	0x11: Saving new pw
uint8_t state_transition(uint8_t* state, uint8_t button,
	uint8_t optional_condition = 1) {
	// No button: no state change
	if (!button) return 1;

	switch (*state) {
		case 0x00: {
			if (button & 2) return 0;
			*state = 0x01;
			return 1;
		}
		case 0x01: {
			if (button & 2) return 0;			
			if (optional_condition) *state = 0x10;
			else *state = 0x00;
			return 1;
		}
		case 0x10: {
			if (button & 2) *state = 0x00;
			else *state = 0x11;
			return 1;
		}
		case 0x11: {
			if (button & 2) return 0;
			*state = 0x10;
			return 1;
		}
		default:
			return 0;
	}
}

// NeoPixels properties & set color functions
uint8_t strip_length = 10;
uint8_t strip_rgb[3][10] = {};
inline void set_color(
	uint8_t loc, uint32_t color,
	uint8_t brightness = 30
) {
	float factor = ((float) brightness) / 255;
	for (int8_t i = 2; i >= 0; i--) {
		strip_rgb[i][loc] = (uint8_t) ((color & 0xFF) * factor);
		color >>= 8;
	}
}
inline void set_color(
	uint8_t loc, uint8_t r, uint8_t g, uint8_t b,
	uint8_t brightness = 30
) {
	float factor = ((float) brightness) / 255;
	strip_rgb[0][loc] = (uint8_t) (r * factor);
	strip_rgb[1][loc] = (uint8_t) (g * factor);
	strip_rgb[2][loc] = (uint8_t) (b * factor);
}

// NeoPixels update function
uint32_t elapsed_time[2] = {};
uint32_t previous_millis = 0, delta = 0;
int16_t counter[2] = {};
void update_leds(uint8_t state, uint8_t error = 0) {
	if (!error) counter[1] = 0;
	delta = millis() - previous_millis;
	previous_millis += delta;
	elapsed_time[0] += delta;
	elapsed_time[1] += delta;

	switch (state) {
		// Static color for locked/unlocked states
		case 0x00: case 0x10: {
			if (elapsed_time[0] > 20) {
				elapsed_time[0] = 0;
				uint32_t color = (uint32_t) 0xFF << ((0x20 - state) >> 1);
				for (int i = 0; i < 10; i++)
					set_color(i, color);
				send_strip(
					strip_rgb[0], strip_rgb[1],
					strip_rgb[2], strip_length
				);
			}
			break;
		}

		// Loading animation for entering/saving password
		case 0x01: case 0x11: {
			for (int i = 0; i < 5; i++) {
				uint32_t color = (uint32_t) max(
					252 - i * 63 * (1 + 0.3 *
					abs((counter[0] >> 1) - 15)) + 3,
					0
				) << ((0x21 - state) >> 1);
				int current = (counter[0] % 10) - i;
				if (current < 0) current += 10;
				set_color(current, color);
				current = (counter[0] % 10) + i;
				if (current > 9) current -= 10;
				set_color(current, color);
				send_strip(
					strip_rgb[0], strip_rgb[1],
					strip_rgb[2], strip_length
				);
			}
			if (elapsed_time[0] > 50) {
				elapsed_time[0] = 0;
				if (++counter[0] >= 60) counter[0] = 0;
			}
			break;
		}
		default:
			break;
	}
	
	// Flashing warning on error side
	if (error) {
		for (int i = (2 - error) * 5; i < (3 - error) * 5; i++)
			set_color(i, (counter[1] & 1)? 0: 255, 0, 0);
		send_strip(
			strip_rgb[0], strip_rgb[1],
			strip_rgb[2], strip_length
		);
		if (elapsed_time[1] > 80) {
			elapsed_time[1] = 0;
			if (++counter[1] == 100) counter[1] = 0;
		}
	}
}

// Acceleration vector
Vector3Int accel;
// Button states
// LEFTBUTTON (PD4) bit 1, RIGHBUTTON (PF6) bit 0
uint8_t current = 0, previous,
		state = 0x10, error = 0;

// Passwords, 0: entered password, 1: saved password
Sequence pw_accel[2], pw_angle[2];

// Averager
double sum_accel = 0, sum_angle = 0;
int8_t avg_counter = 0;

// Recording variables
double frequency;
uint64_t start_time, error_time;

void loop() {
	start_time = micros();
	previous = current;
	current = ((PIND >> 3) & 2) | ((PINF >> 6) & 1);
	// Detect rising edge
	if (current & ~previous) {
		// If two buttons at same time:
		// LEFTBUTTON has higher precedence than RIGHBUTTON
		uint8_t button = current & ~(current >> 1);

#ifdef LOG_ENABLED
		// Right button: bit 1 enabled
		if (button & 2) Serial.println("Right clicked!");
		// Left button: bit 0 enabled
		else Serial.println("Left clicked!");
#endif

		// Resample to DTW maximum size after recording
		uint8_t condition = 1;
		if (
			button == 1 &&
			(state == 0x01 || state == 0x11)
		) {
			double factor = (double) MAX_SIZE /
				pw_accel[state >> 4].length;
			pw_accel[state >> 4].resample(factor);
			pw_angle[state >> 4].resample(factor);

			// Calculate condition if in login mode
			if (state == 0x01) {
				double score[2] = {
					DTW(
						pw_accel[1].data, pw_accel[1].length,
						pw_accel[0].data, pw_accel[0].length
					) / MAX_SIZE * ACCEL_WEIGHT,
					DTW(
						pw_angle[1].data, pw_angle[1].length,
						pw_angle[0].data, pw_angle[0].length
					) / MAX_SIZE * ANGLE_WEIGHT
				};
				condition = (score[0] + score[1] < SCORE_THRESH);

#ifdef LOG_ENABLED
				Serial.print("Accel DTW difference: ");
				Serial.println(score[0]);
				Serial.print("Angle DTW difference: ");
				Serial.println(score[1]);
				Serial.print("Consensus: ");
				Serial.println(condition? "1": "0");
#endif
			}
		}

		// Check if state transition is successful
		if (state_transition(&state, button, condition)) {

#ifdef LOG_ENABLED
			Serial.print("New state: 0x");
			Serial.print(state >> 4);
			Serial.println(state & 0xF);
#endif

			// Check if recording states
			if (state & 0x0F) {
				pw_accel[state >> 4].reset();
				pw_angle[state >> 4].reset();
				if (state == 0x11) frequency = DEFAULT_FREQ;
			}
			error = 0;
		} else {

#ifdef LOG_ENABLED
			Serial.println("State transition failed!");
#endif

			error = button;
			error_time = micros();
		}
	}

	// Record if state > 0
	if (state & 0x0F) {
		SPIread(8, 0x28, 6, accel.xyz);
		for (int i = 0; i < 3; i++) accel.xyz[i] >>= 8;

		// Add sample to averaging sum
		sum_accel += accel.mag();
		if (abs(accel.mag()) > __DBL_EPSILON__)
			sum_angle += acos(accel.dot(Vector3Int(1, 0, 0)) / accel.mag());
		
		// Take average when window is reached
		if (++avg_counter == AVG_WINDOW) {
			pw_accel[state >> 4].write(sum_accel / AVG_WINDOW);
			pw_angle[state >> 4].write(sum_angle / AVG_WINDOW);
			sum_accel = 0; sum_angle = 0;
			avg_counter = 0;

			// Downsample if full when saving password 
			if (
				state == 0x11 &&
				pw_accel[state >> 4].length == Sequence::SEQ_MAX_SIZE
			) {
				pw_accel[state >> 4].resample(RESAMPLE_FACTOR);
				pw_angle[state >> 4].resample(RESAMPLE_FACTOR);
				frequency *= RESAMPLE_FACTOR;
			}
		}

		// Wait until next sample based on frequency
		while(micros() - start_time < 1.0e6 / (frequency * AVG_WINDOW)) {
			update_leds(state, error);
			if (error && micros() - error_time > 600e3) {
				error = 0;
				error_time = 0;
			}
		};
	}

	// Else nominal delay in standby
	else while(micros() - start_time < 5e3) {
		update_leds(state, error);
		if (error && micros() - error_time > 400e3) {
			error = 0;
			error_time = 0;
		}
	}
}
