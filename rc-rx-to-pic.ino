/* Written by Shufeng Tan <shufengtan@gmail.com> */

/* The protocol used by the Flight Simulator is a simple serial protocol and it's running at 9600 Baud.
Byte0: Sync - 0xF0 + #of Channels to be Transmitted
Byte1:Switch Position, since I'm not using any I just send 0x00
Byte2: Channel1 - linear, 100 for 1mS 200 for 2mS
Byte3: Channel2 - linear, 100 for 1mS 200 for 2mS
Byte4: Channel3 - linear, 100 for 1mS 200 for 2mS
Byte5: Channel4 - linear, 100 for 1mS 200 for 2mS

Each channel decoded by receiver is sent to ESCs and Servos as a train of pulses, these pulses are sent
about 50 times a second but the suprising part is, each pulse only lasts between one and two milliseconds
(1/1000 to 2/1000 of a second). Before the next pulse arrives, there is a gap of 10 times the length of
the even longest pulse. 

Arduino Mega external interrupts: 2, 3, 21, 20, 19, 18
RX channels need to connect to pins in the order above.
*/

#define DEBUG 0
#define NUM_OF_CHANNELS 4

// This is specific to Arduino Mega 2560
uint8_t interrupt_pin[] = {2, 3, 21, 20, 19, 18};

uint8_t flag_bit[] = {1, 2, 4, 8, 16, 32};
volatile uint16_t start[NUM_OF_CHANNELS];
volatile uint16_t input[NUM_OF_CHANNELS];
volatile uint8_t update_flag = 0;
#if DEBUG
volatile uint16_t prev_us;
#endif

byte control_byte0;
byte control_byte1;

void setup() {
  Serial.begin(9600);
#if DEBUG
  Serial.print(NUM_OF_CHANNELS);
  Serial.println(" channels");
#endif

control_byte0 = (byte)(0xf0 + NUM_OF_CHANNELS);
control_byte1 = (byte)0x00;

  attachInterrupt(0, calc_ch1, CHANGE);
#if NUM_OF_CHANNELS > 1
  attachInterrupt(1, calc_ch2, CHANGE);
#endif
#if NUM_OF_CHANNELS > 2
  attachInterrupt(2, calc_ch3, CHANGE);
#endif
#if NUM_OF_CHANNELS > 3
  attachInterrupt(3, calc_ch4, CHANGE);
#endif
#if NUM_OF_CHANNELS > 4
  attachInterrupt(4, calc_ch5, CHANGE);
#endif
#if NUM_OF_CHANNELS > 5
  attachInterrupt(5, calc_ch6, CHANGE);
#endif

#if DEBUG
  prev_us = micros();
#endif
}

void loop() {
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that their values will be retained
  // between calls to loop.
  static uint16_t this_input[NUM_OF_CHANNELS];
  // local copy of update flags
  static uint8_t this_update_flag;

  if (update_flag) {
    noInterrupts();
    this_update_flag = update_flag;
    for (uint8_t i = 0; i < NUM_OF_CHANNELS; i++) {
      if(update_flag & flag_bit[i]) {
        this_input[i] = input[i];
      }
    }
    update_flag = 0;
    interrupts();
  }
 
#if DEBUG
  for (uint8_t i = 0; i < NUM_OF_CHANNELS; i++) {
    if(this_update_flag & flag_bit[i]) {
      Serial.print(i);
      Serial.print(": ");
      Serial.print(input[i]);
      Serial.print(" ");
    }
  }

  uint16_t new_us = micros();
  uint16_t dt = new_us - prev_us;
  prev_us = new_us;
  Serial.println(dt);
#else
  send_data(this_input);
#endif
 
  this_update_flag = 0;
}

void send_data(uint16_t *this_input) {
  noInterrupts();
  Serial.write(control_byte0);
  Serial.write(control_byte1);
  for(uint8_t i = 0; i < NUM_OF_CHANNELS; i++) {
    Serial.write((byte)(this_input[i]/10));
  }
  interrupts();
}

void calc(int interrupt) {
  if (digitalRead(interrupt_pin[interrupt]) == HIGH) {
    start[interrupt] = micros();
  } else {
    input[interrupt] = (uint16_t)(micros() - start[interrupt]);
    update_flag |= flag_bit[interrupt];
  }
}

void calc_ch1() {
  calc(0);
}

#if NUM_OF_CHANNELS > 1
void calc_ch2() {
  calc(1);
}
#endif

#if NUM_OF_CHANNELS > 2
void calc_ch3() {
  calc(2);
}
#endif

#if NUM_OF_CHANNELS > 3
void calc_ch4() {
  calc(3);
}
#endif

#if NUM_OF_CHANNELS > 4
void calc_ch5() {
  calc(4);
}
#endif

#if NUM_OF_CHANNELS > 5
void calc_ch6() {
  calc(5);
}
#endif
