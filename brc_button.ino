// Read message from main controller
void readIncomingMessage(byte message_[], uint8_t * message_size_);
// Check incoming message structure
bool checkIncomingMessage(byte message_[], const uint8_t message_size_);
// Process correct incoming message
void processIncomingMessage(byte message_[], uint8_t * message_size_);
// Get message type by message fields
uint8_t getMessageType(byte message_[], const uint8_t message_size_);
// Calculate checksum of incoming message
byte calculateIncomingChecksum(byte message_[], const uint8_t message_size_);
// Calculate checksum of outgoing message
byte calculateOutgoingChecksum(byte message_[], const uint8_t message_size_);
// Send reply based on template to controller
void sendReply(byte message_template_[], uint8_t message_size_, bool calculate_checksum_, bool send_fuel_state_, bool send_button_state_);
// Prepair outgoing message (copy template to message)
void prepairOutgoingMessage(byte message_[], byte message_template_[], const uint8_t message_size_);
// Set button pressed state in outgoing message
void setButtonStateInMessage(byte message_[]);
// set fuel state in outgoing message
void setFuelStateInMessage(byte message_[]);
// Set checksum in outgoing message
void setOutgoingMessageChecksum(byte message_[], uint8_t message_size_);
// Send message to controller
void sendPrepairedMessage(byte message_[], uint8_t message_size_);
// Get fuel state from incoming message
void getStateFromMessage(byte message_[], const uint8_t message_size_);
// Get beep state from incoming message
void getBeepStateFromMessage(byte message_[], const uint8_t message_size_);
// Set leds and beep state from current state
void setState();
// Light leds
void lightLeds();
// When LPG out, blink red and blue leds
void lightLedsEmpty();
// When error, ON red led, OFF other leds
void lightLedsError();
// When changing fuel, blink green and blue leds
void lightLedsChangeFuel();
// Play beep
void playBeep();
// Read button state
void checkButton();
// Check, that init2 message recieved from controller
void checkInitCompleted();
// Check, that messages are comming from controller
void checkAppropriateMessageTimestamp();

#define INIT_TIMEOUT 10000
#define OUTGOING_MESSAGE_LENGTH 20
#define INCOMING_MESSAGE_SIZE 50
#define INCOMING_MESSAGE_RECIEVE_TIMEOUT 5
#define MINIMAL_MESSAGE_LENGTH 5


const byte outgoing_init1_message_template[] = {0x02, 0x30, 0x8B, 0x80, 0x80, 0x81, 0xFF, 0x35, 0x31, 0xA1, 0x03};
const byte outgoing_init2_message_template[] = {0x02, 0x30, 0xAE, 0x90, 0xF0, 0x83, 0xB0, 0xCC, 0xA3, 0x90, 0x90, 0x90, 0x90, 0xA0, 0xC3, 0x88, 0xA3, 0x03};
const byte outgoing_init3_message_template[] = {0x02, 0x30, 0x95, 0x80, 0x80, 0x81, 0x9F, 0xF3, 0x9F, 0xFB, 0x35, 0x31, 0x80, 0x86, 0xFF, 0x03};
const byte outgoing_state_message_template[] = {0x02, 0x30, 0x8D, 0x80, 0x80, 0x81, 0x9F, 0xFB, 0x9F, 0xFC, 0x81, 0x35, 0x31, 0x86, 0xFF, 0x03};

// DST and SRC is from KWP2000, but here it can be SRC vs DST or anything else.
#define MESSAGE_DST_BYTE_POSITION 1
#define MESSAGE_SRC_BYTE_POSITION 2
#define MESSAGE_FIRST_FUEL_BYTE_POSITION 3
#define MESSAGE_SECOND_FUEL_BYTE_POSITION 4
#define MESSAGE_BUTTON_BYTE_POSITION 5
#define MESSAGE_BEEP_BYTE_POSITION 8

#define CHECKSUM_POSITION_FROM_END_OF_MESSAGE 2

#define LPG_BIT 4
#define PETROL_BIT 5
#define EMPTY_BIT 6
#define BUTTON_BIT 0
#define BEEP_BIT 0

#define DST_INIT 0x31
#define DST_STATE 0x35
#define SRC_INIT1 0x89
#define SRC_INIT2 0xAA
#define SRC_INIT3 0x93
#define SRC_STATE 0x89

#define INIT1_MESSAGE_TYPE 1
#define INIT2_MESSAGE_TYPE 2
#define INIT3_MESSAGE_TYPE 3
#define STATE_MESSAGE_TYPE 4

#define BLUE_PIN 9
#define RED_PIN 6
#define GREEN_PIN 5
#define BUTTON_PIN 10
#define BEEP_PIN 3

#define LED_BLINK_TIMEOUT 350

typedef struct state {
  bool lpg;
  bool petrol;
  bool empty;
  bool error;
  bool beep;
  bool init_completed;
  byte fuel_state_first;
  byte fuel_state_second;
  uint64_t last_appropriate_message_timestamp;
} state;

state current_state = {false, false, false, false, false, false, 0x00, 0};

bool button_state = false;
bool previous_button_state = false;
long last_button_state_change_timestamp = millis();
uint8_t debounce_delay = 50;

uint8_t current_empty_pin = RED_PIN;
uint8_t current_fuel_change_pin = GREEN_PIN;

uint64_t next_empty_led_timeout = 0;
uint64_t next_fuel_change_led_timeout = 0;


void setup() {

  pinMode(BLUE_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

//  cSerial1.begin(9600);
  Serial.begin(9600);

}
void loop()  {
// Light leds, play beep
  setState();

// Read button state
  checkButton();

// check init completed
  checkInitCompleted();
  
// Check last appropriate message recieve timestamp
  checkAppropriateMessageTimestamp();
  

  if (Serial.available()) {
    byte * incoming_message = new byte[INCOMING_MESSAGE_SIZE];
    uint8_t message_size = 0;

    readIncomingMessage(incoming_message, &message_size);

    bool message_appropriate = checkIncomingMessage(incoming_message, message_size);

    if (!message_appropriate) {
    }
    else {
      current_state.last_appropriate_message_timestamp = millis();
      processIncomingMessage(incoming_message, message_size);
    }
    delete[] incoming_message;
  }
}


void readIncomingMessage(byte message_[], uint8_t * message_size_) {
  
  uint64_t message_end_timestamp = millis();
  bool message_completed = false;
  
  while (!message_completed) {
    if (Serial.available()) {
      message_[*message_size_] = Serial.read();
      message_end_timestamp = millis();
      ++*message_size_;
    }
    
// If last byte recieved long time ago (3 ms as default), message ended      
    if (millis() - message_end_timestamp > INCOMING_MESSAGE_RECIEVE_TIMEOUT) {
      message_completed = true;
    }
// if message size is greater than INCOMING_MESSAGE_SIZE, stop recieving message to prevent data corruption
    if (*message_size_ >= INCOMING_MESSAGE_SIZE ) {
       message_completed = true;
    }
  }
}

bool checkIncomingMessage(byte message_[], const uint8_t message_size_) {

// Minimum allowed msg length is larger than 5 bytes (start byte 0x02, end byte 0x03, dst, src, some payload and checksum)
  if (message_size_ <= MINIMAL_MESSAGE_LENGTH) {
    return false;
  }

// Checking that first byte is 0x02 and last byte is 0x03
  if (message_[0] != 0x02 || message_[message_size_ - 1] != 0x03) {
    return false;
  }
  byte checksum = calculateIncomingChecksum(message_, message_size_);
  
// Checking checksum
  if (checksum != message_[message_size_ - CHECKSUM_POSITION_FROM_END_OF_MESSAGE]) {
    return false;
  }
  return true;
}

void processIncomingMessage(byte message_[], uint8_t * message_size_) {
  uint8_t message_type = getMessageType(message_, message_size_);
  switch (message_type) {
    case INIT1_MESSAGE_TYPE:
      sendReply(outgoing_init1_message_template, sizeof(outgoing_init1_message_template), false, false, false);
      break;
    case INIT2_MESSAGE_TYPE:
      current_state.init_completed = true;
      sendReply(outgoing_init2_message_template, sizeof(outgoing_init2_message_template), false, false, false);
      break;
    case INIT3_MESSAGE_TYPE:
      getBeepStateFromMessage(message_, message_size_);
      sendReply(outgoing_init3_message_template, sizeof(outgoing_init3_message_template), true, true, true);
      break;
    case STATE_MESSAGE_TYPE:
      getStateFromMessage(message_, message_size_);
      sendReply(outgoing_state_message_template, sizeof(outgoing_state_message_template), true, true, true);
      break;
  }
}

uint8_t getMessageType(byte message_[], const uint8_t message_size_) {
  if (message_[MESSAGE_DST_BYTE_POSITION] == DST_INIT && message_[MESSAGE_SRC_BYTE_POSITION] == SRC_INIT1) {
    return INIT1_MESSAGE_TYPE;   
  }
  if (message_[MESSAGE_DST_BYTE_POSITION] == DST_STATE && message_[MESSAGE_SRC_BYTE_POSITION] == SRC_INIT2) {
    return INIT2_MESSAGE_TYPE;   
  }
  if (message_[MESSAGE_DST_BYTE_POSITION] == DST_STATE && message_[MESSAGE_SRC_BYTE_POSITION] == SRC_INIT3) {
    return INIT3_MESSAGE_TYPE;   
  }
  else if (message_[MESSAGE_DST_BYTE_POSITION] == DST_STATE && message_[MESSAGE_SRC_BYTE_POSITION] == SRC_STATE) {
    return STATE_MESSAGE_TYPE;
  }
}

byte calculateIncomingChecksum(byte message_[], const uint8_t message_size_) {
// Calculating checksum for incoming message. Byte before checksum is not used in calculating (don't know why)  
  uint8_t last_checksum_byte = message_size_ - 4;
  byte checksum = 0x0;
// First byte 0x02 is not used in checksum calculating
  for (uint8_t i = 1; i < last_checksum_byte; i++) {
    checksum += message_[i];
  }
// For not init messages, byte -2 before checksum is decreasing checksum
  if (message_[MESSAGE_DST_BYTE_POSITION] != DST_INIT && message_[MESSAGE_SRC_BYTE_POSITION] != SRC_INIT2 && message_[MESSAGE_SRC_BYTE_POSITION] != SRC_INIT3) {
    checksum -= message_[message_size_ - 4];
  }
  if (message_[MESSAGE_SRC_BYTE_POSITION] == SRC_INIT3) {
    checksum += message_[message_size_ - 4];
  }
  return checksum;
}

byte calculateOutgoingChecksum(byte message_[], const uint8_t message_size_) {
// Calculating checksum. Byte before checksum is not used in calculating (don't know why)  
  uint8_t last_checksum_byte = message_size_ - 3;
  byte checksum = 0x0;
  for (uint8_t i = 1; i < last_checksum_byte; i++) {
// Byte 4 is not used in checksum calculating
    if (i != 4) {
      checksum += message_[i];
    }
// But if byte №4 is 0xC0, decrease checksum by its value    
    else {
      if (message_[i] == 0xC0) {
        checksum -= message_[i];
      }
    }
  }
  return checksum;
}

void sendReply(byte message_template_[], uint8_t message_size_, bool calculate_checksum_, bool send_fuel_state_, bool send_button_state_) {

  byte * outgoing_message = new byte[message_size_];

  prepairOutgoingMessage(outgoing_message, message_template_, message_size_);

  if (send_fuel_state_) {
    setFuelStateInMessage(outgoing_message);
  }
  if (send_button_state_) {
    setButtonStateInMessage(outgoing_message);  
  }

  if (calculate_checksum_) {
    setOutgoingMessageChecksum(outgoing_message, message_size_);
  }
  
  sendPrepairedMessage(outgoing_message, message_size_);

  delete[] outgoing_message;

}

// copy template to outgoing message
void prepairOutgoingMessage(byte message_[], byte message_template_[], const uint8_t message_size_) {
  memcpy(message_, message_template_, message_size_);
}

// if button is pressed, set zero bit to 0 in fifth byte, else it should be 1
void setButtonStateInMessage(byte message_[]){
    bitWrite(message_[MESSAGE_BUTTON_BYTE_POSITION], BUTTON_BIT, !button_state);
}

// set fuel state in outgoing message
void setFuelStateInMessage(byte message_[]){
  message_[MESSAGE_FIRST_FUEL_BYTE_POSITION] = current_state.fuel_state_first;
  message_[MESSAGE_SECOND_FUEL_BYTE_POSITION] = current_state.fuel_state_second;
}

void sendPrepairedMessage(byte message_[], uint8_t message_size_) {

  for (uint8_t i = 0; i < message_size_; i++) {
    Serial.write(message_[i]);
// Measured 1.2 ms between bytes in message with original button
    delayMicroseconds(1200);
  }
}

void setOutgoingMessageChecksum(byte message_[], uint8_t message_size_) {
  message_[message_size_ - CHECKSUM_POSITION_FROM_END_OF_MESSAGE] = calculateOutgoingChecksum(message_, message_size_);
}

void getStateFromMessage(byte message_[], const uint8_t message_size_) {
  current_state.fuel_state_first = message_[MESSAGE_FIRST_FUEL_BYTE_POSITION];
  current_state.fuel_state_second = message_[MESSAGE_SECOND_FUEL_BYTE_POSITION];
  current_state.empty = bitRead(message_[MESSAGE_FIRST_FUEL_BYTE_POSITION], EMPTY_BIT);

// To prevent leds blinking on wrong message fuel byte
  bool lpg = bitRead(message_[MESSAGE_FIRST_FUEL_BYTE_POSITION], LPG_BIT);
  bool petrol = bitRead(message_[MESSAGE_FIRST_FUEL_BYTE_POSITION], PETROL_BIT);

  if (lpg || petrol) {
// Beep if fuel changed
    if (current_state.lpg != lpg || current_state.petrol != petrol) {
      current_state.beep = true;
    }
    current_state.lpg = lpg;
    current_state.petrol = petrol;
  }
}

void getBeepStateFromMessage(byte message_[], const uint8_t message_size_) {
  current_state.beep = bitRead(message_[MESSAGE_BEEP_BYTE_POSITION], BEEP_BIT);
}

void setState(){
  lightLeds();
  playBeep();
}

void lightLeds() {
  if (current_state.error) {
    lightLedsError();
    return;
  }
  if (current_state.empty) {
    lightLedsEmpty();
    return;
  }
  if (current_state.lpg && current_state.petrol) {
    lightLedsChangeFuel();
    return;
  }
  digitalWrite(RED_PIN, current_state.empty);
  digitalWrite(GREEN_PIN, current_state.petrol);
  digitalWrite(BLUE_PIN, current_state.lpg);
}

// When LPG out, blink red and blue leds
void lightLedsEmpty() {
  digitalWrite(GREEN_PIN, LOW);
  if (next_empty_led_timeout < millis()) {
    next_empty_led_timeout = millis() + LED_BLINK_TIMEOUT;
    digitalWrite(current_empty_pin, LOW);
    current_empty_pin = (current_empty_pin == RED_PIN) ? BLUE_PIN : RED_PIN;
    digitalWrite(current_empty_pin, HIGH);
  }
}

// When error, ON red led, OFF other leds
void lightLedsError() {
  digitalWrite(BLUE_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(RED_PIN, HIGH);
}

// When changing fuel, blink green and blue leds
void lightLedsChangeFuel() {
  digitalWrite(RED_PIN, LOW);
  if (next_fuel_change_led_timeout  < millis()) {
    next_fuel_change_led_timeout  = millis() + LED_BLINK_TIMEOUT;
    digitalWrite(current_fuel_change_pin, LOW);
    current_fuel_change_pin  = (current_fuel_change_pin  == GREEN_PIN) ? BLUE_PIN : GREEN_PIN;
    digitalWrite(current_fuel_change_pin, HIGH);
  }
}

void playBeep() {
  if (current_state.empty) {
    if (current_empty_pin == RED_PIN) {
      tone(BEEP_PIN, 4000, 1000);
    }
    else {
      noTone(BEEP_PIN);
    }
  }
  else if (current_state.beep) {
      tone(BEEP_PIN, 4000, 350);
      current_state.beep = false;
  }
}



void checkButton() {
  bool current_button_state = digitalRead(BUTTON_PIN);
  if (current_button_state != previous_button_state) {
    previous_button_state = current_button_state;
    last_button_state_change_timestamp = millis();
  }
  else {
    if (millis() - last_button_state_change_timestamp > debounce_delay) {
      if (button_state != current_button_state && current_button_state) {
        current_state.beep = true;
      }
      button_state = current_button_state;
    }
  }
}

void checkInitCompleted() {
  if (!current_state.init_completed && millis() > INIT_TIMEOUT) {
    current_state.error = true;    
  }
  else {
    current_state.error = false; 
  }
}

void checkAppropriateMessageTimestamp() {
  if (millis() - current_state.last_appropriate_message_timestamp > INIT_TIMEOUT) {
    current_state.error = true; 
  }
  else {
    current_state.error = false; 
  }
}
