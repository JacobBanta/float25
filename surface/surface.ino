#include <SPI.h>
#include <RF24.h>

// NRF24L01 pins
#define CE_PIN 7
#define CSN_PIN 8

// Radio setup
RF24 radio(CE_PIN, CSN_PIN);
const byte addresses[][6] = {"00001", "00002"};

// Packet tracking
struct PacketTracker {
  int lastSequence;
  unsigned long lastReceived;
  bool waitingForAck[100]; // Track last 100 packets
};

PacketTracker tracker = {0, 0, {false}};

// User interface
String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  
  // Initialize radio
  radio.begin();
  radio.openWritingPipe(addresses[1]);     // To A2
  radio.openReadingPipe(1, addresses[0]);  // From A2
  radio.setPALevel(RF24_PA_HIGH);
  radio.setRetries(15, 15);
  radio.startListening();
  radio.setPayloadSize(256);
  
  //Serial.println("A1 Controller initialized");
  //Serial.println("Commands:");
  //Serial.println("  START - Begin depth control");
  //Serial.println("  0 <pin> - Set GPIO pin LOW");
  //Serial.println("  1 <pin> - Set GPIO pin HIGH");
  //Serial.println("  SET <param> <value> - Update parameter");
  //Serial.println("  MONITOR - Show packet status");
}

void loop() {
  serialEvent();
  // Handle incoming serial commands
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
  
  // Handle incoming radio messages
  handleRadioMessages();
  
  // Check for missing packets
  checkMissingPackets();
  
  delay(10);
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    } else {
      Serial.println(inChar, HEX);
      inputString += inChar;
    }
  }
}

void processCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  if (command == "START") {
    sendCommand("START");
    Serial.println("INFO: Sent START command");
    
  } else if (command.startsWith("SET ")) {
    sendCommand(command.c_str());
    Serial.println("INFO: Sent parameter update: " + command);
    
  } else if (command.length() >= 3 && (command.startsWith("0 ") || command.startsWith("1 "))) {
    sendCommand(command.c_str());
    Serial.println("INFO: Sent GPIO command: " + command);
    
  } else if (command == "MONITOR") {
    showPacketStatus();
    
  } else {
    Serial.println("INFO: Unknown command: " + command);
  }
}

void sendCommand(const char* command) {
  radio.stopListening();
  
  bool result = radio.write(command, strlen(command) + 1);
  
  if (result) {
    Serial.println("INFO: Command sent successfully");
  } else {
    Serial.println("INFO: Failed to send command");
  }
  
  radio.startListening();
}

void handleRadioMessages() {
  char buffer[256] = {0};
  
  if (radio.available()) {
    radio.read(buffer, sizeof(buffer));
    
    if (strncmp(buffer, "PRESSURE:", 9) == 0) {
      processPressureReading(buffer);
    } else if (strncmp(buffer, "ACK:", 4) == 0) {
      processAcknowledgment(buffer);
    } else {
      Serial.println("INFO: received " + String(buffer));
    }
  }
}

void processPressureReading(const char* message) {
  int sequence;
  float pressure;
  
  if (sscanf(message, "PRESSURE:%d:%f", &sequence, &pressure) == 2) {
    Serial.print("Pressure: ");
    Serial.print(pressure, 2);
    Serial.print(" mbar (seq: ");
    Serial.print(sequence);
    Serial.println(")");
    
    tracker.lastSequence = sequence;
    tracker.lastReceived = millis();
    
    // Mark as received
    if (sequence < 100) {
      tracker.waitingForAck[sequence % 100] = true;
    }
  }
}

void processAcknowledgment(const char* message) {
  int sequence;
  
  if (sscanf(message, "ACK:PRESSURE:%d", &sequence) == 1) {
    // Mark as acknowledged
    if (sequence < 100) {
      tracker.waitingForAck[sequence % 100] = false;
    }
  }
}

void checkMissingPackets() {
  static unsigned long lastCheck = 0;
  
  if (millis() - lastCheck > 30000) { // Check every 30 seconds
    lastCheck = millis();
    
    // Check for missing packets in last 100
    for (int i = 0; i < 100; i++) {
      if (tracker.waitingForAck[i]) {
        Serial.print("INFO: Missing packet: ");
        Serial.println(i);
        requestRetransmission(i);
      }
    }
  }
}

void requestRetransmission(int sequence) {
  char request[32];
  snprintf(request, sizeof(request), "RETRANSMIT:%d", sequence);
  
  radio.stopListening();
  radio.write(request, strlen(request) + 1);
  radio.startListening();
  
  Serial.print("INFO: Requested retransmission of packet ");
  Serial.println(sequence);
}

void showPacketStatus() {
  Serial.println("=== Packet Status ===");
  Serial.print("Last sequence: ");
  Serial.println(tracker.lastSequence);
  Serial.print("Last received: ");
  Serial.print((millis() - tracker.lastReceived) / 1000);
  Serial.println(" seconds ago");
  
  int missing = 0;
  for (int i = 0; i < 100; i++) {
    if (tracker.waitingForAck[i]) missing++;
  }
  Serial.print("Missing packets: ");
  Serial.println(missing);
  Serial.println("====================");
}
