#include <SPI.h>
#include <RF24.h>

// NRF24L01 pins
#define CE_PIN 8
#define CSN_PIN 7

// Radio setup
RF24 radio(CE_PIN, CSN_PIN);
const byte addresses[][6] = {"00001", "00002"};

// Communication buffers
char uartBuffer[256] = {0};
char radioBuffer[256] = {0};
int uartIndex = 0;
int radioIndex = 0;

// Acknowledgment tracking
struct AckPacket {
  char type[10];
  int sequence;
  bool received;
};

void setup() {
  // Initialize hardware UART (Pi communication)
  Serial.begin(9600);
  
  // Initialize radio
  radio.begin();
  radio.openWritingPipe(addresses[0]);     // To A1
  radio.openReadingPipe(1, addresses[1]);  // From A1
  radio.setPALevel(RF24_PA_HIGH);
  radio.setRetries(15, 15);
  radio.startListening();
  radio.setPayloadSize(256);
  
  Serial.println("A2 Relay initialized");
}

void loop() {
  // Handle Pi -> A1 communication
  handlePiToA1();
  
  // Handle A1 -> Pi communication
  handleA1ToPi();
  
  delay(10);
}

void handlePiToA1() {
  // Read from Pi via UART
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || uartIndex >= sizeof(uartBuffer) - 1) {
      uartBuffer[uartIndex] = '\0';
      
      // Send to A1 via radio
      if (strlen(uartBuffer) > 0) {
        sendToA1(uartBuffer);
      }
      
      uartIndex = 0;
    } else {
      uartBuffer[uartIndex++] = c;
    }
  }
}

void handleA1ToPi() {
  // Read from A1 via radio
  if (radio.available()) {
    radio.read(&radioBuffer, sizeof(radioBuffer));
    
    Serial.println("received");
    // Send to Pi via UART
    Serial.println(radioBuffer);
    
    // Send acknowledgment back to A1
    sendAckToA1(radioBuffer);
  }
}

void sendToA1(const char* message) {
  radio.stopListening();
  
  bool result = radio.write(message, strlen(message) + 1);
  
  if (!result) {
    Serial.println("ERROR: Failed to send to A1");
  }
  else{
    Serial.println("sent to A1");
  }
  
  radio.startListening();
}

void sendAckToA1(const char* originalMessage) {
  // Parse sequence number from message if it's a pressure reading
  if (strncmp(originalMessage, "PRESSURE:", 9) == 0) {
    char ackMessage[64];
    
    // Extract sequence number
    int sequence = 0;
    sscanf(originalMessage, "PRESSURE:%d:", &sequence);
    
    snprintf(ackMessage, sizeof(ackMessage), "ACK:PRESSURE:%d", sequence);
    
    radio.stopListening();
    radio.write(ackMessage, strlen(ackMessage) + 1);
    radio.startListening();
    Serial.println("sent ACK to A1");
  }
}
