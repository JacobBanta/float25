/*
 * Arduino A1 - Laptop Interface with nRF24L01
 * Sends START to A2, receives depth data from A2, formats output for laptop
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// nRF24L01 Configuration
#define CE_PIN 7
#define CSN_PIN 8
RF24 radio(CE_PIN, CSN_PIN);

// Communication addresses
const byte A1_ADDRESS[6] = "00001";  // A1's receiving address (this device)
const byte A2_ADDRESS[6] = "00002";  // A2's receiving address

enum SystemState {
  WAITING_FOR_START_COMMAND,
  ACTIVE,
  ERROR_STATE
};

SystemState currentState = WAITING_FOR_START_COMMAND;
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 10000; // 10 seconds

// Statistics
unsigned long totalDataReceived = 0;
unsigned long totalStartCommands = 0;
unsigned long lastDataTime = 0;

void setup() {
  Serial.begin(9600);
  
  // Initialize nRF24L01
  if (!radio.begin()) {
    Serial.println("INFO:nRF24L01 initialization failed!");
    currentState = ERROR_STATE;
    return;
  }
  
  setupRadio();
  
  Serial.println("INFO:A1 Ready - Send START to begin communication with A2");
}

void setupRadio() {
  // Configure radio settings (same as A2 for compatibility)
  radio.setPALevel(RF24_PA_HIGH);     // Max power for underwater communication
  radio.setDataRate(RF24_250KBPS);    // Slower but more reliable
  radio.setChannel(76);               // Channel 76 (2.476 GHz)
  radio.setRetries(5, 15);            // 5 retries, 15*250µs delay
  radio.enableAckPayload();           // Enable acknowledgment payloads
  
  // Set up addresses (opposite of A2)
  radio.openWritingPipe(A2_ADDRESS);    // Write to A2
  radio.openReadingPipe(1, A1_ADDRESS); // Read on A1 address
  
  // Start listening for incoming messages
  radio.startListening();
  
  Serial.println("INFO:nRF24L01 configured successfully");
}

void loop() {
  switch (currentState) {
    case WAITING_FOR_START_COMMAND:
      handleWaitingForStart();
      break;
      
    case ACTIVE:
      handleActiveState();
      break;
      
    case ERROR_STATE:
      handleErrorState();
      break;
  }
  
  // Handle serial commands from laptop
  handleSerialCommands();
  
  // Periodic heartbeat
  sendHeartbeat();
  
  delay(10); // Small delay to prevent overwhelming the system
}

void handleWaitingForStart() {
  // Just wait for START command from serial
  // All logic is in handleSerialCommands()
}

void handleActiveState() {
  // Listen for data from A2
  if (radio.available()) {
    char receivedMessage[32];
    radio.read(&receivedMessage, sizeof(receivedMessage));
    
    String message = String(receivedMessage);
    message.trim();
    
    if (message.length() > 0) {
      processMessageFromA2(message);
    }
  }
}

void handleErrorState() {
  Serial.println("INFO:System in ERROR state. Attempting to reinitialize radio.");
  delay(5000);
  
  // Try to reinitialize
  if (radio.begin()) {
    setupRadio();
    currentState = WAITING_FOR_START_COMMAND;
    Serial.println("INFO:Radio reinitialized successfully");
  }
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.equalsIgnoreCase("START")) {
      handleStartCommand();
    } else if (command.equalsIgnoreCase("STATUS")) {
      printStatus();
    } else if (command.equalsIgnoreCase("RESET")) {
      resetSystem();
    } else if (command.length() > 0) {
      Serial.print("INFO:Unknown command: ");
      Serial.println(command);
      Serial.println("INFO:Available commands - START, STATUS, RESET");
    }
  }
}

void handleStartCommand() {
  Serial.println("INFO:Sending START signal to A2...");
  
  if (sendMessageToA2("START")) {
    Serial.println("INFO:START signal sent successfully");
    currentState = ACTIVE;
    totalStartCommands++;
    Serial.println("INFO:System activated - Now listening for depth data from A2");
  } else {
    Serial.println("INFO:Failed to send START signal to A2");
    Serial.println("INFO:Check A2 connection and try again");
  }
}

void processMessageFromA2(String message) {
  lastDataTime = millis();
  
  if (message.startsWith("DEPTH:")) {
    // Extract the depth value (number only)
    String depthStr = message.substring(6);
    float depthValue = depthStr.toFloat();
    
    // Print in required format: DATA:{number only}
    Serial.print("DATA:");
    Serial.println(depthValue, 2); // 2 decimal places
    
    totalDataReceived++;
    
    // Send acknowledgment back to A2
    String ackMessage = "ACK:" + String(totalDataReceived);
    sendMessageToA2(ackMessage.c_str());
    
  } else {
    // Any other message from A2
    Serial.print("INFO:A2 message: ");
    Serial.println(message);
  }
}

bool sendMessageToA2(const char* message) {
  radio.stopListening(); // Stop listening to send
  
  bool result = radio.write(message, strlen(message) + 1);
  
  radio.startListening(); // Resume listening
  
  if (!result) {
    Serial.print("INFO:Failed to send message to A2: ");
    Serial.println(message);
  }
  
  return result;
}

void sendHeartbeat() {
  unsigned long currentTime = millis();
  
  if (currentState == ACTIVE && 
      currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
    
    lastHeartbeat = currentTime;
    
    Serial.print("INFO:Heartbeat - Data received: ");
    Serial.print(totalDataReceived);
    
    if (lastDataTime > 0) {
      unsigned long timeSinceLastData = currentTime - lastDataTime;
      Serial.print(", Last data: ");
      Serial.print(timeSinceLastData / 1000);
      Serial.println("s ago");
    } else {
      Serial.println(", No data received yet");
    }
  }
}

void printStatus() {
  Serial.println("INFO:=== A1 STATUS ===");
  
  Serial.print("INFO:State: ");
  switch (currentState) {
    case WAITING_FOR_START_COMMAND:
      Serial.println("Waiting for START command");
      break;
    case ACTIVE:
      Serial.println("Active - Listening for A2 data");
      break;
    case ERROR_STATE:
      Serial.println("Error");
      break;
  }
  
  Serial.print("INFO:START commands sent: ");
  Serial.println(totalStartCommands);
  
  Serial.print("INFO:Data messages received: ");
  Serial.println(totalDataReceived);
  
  if (lastDataTime > 0) {
    unsigned long timeSinceLastData = millis() - lastDataTime;
    Serial.print("INFO:Time since last data: ");
    Serial.print(timeSinceLastData / 1000);
    Serial.println(" seconds");
  } else {
    Serial.println("INFO:No data received yet");
  }
  
  // Radio status
  //Serial.print("INFO:Radio listening: ");
  //Serial.println(radio.isListening() ? "Yes" : "No");
  
  Serial.println("INFO:==================");
}

void resetSystem() {
  Serial.println("INFO:Resetting system...");
  
  // Reset statistics
  totalDataReceived = 0;
  totalStartCommands = 0;
  lastDataTime = 0;
  lastHeartbeat = 0;
  
  // Reset state
  currentState = WAITING_FOR_START_COMMAND;
  
  // Reinitialize radio
  setupRadio();
  
  Serial.println("INFO:System reset complete");
  Serial.println("INFO:Send START to begin communication with A2");
}

void printHelp() {
  Serial.println("INFO:=== A1 COMMAND HELP ===");
  Serial.println("INFO:START  - Send START signal to A2 and begin operation");
  Serial.println("INFO:STATUS - Print current system status");
  Serial.println("INFO:RESET  - Reset system and return to initial state");
  Serial.println("INFO:========================");
}
