/*
 * Arduino A2 - Raspberry Pi Interface with nRF24L01
 * Receives START from A1, communicates with RPi, queues depth data for A1
 */

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// nRF24L01 Configuration
#define CE_PIN 8
#define CSN_PIN 7
RF24 radio(CE_PIN, CSN_PIN);

// Communication addresses
const byte A1_ADDRESS[6] = "00001";  // A1's receiving address
const byte A2_ADDRESS[6] = "00002";  // A2's receiving address (this device)

// Message queue configuration
#define MAX_QUEUE_SIZE 50
#define MAX_MESSAGE_LENGTH 15

struct QueuedMessage {
  char message[MAX_MESSAGE_LENGTH];
  bool valid;
};

// Global variables
QueuedMessage messageQueue[MAX_QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;
int queueCount = 0;

enum SystemState {
  WAITING_FOR_START,
  ACTIVE,
  ERROR_STATE
};

SystemState currentState = WAITING_FOR_START;
unsigned long lastTransmitAttempt = 0;
unsigned long lastQueueProcess = 0;
const unsigned long TRANSMIT_RETRY_INTERVAL = 100;  // Try every 100ms
const unsigned long QUEUE_PROCESS_INTERVAL = 50;    // Process queue every 50ms

// Statistics
unsigned long totalMessagesSent = 0;
unsigned long totalMessagesQueued = 0;
unsigned long totalMessagesDropped = 0;

void setup() {
  Serial.begin(9600);
  
  // Initialize nRF24L01
  if (!radio.begin()) {
    Serial.println("ERROR: nRF24L01 initialization failed!");
    currentState = ERROR_STATE;
    return;
  }
  
  setupRadio();
  
  Serial.println("A2 Ready - Waiting for START signal from A1");
  printStatus();
}

void setupRadio() {
  // Configure radio settings
  radio.setPALevel(RF24_PA_HIGH);     // Max power for underwater communication
  radio.setDataRate(RF24_250KBPS);    // Slower but more reliable
  radio.setChannel(76);               // Channel 76 (2.476 GHz)
  radio.setRetries(5, 15);            // 5 retries, 15*250µs delay
  radio.enableAckPayload();           // Enable acknowledgment payloads
  
  // Set up addresses
  radio.openWritingPipe(A1_ADDRESS);    // Write to A1
  radio.openReadingPipe(1, A2_ADDRESS); // Read on A2 address
  
  // Start listening for incoming messages
  radio.startListening();
  
  Serial.println("nRF24L01 configured successfully");
}

void loop() {
  switch (currentState) {
    case WAITING_FOR_START:
      handleWaitingForStart();
      break;
      
    case ACTIVE:
      handleActiveState();
      break;
      
    case ERROR_STATE:
      handleErrorState();
      break;
  }
  
  // Always process the message queue
  processMessageQueue();
  
  delay(10); // Small delay to prevent overwhelming the system
}

void handleWaitingForStart() {
  // Check for START message from A1
  if (radio.available()) {
    char receivedMessage[32];
    radio.read(&receivedMessage, sizeof(receivedMessage));
    
    String message = String(receivedMessage);
    message.trim();
    
    Serial.print("Received from A1: ");
    Serial.println(message);
    
    if (message == "START") {
      Serial.println("START"); // Tell RPi to start
      currentState = ACTIVE;
      Serial.println("System activated - Now accepting depth data from RPi");
      printStatus();
    }
    if (message == "EXPEL") {
      Serial.println("EXPEL");
    }
    if (message == "INTAKE") {
      Serial.println("INTAKE");
    }
    if (message == "STOP") {
      Serial.println("STOP");
    }
  }
}

void handleActiveState() {
  // Check for depth data from Raspberry Pi
  if (Serial.available()) {
    String incomingData = Serial.readStringUntil('\n');
    incomingData.trim();
    
    if (incomingData.startsWith("DEPTH:")) {
      // Extract depth value for validation
      float depth = incomingData.substring(6).toFloat();
      
      // Queue the message for transmission to A1
      if (queueMessage(incomingData)) {
        Serial.print("Queued: ");
        Serial.print(incomingData);
        Serial.print(" (Queue: ");
        Serial.print(queueCount);
        Serial.println("/180)");
      } else {
        Serial.println("ERROR: Queue full! Message dropped.");
        totalMessagesDropped++;
      }
    }else{
      Serial.print("unknown command: ");
      Serial.println(incomingData);
    }
  }
  
  // Check for any messages from A1 (acknowledgments, commands, etc.)
  if (radio.available()) {
    char receivedMessage[32];
    radio.read(&receivedMessage, sizeof(receivedMessage));
    
    String message = String(receivedMessage);
    message.trim();
    
    if (message.length() > 0) {
      Serial.print("A1 response: ");
      Serial.println(message);
    }
  }
}

void handleErrorState() {
  Serial.println("System in ERROR state. Check nRF24L01 connection.");
  delay(5000);
  
  // Try to reinitialize
  if (radio.begin()) {
    setupRadio();
    currentState = WAITING_FOR_START;
    Serial.println("Radio reinitialized. Waiting for START signal.");
  }
}

bool queueMessage(String message) {
  if (queueCount >= MAX_QUEUE_SIZE) {
    return false; // Queue is full
  }
  // Add message to queue
  message.toCharArray(messageQueue[queueTail].message, MAX_MESSAGE_LENGTH);
  //messageQueue[queueTail].timestamp = millis();
  messageQueue[queueTail].valid = true;
  
  queueTail = (queueTail + 1) % MAX_QUEUE_SIZE;
  queueCount++;
  totalMessagesQueued++;
  
  return true;
}

void processMessageQueue() {
  if (queueCount == 0) {
    return; // No messages to process
  }
  
  unsigned long currentTime = millis();
  
  // Only try to process queue at specified intervals
  if (currentTime - lastQueueProcess < QUEUE_PROCESS_INTERVAL) {
    return;
  }
  
  lastQueueProcess = currentTime;
  
  // Try to send the oldest message in the queue
  if (messageQueue[queueHead].valid) {
    if (sendMessageToA1(messageQueue[queueHead].message)) {
      // Message sent successfully
      Serial.print("Sent to A1: ");
      Serial.print(messageQueue[queueHead].message);
      //Serial.print(" (Age: ");
      //Serial.print(currentTime - messageQueue[queueHead].timestamp);
      //Serial.println("ms)");
      
      // Remove message from queue
      messageQueue[queueHead].valid = false;
      queueHead = (queueHead + 1) % MAX_QUEUE_SIZE;
      queueCount--;
      totalMessagesSent++;
      
      // Print queue status every 10 messages
      if (totalMessagesSent % 10 == 0) {
        printStatus();
      }
    }
    // If sending failed, message stays in queue for next attempt
  }
}

bool sendMessageToA1(const char* message) {
  radio.stopListening(); // Stop listening to send
  
  bool result = radio.write(message, strlen(message) + 1);
  
  radio.startListening(); // Resume listening
  
  return result;
}

void printStatus() {
  Serial.println("=== A2 STATUS ===");
  Serial.print("State: ");
  switch (currentState) {
    case WAITING_FOR_START:
      Serial.println("Waiting for START");
      break;
    case ACTIVE:
      Serial.println("Active");
      break;
    case ERROR_STATE:
      Serial.println("Error");
      break;
  }
  
  Serial.print("Queue: ");
  Serial.print(queueCount);
  Serial.print("/");
  Serial.println(MAX_QUEUE_SIZE);
  
  Serial.print("Messages - Sent: ");
  Serial.print(totalMessagesSent);
  Serial.print(", Queued: ");
  Serial.print(totalMessagesQueued);
  Serial.print(", Dropped: ");
  Serial.println(totalMessagesDropped);
  
  //if (queueCount > 0) {
    //unsigned long oldestAge = millis() - messageQueue[queueHead].timestamp;
    //Serial.print("Oldest queued message age: ");
    //Serial.print(oldestAge);
    //Serial.println("ms");
  //}
  
  Serial.println("================");
}

// Command interface for debugging (optional)
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    if (command == "STATUS") {
      printStatus();
    } else if (command == "RESET") {
      // Reset queue
      queueHead = 0;
      queueTail = 0;
      queueCount = 0;
      Serial.println("Queue reset");
    } else if (command.startsWith("TEST:")) {
      // Test message
      String testMsg = command.substring(5);
      if (queueMessage(testMsg)) {
        Serial.println("Test message queued");
      } else {
        Serial.println("Failed to queue test message");
      }
    }
  }
}
