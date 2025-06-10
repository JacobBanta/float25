#include <SPI.h>
#include <RF24.h>

#define CE_PIN 7
#define CSN_PIN 8

uint8_t address[][6] = {"1Node", "2Node"};

RF24 radio(CE_PIN, CSN_PIN);

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address[0]);
  radio.openReadingPipe(1, address[1]);
  radio.stopListening();
}

void loop() {
  if (Serial.available() > 0) {
    char message[32];
    Serial.readBytesUntil('\n', message, 32);
    message[Serial.readBytesUntil('\n', message, 32)] = '\0';
    radio.stopListening();
    if (radio.write(&message, sizeof(message))){
      Serial.print("Sent: ");
      Serial.println(message);
      radio.startListening();
      if (radio.available()) {
        uint8_t ack;
        radio.read(&ack, sizeof(ack));
        Serial.print("Received acknowledgment: ");
        Serial.println(ack);
      } else {
        Serial.println("No acknowledgment received");
      }
    } else {
      Serial.println("Failed to send message");
    }
    radio.stopListening();
  }
  delay(100);
}
