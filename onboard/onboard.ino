#include <SPI.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10

uint8_t address[][6] = {"1Node", "2Node"};

RF24 radio(CE_PIN, CSN_PIN);

uint8_t ack = 0;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(0, address[0]);
  radio.openWritingPipe(address[1]);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    char message[32];
    radio.read(&message, sizeof(message));
    Serial.println(message);
    radio.stopListening();
    radio.write(&ack, sizeof(ack));
    Serial.print("Sent acknowledgment: ");
    Serial.println(ack);
    ack++;
    radio.startListening();
  }
  delay(100);
}
