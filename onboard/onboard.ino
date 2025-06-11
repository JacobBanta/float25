#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(8, 10); // CE=8, CSN=10
const byte address[6] = "00001";

struct DataPacket {
  char type;
  float pressure;
  uint16_t sequence;
  char command;
  char param_name[16];
  float param_value;
};

void setup() {
  // Hardware UART for Pi communication (pins 0/1)
  Serial.begin(9600);
  
  radio.begin();
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
  // Clear UART buffer
  while(Serial.available()) {
    Serial.read();
  }
  
  delay(1000);
  Serial.println("A2_READY");
}

void loop() {
  // Check for radio data from A1
  if (radio.available()) {
    DataPacket packet;
    radio.read(&packet, sizeof(packet));
    
    if (packet.type == 'C') {
      Serial.print("CMD:");
      Serial.print(packet.command);
      Serial.print("\n");
      Serial.flush();
    }
    else if (packet.type == 'S') {
      Serial.print("START\n");
      Serial.flush();
    }
    else if (packet.type == 'M') {
      Serial.print("PARAM:");
      Serial.print(packet.param_name);
      Serial.print(":");
      Serial.print(packet.param_value);
      Serial.print("\n");
      Serial.flush();
    }
    else if (packet.type == 'R') {
      Serial.print("REQ:");
      Serial.print(packet.sequence);
      Serial.print("\n");
      Serial.flush();
    }
    else if (packet.type == 'A') {
      Serial.print("ACK:");
      Serial.print(packet.sequence);
      Serial.print("\n");
      Serial.flush();
    }
  }
  
  // Check for data from Pi
  if (Serial.available()) {
    String piData = "";
    unsigned long startTime = millis();
    
    // Read complete line with timeout
    while (millis() - startTime < 1000) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
          break;
        } else if (c >= 32 && c <= 126) { // Only printable ASCII
          piData += c;
        }
      }
    }
    
    if (piData.length() > 0) {
      if (piData.startsWith("PRESSURE:")) {
        int firstColon = piData.indexOf(':');
        int secondColon = piData.indexOf(':', firstColon + 1);
        
        if (firstColon != -1 && secondColon != -1) {
          float pressure = piData.substring(firstColon + 1, secondColon).toFloat();
          uint16_t sequence = piData.substring(secondColon + 1).toInt();
          
          DataPacket packet;
          packet.type = 'P';
          packet.pressure = pressure;
          packet.sequence = sequence;
          
          radio.stopListening();
          bool success = radio.write(&packet, sizeof(packet));
          radio.startListening();
        }
      }
    }
  }
  
  delay(10);
}
