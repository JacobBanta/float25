#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN pins
const byte address[6] = "00001";

struct DataPacket {
  char type; // 'P' for pressure, 'C' for command, 'R' for request, 'A' for ack, 'S' for start, 'M' for param
  float pressure;
  uint16_t sequence;
  char command;
  char param_name[16];
  float param_value;
};

uint16_t expected_sequence = 0;
uint16_t last_received_sequence = 0;
unsigned long last_packet_time = 0;
const unsigned long PACKET_TIMEOUT = 10000;

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
  Serial.println("A1 Ready - Enhanced Control System");
  Serial.println("Commands:");
  Serial.println("  0/1 - GPIO control");
  Serial.println("  START - Begin depth control sequence");
  Serial.println("  SET <param> <value> - Set parameter");
  Serial.println("    depth_trigger <mbar>");
  Serial.println("    max_depth <mbar>");
  Serial.println("    stability_threshold <mbar>");
  Serial.println("    stability_duration <seconds>");
}

void loop() {
  // Check for user input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input == "0" || input == "1") {
      send_gpio_command(input.charAt(0));
    }
    else if (input == "START") {
      send_start_command();
    }
    else if (input.startsWith("SET ")) {
      handle_parameter_command(input);
    }
  }
  
  // Check for incoming pressure data
  if (radio.available()) {
    DataPacket packet;
    radio.read(&packet, sizeof(packet));
    
    if (packet.type == 'P') {
      handle_pressure_packet(packet);
    }
  }
  
  // Check for timeout
  if (millis() - last_packet_time > PACKET_TIMEOUT && last_packet_time > 0) {
    Serial.println("Packet timeout - requesting latest data");
    request_packet(expected_sequence);
    last_packet_time = millis();
  }
  
  delay(10);
}

void send_gpio_command(char command) {
  DataPacket packet;
  packet.type = 'C';
  packet.command = command;
  packet.sequence = 0;
  
  radio.stopListening();
  radio.write(&packet, sizeof(packet));
  radio.startListening();
  
  Serial.print("Sent GPIO command: ");
  Serial.println(command);
}

void send_start_command() {
  DataPacket packet;
  packet.type = 'S';
  packet.sequence = 0;
  
  radio.stopListening();
  radio.write(&packet, sizeof(packet));
  radio.startListening();
  
  Serial.println("Sent START command - Beginning depth control sequence");
}

void handle_parameter_command(String input) {
  // Parse "SET param_name value"
  int first_space = input.indexOf(' ');
  int second_space = input.indexOf(' ', first_space + 1);
  
  if (first_space != -1 && second_space != -1) {
    String param_name = input.substring(first_space + 1, second_space);
    float param_value = input.substring(second_space + 1).toFloat();
    
    DataPacket packet;
    packet.type = 'M';
    packet.param_value = param_value;
    packet.sequence = 0;
    
    // Copy parameter name (truncate if too long)
    param_name.toCharArray(packet.param_name, sizeof(packet.param_name));
    
    radio.stopListening();
    radio.write(&packet, sizeof(packet));
    radio.startListening();
    
    Serial.print("Set parameter ");
    Serial.print(param_name);
    Serial.print(" = ");
    Serial.println(param_value);
  } else {
    Serial.println("Invalid format. Use: SET <param_name> <value>");
  }
}

void handle_pressure_packet(DataPacket packet) {
  last_packet_time = millis();
  
  // Check for missing packets
  if (packet.sequence != expected_sequence && expected_sequence > 0) {
    Serial.print("MISSING PACKETS! Expected: ");
    Serial.print(expected_sequence);
    Serial.print(", Received: ");
    Serial.println(packet.sequence);
    
    for (uint16_t missing = expected_sequence; missing < packet.sequence; missing++) {
      request_packet(missing);
      delay(50);
    }
  }
  
  send_ack(packet.sequence);
  
  Serial.print("Pressure: ");
  Serial.print(packet.pressure);
  Serial.print(" mbar, Sequence: ");
  Serial.println(packet.sequence);
  
  expected_sequence = packet.sequence + 1;
  last_received_sequence = packet.sequence;
}

void request_packet(uint16_t sequence) {
  DataPacket request;
  request.type = 'R';
  request.sequence = sequence;
  
  radio.stopListening();
  radio.write(&request, sizeof(request));
  radio.startListening();
}

void send_ack(uint16_t sequence) {
  DataPacket ack;
  ack.type = 'A';
  ack.sequence = sequence;
  
  radio.stopListening();
  radio.write(&ack, sizeof(ack));
  radio.startListening();
}
