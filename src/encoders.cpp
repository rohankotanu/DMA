#include <Arduino.h>
#include "DMA.hpp"

// Define and set the command sent to the encoders to trigger a position reading
constexpr uint8_t encoder_command = 0x54;
volatile uint16_t hcom, vcom;

const float maxEncoderCount = pow(2, 14);

DMA dma;

void setup() {
  
  dma.addUARTEncoder(1, &encoder_command, &hcom);
  dma.addUARTEncoder(3, &encoder_command, &vcom);
  
}

void loop() {

  float hcom_ang = 2.0 * PI * static_cast<float>(hcom & 0x3FFF) / maxEncoderCount ;
  float vcom_ang = 2.0 * PI * static_cast<float>(vcom & 0x3FFF) / maxEncoderCount ;

  Serial.print("hcom: ");
  Serial.print(hcom_ang);
  Serial.print(", vcom: ");
  Serial.println(vcom_ang);
  
}