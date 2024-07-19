#include <Arduino.h>
#include "DMA.hpp"

// Define and set the command sent to the encoders to trigger a position reading
constexpr uint8_t encoder_command = 0x54;
volatile uint16_t hcom, vcom;

// Init LINK buffers
uint16_t encoder_vals[5];
uint16_t snd_link[3];
uint16_t rec_link[3];

const float maxEncoderCount = pow(2, 14);

DMA dma;

int i = 0;
uint8_t state = 3;
uint8_t recv_state = 0;

void setup() {
  snd_link[0] = 0b01;
  snd_link[1] = 0b11;
  snd_link[2] = 0b111;

  dma.addUARTEncoder(4, &encoder_command, encoder_vals+1); // HPOS
  dma.addUARTEncoder(8, &encoder_command, encoder_vals+2); // VPOS
  dma.addUARTEncoder(1, &encoder_command, encoder_vals+3); // HCOM
  dma.addUARTEncoder(3, &encoder_command, encoder_vals+4); // VCOM

  dma.addLinkChannel(6, snd_link, rec_link);
  dma.addStateChannel(7, &state, &recv_state);

  // IMXRT_LPUART_t* LPUART = (IMXRT_LPUART_t*)(0x40184000 + (5) * 0x4000); // Pointer to the LPUART registers corresponding to the desired HardwareSerial port

  // uint32_t temp;
  // (*LPUART).CTRL &= ~(0b11 << 18);
  // while ((*LPUART).CTRL & (0b11<<18)) {;}
  // // while (rec_link[0] != 0x0015) {
  // //   temp = (*LPUART).DATA;
  // // }
  // (*LPUART).CTRL |= (0b11<<18);

}

void loop() {
  
  if ((i++ % 1000) == 0) {
    float hcom_ang = 2.0 * PI * static_cast<float>(encoder_vals[3] & 0x3FFF) / maxEncoderCount ;
    float vcom_ang = 2.0 * PI * static_cast<float>(encoder_vals[4] & 0x3FFF) / maxEncoderCount ;
    // float hcom_ang = 2.0 * PI * static_cast<float>(rec_link[1] & 0x3FFF) / maxEncoderCount ;
    // float vcom_ang = 2.0 * PI * static_cast<float>(rec_link[2] & 0x3FFF) / maxEncoderCount ;

    // Serial.print("parity: ");
    // Serial.print(rec_link[0]);
    Serial.print("hcom: ");
    Serial.print(hcom_ang);
    Serial.print(", vcom: ");
    Serial.println(vcom_ang);

    // Serial.print("val1: ");
    // Serial.print(rec_link[0], BIN);
    // Serial.print(", val2: ");
    // Serial.print(rec_link[1], BIN);
    // Serial.print(", val3: ");
    // Serial.println(rec_link[2], BIN);

    // Serial.print("state: ");
    // Serial.println(recv_state);

  }
}