#include <Arduino.h>
#include "DMA.hpp"

// Define and set the command sent to the encoders to trigger a position reading
constexpr uint8_t encoder_command = 0x54;
volatile uint16_t hcom, vcom;

// Init LINK buffers
uint8_t pos_encoders[6];
uint8_t com_encoders[2];
uint8_t rec_link[6];
uint16_t snd_link[3];

const float maxEncoderCount = pow(2, 14);

DMA dma;

int i = 0;
uint8_t state = 3;
uint8_t recv_state = 0;

void setup() {
  pos_encoders[0] = 21;
  snd_link[0] = 0x1234;
  snd_link[1] = 0x5678;
  snd_link[2] = 0x9ABC;
  // snd_link[0] = 0x12;
  // snd_link[1] = 0x34;
  // snd_link[2] = 0x56;
  // snd_link[3] = 0x78;
  // snd_link[4] = 0x9A;
  // snd_link[5] = 0xBC;

  dma.addUARTEncoder(4, &encoder_command, pos_encoders+3); // HPOS
  dma.addUARTEncoder(8, &encoder_command, pos_encoders+5); // VPOS
  dma.addUARTEncoder(1, &encoder_command, com_encoders); // HCOM
  dma.addUARTEncoder(3, &encoder_command, com_encoders+1); // VCOM

  dma.addLinkChannel(6, pos_encoders, rec_link);
  dma.addStateChannel(7, &state, &recv_state);

  // DMA_CERQ = 17;
  // IMXRT_DMA_TCD[17].CSR |= (0x4 << 8) | (0b1 <<5);
  // DMA_SERQ = 17;

  // IMXRT_LPUART_t* LPUART = (IMXRT_LPUART_t*)(0x40184000 + (5) * 0x4000); // Pointer to the LPUART registers corresponding to the desired HardwareSerial port

  // uint32_t temp;
  // (*LPUART).CTRL &= ~(0b11 << 18);
  // while ((*LPUART).CTRL & (0b11<<18)) {;}
  // // while (rec_link[0] != 0x0015) {
  // //   temp = (*LPUART).DATA;
  // // }
  // (*LPUART).CTRL |= (0b11<<18);
  // Serial.begin(115200);
  // while (rec_link[0] != 21) {
  //   Serial.println(rec_link[0]);
  //   DMA_CERQ = 20;
  //   delayNanoseconds(25);
  //   DMA_SERQ = 20;
  // }

}

void loop() {
  
  
  if ((i++ % 1000) == 0) {

    // float hcom_ang = 2.0 * PI * static_cast<float>(encoder_vals[3] & 0x3FFF) / maxEncoderCount ;
    // float vcom_ang = 2.0 * PI * static_cast<float>(encoder_vals[4] & 0x3FFF) / maxEncoderCount ;
    // float hcom_ang = 2.0 * PI * static_cast<float>(((pos_encoders[2] & 0x3F) << 8) | pos_encoders[3]) / maxEncoderCount ;
    // float vcom_ang = 2.0 * PI * static_cast<float>(((pos_encoders[4] & 0x3F) << 8) | pos_encoders[5]) / maxEncoderCount ;
    // float hcom_ang = 2.0 * PI * static_cast<float>(rec_link[1] & 0x3FFF) / maxEncoderCount ;
    // float vcom_ang = 2.0 * PI * static_cast<float>(rec_link[2] & 0x3FFF) / maxEncoderCount ;
    // float hcom_ang = 2.0 * PI * static_cast<float>(*((uint16_t*)(rec_link+2)) & 0x3FFF) / maxEncoderCount ;
    // float vcom_ang = 2.0 * PI * static_cast<float>(*((uint16_t*)(rec_link+4)) & 0x3FFF) / maxEncoderCount ;
    // float hcom_ang = 2.0 * PI * static_cast<float>(((rec_link[2] & 0x3F) << 8) | rec_link[3]) / maxEncoderCount ;
    // float vcom_ang = 2.0 * PI * static_cast<float>(((rec_link[4] & 0x3F) << 8) | rec_link[5]) / maxEncoderCount ;

    // Serial.print("parity: ");
    // Serial.print(rec_link[0]);
    // Serial.print(", hcom: ");
    // Serial.print(hcom_ang);
    // Serial.print(", vcom: ");
    // Serial.println(vcom_ang);

    // Serial.print("val1: ");
    // Serial.print(rec_link[0], HEX);
    // Serial.print(", val2: ");
    // Serial.print(rec_link[1], HEX);
    // Serial.print(", val3: ");
    // Serial.println(rec_link[2], HEX);

    // Serial.print("val1: ");
    // Serial.print(snd_link[0]);
    // Serial.print(", val2: ");
    // Serial.print(snd_link[1]);
    // Serial.print(", val3: ");
    // Serial.println(snd_link[3]);

    // unsigned char * p=((unsigned char *)(snd_link+1));
    // Serial.println(*p, HEX);

    // Serial.print("state: ");
    // Serial.println(recv_state);
  }

  //Serial.println(IMXRT_DMA_TCD[4].CSR, BIN);
}