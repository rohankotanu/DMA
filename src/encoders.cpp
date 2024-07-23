#include <Arduino.h>
#include "DMA.hpp"

// Define and set the command sent to the encoders to trigger a position reading
constexpr uint8_t encoder_command = 0x54;
volatile uint16_t hcom, vcom;

// Init LINK buffers
uint8_t pos_encoders[6];
uint8_t com_encoders[4];
uint8_t rec_link[6];

const float maxEncoderCount = pow(2, 14);

DMA dma;

int i = 0;
uint8_t state = 3;
uint8_t recv_state = 0;

void setup() {
  pos_encoders[4] = 21;

  dma.addUARTEncoder(4, &encoder_command, pos_encoders+1); // HPOS
  dma.addUARTEncoder(8, &encoder_command, pos_encoders+3); // VPOS
  dma.addUARTEncoder(1, &encoder_command, com_encoders+1); // HCOM
  dma.addUARTEncoder(3, &encoder_command, com_encoders+3); // VCOM

  dma.addLinkChannel(6, pos_encoders, rec_link);
  //dma.addStateChannel(7, &state, &recv_state);

  DMA_CERQ = 17;
  IMXRT_DMA_TCD[17].CSR |= (0x4 << 8) | (0b1 <<5);
  DMA_SERQ = 17;

  // IMXRT_LPUART_t* LPUART = (IMXRT_LPUART_t*)(0x40184000 + (5) * 0x4000); // Pointer to the LPUART registers corresponding to the desired HardwareSerial port

  // uint32_t temp;
  // (*LPUART).CTRL &= ~(0b11 << 18);
  // while ((*LPUART).CTRL & (0b11<<18)) {;}
  // // while (rec_link[0] != 0x0015) {
  // //   temp = (*LPUART).DATA;
  // // }
  // (*LPUART).CTRL |= (0b11<<18);
  Serial.begin(115200);
  while (rec_link[4] != 21) {
    Serial.println(rec_link[4]);
    DMA_CERQ = 20;
    delayNanoseconds(10);
    DMA_SERQ = 20;
  }

}

void loop() {
  
  if ((i++ % 1000) == 0) {

    float hpos_ang = 2.0 * PI * static_cast<float>(((pos_encoders[0] & 0x3F) << 8) | pos_encoders[1]) / maxEncoderCount ;
    float vpos_ang = 2.0 * PI * static_cast<float>(((pos_encoders[2] & 0x3F) << 8) | pos_encoders[3]) / maxEncoderCount ;
    float hcom_ang = 2.0 * PI * static_cast<float>(((com_encoders[0] & 0x3F) << 8) | com_encoders[1]) / maxEncoderCount ;
    float vcom_ang = 2.0 * PI * static_cast<float>(((com_encoders[2] & 0x3F) << 8) | com_encoders[3]) / maxEncoderCount ;
    float hpos_recv = 2.0 * PI * static_cast<float>(((rec_link[0] & 0x3F) << 8) | rec_link[1]) / maxEncoderCount ;
    float vpos_recv = 2.0 * PI * static_cast<float>(((rec_link[2] & 0x3F) << 8) | rec_link[3]) / maxEncoderCount ;

    //if (rec_link[4] == 21) {
      Serial.print("hpos: ");
      Serial.print(hpos_ang);
      Serial.print(", vpos: ");
      Serial.print(vpos_ang);
      Serial.print(", hcom: ");
      Serial.print(hcom_ang);
      Serial.print(", vcom: ");
      Serial.print(vcom_ang);
      Serial.print(", hpos_recv: ");
      Serial.print(hpos_recv);
      Serial.print(", vpos_recv: ");
      Serial.println(vpos_recv);
    //}

    //Serial.println(DMA_DCHPRI10);
  }
}