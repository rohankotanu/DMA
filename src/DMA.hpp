#ifndef DMAUART_h
#define DMAUART_h

#include <Arduino.h>

HardwareSerialIMXRT serial_channels[8] = {Serial1, Serial2, Serial3, Serial4, Serial5, Serial6, Serial7, Serial8};
int lpuart_channels[8] = {6, 4, 2, 3, 8, 1, 7, 5};
int tx_enable_pins[8] = {54, 0, 18, 41, 0, 0, 0, 49};
int TX_DMAMUX_SOURCES[8] = {DMAMUX_SOURCE_LPUART1_TX, DMAMUX_SOURCE_LPUART2_TX, DMAMUX_SOURCE_LPUART3_TX, DMAMUX_SOURCE_LPUART4_TX, DMAMUX_SOURCE_LPUART5_TX, DMAMUX_SOURCE_LPUART6_TX, DMAMUX_SOURCE_LPUART7_TX, DMAMUX_SOURCE_LPUART8_TX};
int RX_DMAMUX_SOURCES[8] = {DMAMUX_SOURCE_LPUART1_RX, DMAMUX_SOURCE_LPUART2_RX, DMAMUX_SOURCE_LPUART3_RX, DMAMUX_SOURCE_LPUART4_RX, DMAMUX_SOURCE_LPUART5_RX, DMAMUX_SOURCE_LPUART6_RX, DMAMUX_SOURCE_LPUART7_RX, DMAMUX_SOURCE_LPUART8_RX};

class DMA {

  public:
  
    DMA() {
        CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);   // enables DMA clock if not already enabled
        __enable_irq();

        // turn on PIT
        PIT_MCR = 0x00;
    }

    void addUARTEncoder(const int serial_ch, const void* read_cmd, volatile void* dest) {
        int tx_ch = next_dma_ch_;
        int rx_ch = next_dma_ch_ + 16;
        next_dma_ch_++;

        HardwareSerialIMXRT s = serial_channels[serial_ch - 1];
        IMXRT_LPUART_t* LPUART = (IMXRT_LPUART_t*)(0x40184000 + (lpuart_channels[serial_ch-1] - 1) * 0x4000);

        s.begin(SERIAL_8N1);
        (*LPUART).BAUD = 0xBA00001;
        (*LPUART).CTRL = 0x300000;
        (*LPUART).DATA = 0x1000;
        (*LPUART).MODIR = 0x6;
        (*LPUART).FIFO = 0xC10099;
        (*LPUART).WATER = 0x10000;
        (*LPUART).CTRL |= 0b11 << 18;
        *(portConfigRegister(tx_enable_pins[serial_ch-1])) = 2;
        
        uint32_t* TX_CHANNEL_CONFIG = (uint32_t*)(0x400EC000 + tx_ch * 0x04);
        uint32_t* RX_CHANNEL_CONFIG = (uint32_t*)(0x400EC000 + rx_ch * 0x04);
        
        // Init DMA channels that send the encoder read command over each respective encoder UART channel
        *TX_CHANNEL_CONFIG = ((0b1 << 31) | (0b1 << 30) | TX_DMAMUX_SOURCES[lpuart_channels[serial_ch-1]-1]); // Set ENBL (bit 31), TRIG (bit 30), and SOURCE (bits 0-6)
        IMXRT_DMA_TCD[tx_ch].CITER = 1;
        IMXRT_DMA_TCD[tx_ch].BITER = 1;
        IMXRT_DMA_TCD[tx_ch].NBYTES = 1;
        IMXRT_DMA_TCD[tx_ch].SADDR = read_cmd;
        IMXRT_DMA_TCD[tx_ch].SOFF = 0; // Adjustment to the source address made after every minor loop iteration
        IMXRT_DMA_TCD[tx_ch].SLAST = 0; // Source address adjustment after major loop completion
        IMXRT_DMA_TCD[tx_ch].DADDR = &((*LPUART).DATA);
        IMXRT_DMA_TCD[tx_ch].DOFF = 0;
        IMXRT_DMA_TCD[tx_ch].DLASTSGA = 0;
        IMXRT_DMA_TCD[tx_ch].ATTR = 0;
        IMXRT_DMA_TCD[tx_ch].CSR = 0;
        DMA_SERQ = tx_ch;  // enable channel

        // Init DMA channels that transfer the encoder responses out of the UART recieve buffer
        *RX_CHANNEL_CONFIG = ((0b1 << 31) | RX_DMAMUX_SOURCES[lpuart_channels[serial_ch-1]-1]);
        IMXRT_DMA_TCD[rx_ch].CITER = 1;
        IMXRT_DMA_TCD[rx_ch].BITER = 1;
        IMXRT_DMA_TCD[rx_ch].NBYTES = 2;
        IMXRT_DMA_TCD[rx_ch].SADDR = &((*LPUART).DATA);
        IMXRT_DMA_TCD[rx_ch].SOFF = 0; // Adjustment to the source address made after every minor loop iteration
        IMXRT_DMA_TCD[rx_ch].SLAST = 0; // Source address adjustment after major loop completion
        IMXRT_DMA_TCD[rx_ch].DADDR = dest;
        IMXRT_DMA_TCD[rx_ch].DOFF = 1;
        IMXRT_DMA_TCD[rx_ch].DLASTSGA = -2;
        IMXRT_DMA_TCD[rx_ch].ATTR = 0;
        IMXRT_DMA_TCD[rx_ch].CSR = 0;
        DMA_SERQ = rx_ch;

        // If the channel has an associated PIT timer, start the timer
        if (tx_ch >= 0 && tx_ch <= 3) {
            uint32_t* PIT_LDVAL = (uint32_t*)(0x40084100 + tx_ch * 0x10);
            uint32_t* PIT_TCTRL = (uint32_t*)(0x40084108 + tx_ch * 0x10);

            // Timer 1
            *PIT_LDVAL = 2400; // setup timer 1 for 2400 cycles (24 MHz clock / 2400 clock cycles = 1 kHz timer)
            *PIT_TCTRL |= 0b11; // enable Timer 1 interrupts and start timer
        }
    }

  private:
    int next_dma_ch_ = 0;
};
    
#endif
