#ifndef DMA_h
#define DMA_h

#include <Arduino.h>

// These are the HardwareSerial ports for the Teensy4.1
HardwareSerialIMXRT serial_channels[8] = {Serial1, Serial2, Serial3, Serial4, Serial5, Serial6, Serial7, Serial8};

// NOTE: The HardwareSerial port numbers do not match the LPUART numbers
// This array maps Serialx to LPUARTy
// Serial1 --> LPUART6, Serial2 --> LPUART4, etc.
int lpuart_channels[8] = {6, 4, 2, 3, 8, 1, 7, 5};

// On the PCB, these are the TX enable pins for each HardwareSerial port
int tx_enable_pins[8] = {54, 0, 18, 41, 0, 0, 0, 49};

// These arrays contain the DMAMUX channels for all the LPUART RX and TX sources
int TX_DMAMUX_SOURCES[8] = {DMAMUX_SOURCE_LPUART1_TX, DMAMUX_SOURCE_LPUART2_TX, DMAMUX_SOURCE_LPUART3_TX, DMAMUX_SOURCE_LPUART4_TX, DMAMUX_SOURCE_LPUART5_TX, DMAMUX_SOURCE_LPUART6_TX, DMAMUX_SOURCE_LPUART7_TX, DMAMUX_SOURCE_LPUART8_TX};
int RX_DMAMUX_SOURCES[8] = {DMAMUX_SOURCE_LPUART1_RX, DMAMUX_SOURCE_LPUART2_RX, DMAMUX_SOURCE_LPUART3_RX, DMAMUX_SOURCE_LPUART4_RX, DMAMUX_SOURCE_LPUART5_RX, DMAMUX_SOURCE_LPUART6_RX, DMAMUX_SOURCE_LPUART7_RX, DMAMUX_SOURCE_LPUART8_RX};

class DMA {

  public:
  
    /**
     * Instantiating a DMA object performs general setup required for all DMA channels
     */
    DMA() {
      // Enable DMA clock if not already enabled
        CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);

      // Enable interrupt requests
      __enable_irq();

      // Turn on PIT
      PIT_MCR = 0x00;
    }

    /**
     * Sets up a UART encoder over DMA.
     *
     * @param serial_ch The number of the HarwareSerial channel to be used.
     * @param read_cmd A pointer to the encoder's read command value.
     * @param dest A pointer to the variable in which the encoder readings should be stored.
     */
    void addUARTEncoder(const int serial_ch, const void* read_cmd, volatile void* dest) {
        int tx_ch = next_dma_ch_; // The DMA channel number that will be used for transmitting data over UART
        int rx_ch = next_dma_ch_ + 16; // The DMA channel number that will be used for receiving data over UART
        next_dma_ch_++; // Increment the next available DMA channel

        IMXRT_LPUART_t* LPUART = configureUART(serial_ch);
        
        // Addresses for DMA MUX Channel Config Registers
        uint32_t* TX_CHANNEL_CONFIG = (uint32_t*)(0x400EC000 + tx_ch * 0x04);
        uint32_t* RX_CHANNEL_CONFIG = (uint32_t*)(0x400EC000 + rx_ch * 0x04);
        
        /**** Init DMA TX channel that sends the encoder read command ****/

        // DMA MUX Channel Configuration Register (page 85)
        // DMA Mux Channel Enable (bit 31)
        // DMA Channel Trigger Enable: Set to periodic trigger mode (bit 30)
        // DMA Channel Source: Set to appropriate DMA_MUX_LPUART_TX source (bits 0-6)
        *TX_CHANNEL_CONFIG = ((0b1 << 31) | (0b1 << 30) | TX_DMAMUX_SOURCES[lpuart_channels[serial_ch-1]-1]);

        // Configure the TCD for the DMA channel
        IMXRT_DMA_TCD[tx_ch].CITER = 1;
        IMXRT_DMA_TCD[tx_ch].BITER = 1;
        IMXRT_DMA_TCD[tx_ch].NBYTES = 1;
        IMXRT_DMA_TCD[tx_ch].SADDR = read_cmd;
        IMXRT_DMA_TCD[tx_ch].SOFF = 0;
        IMXRT_DMA_TCD[tx_ch].SLAST = 0;
        IMXRT_DMA_TCD[tx_ch].DADDR = &((*LPUART).DATA);
        IMXRT_DMA_TCD[tx_ch].DOFF = 0;
        IMXRT_DMA_TCD[tx_ch].DLASTSGA = 0;
        IMXRT_DMA_TCD[tx_ch].ATTR = 0;
        IMXRT_DMA_TCD[tx_ch].CSR = 0;
        DMA_SERQ = tx_ch; // Enable channel

        /**** Init DMA RX channel that places the encoder reading in memory ****/

        // DMA MUX Channel Configuration Register (page 85)
        // DMA Mux Channel Enable (bit 31)
        // DMA Channel Source: Set to appropriate DMA_MUX_LPUART_RX source (bits 0-6)
        *RX_CHANNEL_CONFIG = ((0b1 << 31) | RX_DMAMUX_SOURCES[lpuart_channels[serial_ch-1]-1]);

        // Configure the TCD for the DMA channel
        IMXRT_DMA_TCD[rx_ch].CITER = 1;
        IMXRT_DMA_TCD[rx_ch].BITER = 1;
        IMXRT_DMA_TCD[rx_ch].NBYTES = 2;
        IMXRT_DMA_TCD[rx_ch].SADDR = &((*LPUART).DATA);
        IMXRT_DMA_TCD[rx_ch].SOFF = 0;
        IMXRT_DMA_TCD[rx_ch].SLAST = 0;
        IMXRT_DMA_TCD[rx_ch].DADDR = dest;
        IMXRT_DMA_TCD[rx_ch].DOFF = 1;
        IMXRT_DMA_TCD[rx_ch].DLASTSGA = -2;
        IMXRT_DMA_TCD[rx_ch].ATTR = 0;
        IMXRT_DMA_TCD[rx_ch].CSR = 0;
        DMA_SERQ = rx_ch; // Enable channel

        initPIT(tx_ch);
    }

    /**
     * Sets up a UART TX Link
     *
     * @param serial_ch The number of the HarwareSerial channel to be used.
     * @param src A pointer to the address of the encoder value(s) to be transmitted
     * @param dest A pointer to where the incoming encoder value(s) should be stored
     */
    void addLinkChannel(const int serial_ch, const void* src, volatile void* dest) {
        int tx_ch = next_dma_ch_; // The DMA channel number that will be used for transmitting data over UART
        int rx_ch = next_dma_ch_ + 16; // The DMA channel number that will be used for receiving data over UART
        next_dma_ch_++; // Increment the next available DMA channel

        IMXRT_LPUART_t* LPUART = configureUART(serial_ch);
        
        // Address for DMA MUX Channel Config Register
        uint32_t* TX_CHANNEL_CONFIG = (uint32_t*)(0x400EC000 + tx_ch * 0x04);
        uint32_t* RX_CHANNEL_CONFIG = (uint32_t*)(0x400EC000 + rx_ch * 0x04);
        
        /**** Init DMA TX channel that sends the encoder read command ****/

        // DMA MUX Channel Configuration Register (page 85)
        // DMA Mux Channel Enable (bit 31)
        // DMA Channel Trigger Enable: Set to periodic trigger mode (bit 30)
        // DMA Channel Source: Set to appropriate DMA_MUX_LPUART_TX source (bits 0-6)
        *TX_CHANNEL_CONFIG = ((0b1 << 31) | (0b1 << 30) | TX_DMAMUX_SOURCES[lpuart_channels[serial_ch-1]-1]);

        // Configure the TCD for the DMA channel
        IMXRT_DMA_TCD[tx_ch].CITER = 1;
        IMXRT_DMA_TCD[tx_ch].BITER = 1;
        IMXRT_DMA_TCD[tx_ch].NBYTES = 6;
        IMXRT_DMA_TCD[tx_ch].SADDR = src;
        IMXRT_DMA_TCD[tx_ch].SOFF = 1;
        IMXRT_DMA_TCD[tx_ch].SLAST = -6;
        IMXRT_DMA_TCD[tx_ch].DADDR = &((*LPUART).DATA);
        IMXRT_DMA_TCD[tx_ch].DOFF = 0;
        IMXRT_DMA_TCD[tx_ch].DLASTSGA = 0;
        IMXRT_DMA_TCD[tx_ch].ATTR = 0;
        IMXRT_DMA_TCD[tx_ch].CSR = 0;
        DMA_SERQ = tx_ch; // Enable channel

        /**** Init DMA RX channel that places the encoder reading in memory ****/

        // DMA MUX Channel Configuration Register (page 85)
        // DMA Mux Channel Enable (bit 31)
        // DMA Channel Source: Set to appropriate DMA_MUX_LPUART_RX source (bits 0-6)
        *RX_CHANNEL_CONFIG = ((0b1 << 31) | RX_DMAMUX_SOURCES[lpuart_channels[serial_ch-1]-1]);

        // Configure the TCD for the DMA channel
        IMXRT_DMA_TCD[rx_ch].CITER = 1;
        IMXRT_DMA_TCD[rx_ch].BITER = 1;
        IMXRT_DMA_TCD[rx_ch].NBYTES = 6;
        IMXRT_DMA_TCD[rx_ch].SADDR = &((*LPUART).DATA);
        IMXRT_DMA_TCD[rx_ch].SOFF = 0;
        IMXRT_DMA_TCD[rx_ch].SLAST = 0;
        IMXRT_DMA_TCD[rx_ch].DADDR = dest;
        IMXRT_DMA_TCD[rx_ch].DOFF = 1;
        IMXRT_DMA_TCD[rx_ch].DLASTSGA = -6;
        IMXRT_DMA_TCD[rx_ch].ATTR = 0;
        IMXRT_DMA_TCD[rx_ch].CSR = 0;
        DMA_SERQ = rx_ch; // Enable channel

        initPIT(tx_ch);
    }

    /**
     * Sets up a UART TX Link
     *
     * @param serial_ch The number of the HarwareSerial channel to be used.
     * @param src A pointer to the address of the encoder value(s) to be transmitted
     * @param dest A pointer to where the incoming encoder value(s) should be stored
     */
    void addStateChannel(const int serial_ch, const void* src, volatile void* dest) {
        int tx_ch = next_dma_ch_; // The DMA channel number that will be used for transmitting data over UART
        int rx_ch = next_dma_ch_ + 16; // The DMA channel number that will be used for receiving data over UART
        next_dma_ch_++; // Increment the next available DMA channel

        IMXRT_LPUART_t* LPUART = configureUART(serial_ch);
        
        // Address for DMA MUX Channel Config Register
        uint32_t* TX_CHANNEL_CONFIG = (uint32_t*)(0x400EC000 + tx_ch * 0x04);
        uint32_t* RX_CHANNEL_CONFIG = (uint32_t*)(0x400EC000 + rx_ch * 0x04);
        
        /**** Init DMA TX channel that sends the encoder read command ****/

        // DMA MUX Channel Configuration Register (page 85)
        // DMA Mux Channel Enable (bit 31)
        // DMA Channel Trigger Enable: Set to periodic trigger mode (bit 30)
        // DMA Channel Source: Set to appropriate DMA_MUX_LPUART_TX source (bits 0-6)
        *TX_CHANNEL_CONFIG = ((0b1 << 31) | (0b1 << 30) | TX_DMAMUX_SOURCES[lpuart_channels[serial_ch-1]-1]);

        // Configure the TCD for the DMA channel
        IMXRT_DMA_TCD[tx_ch].CITER = 1;
        IMXRT_DMA_TCD[tx_ch].BITER = 1;
        IMXRT_DMA_TCD[tx_ch].NBYTES = 1;
        IMXRT_DMA_TCD[tx_ch].SADDR = src;
        IMXRT_DMA_TCD[tx_ch].SOFF = 1;
        IMXRT_DMA_TCD[tx_ch].SLAST = -1;
        IMXRT_DMA_TCD[tx_ch].DADDR = &((*LPUART).DATA);
        IMXRT_DMA_TCD[tx_ch].DOFF = 0;
        IMXRT_DMA_TCD[tx_ch].DLASTSGA = 0;
        IMXRT_DMA_TCD[tx_ch].ATTR = 0;
        IMXRT_DMA_TCD[tx_ch].CSR = 0;
        DMA_SERQ = tx_ch; // Enable channel

        /**** Init DMA RX channel that places the encoder reading in memory ****/

        // DMA MUX Channel Configuration Register (page 85)
        // DMA Mux Channel Enable (bit 31)
        // DMA Channel Source: Set to appropriate DMA_MUX_LPUART_RX source (bits 0-6)
        *RX_CHANNEL_CONFIG = ((0b1 << 31) | RX_DMAMUX_SOURCES[lpuart_channels[serial_ch-1]-1]);

        // Configure the TCD for the DMA channel
        IMXRT_DMA_TCD[rx_ch].CITER = 1;
        IMXRT_DMA_TCD[rx_ch].BITER = 1;
        IMXRT_DMA_TCD[rx_ch].NBYTES = 1;
        IMXRT_DMA_TCD[rx_ch].SADDR = &((*LPUART).DATA);
        IMXRT_DMA_TCD[rx_ch].SOFF = 0;
        IMXRT_DMA_TCD[rx_ch].SLAST = 0;
        IMXRT_DMA_TCD[rx_ch].DADDR = dest;
        IMXRT_DMA_TCD[rx_ch].DOFF = 1;
        IMXRT_DMA_TCD[rx_ch].DLASTSGA = -1;
        IMXRT_DMA_TCD[rx_ch].ATTR = 0;
        IMXRT_DMA_TCD[rx_ch].CSR = 0;
        DMA_SERQ = rx_ch; // Enable channel

        initPIT(tx_ch);
    }

  private:
    // Counter to keep track of next available DMA channel
    int next_dma_ch_ = 0;

    IMXRT_LPUART_t* configureUART(const int serial_ch) {
      HardwareSerialIMXRT s = serial_channels[serial_ch - 1]; // Obtain the appropriate HardwareSerial port
      IMXRT_LPUART_t* LPUART = (IMXRT_LPUART_t*)(0x40184000 + (lpuart_channels[serial_ch-1] - 1) * 0x4000); // Pointer to the LPUART registers corresponding to the desired HardwareSerial port

      /**** Set Up UART Channel ****/

      // Set up the serial port
      s.begin(SERIAL_8N1);
      // LPUART BAUG Register (page 2920)
      // Oversampling Ratio: Set to 12 (bits 24-28)
      // Transmitter DMA Enable: Enable DMA requests (bit 23)
      // Receiver Full DMA Enable: Enable DMA requests (bit 21)
      // Baud Rate Modulo Divisor: Set to 1 (bits 0-12)
      (*LPUART).BAUD = (0b01011 << 24) | (0b1 << 23) | (0b1 << 21) | 0b1; 
      // LPUART CTRL Register (page 2926)
      // Receiver Interrupt Enable: Hardware interrupt is requested when RDRF flag is 1 (bit 21)
      // Idle Line Interrupt Enable: Hardware interrupt is requested when IDLE flag is 1 (bit 20)
      (*LPUART).CTRL = (0b11 << 20);
      // LPUART Modem IrDA Register (page 2934)
      // Transmitter request-to-send polarity: Transmitter RTS is active high (bit 2)
      // Transmitter request-to-send enable (bit 1)
      (*LPUART).MODIR = (0b11 << 1);
      // LPUART Watermark Register (page 2939)
      // Receive Watermark: Set to 1 (bits 16-17)
      (*LPUART).WATER = (0b01 << 16);
      // LPUART Control Register (page 2939)
      // Transmitter Enable (bit 19)
      // Receiver Enable (bit 18)
      (*LPUART).CTRL |= (0b11 << 18);
      // Specify TX Enable pin
      *(portConfigRegister(tx_enable_pins[serial_ch-1])) = 2;

      return LPUART;
    }

    void initPIT(const int ch) {
      // If the channel has an associated PIT timer, start the timer
      if (ch >= 0 && ch <= 3) {
          // Address of the appropriate PIT Timer Load Value Register
          uint32_t* PIT_LDVAL = (uint32_t*)(0x40084100 + ch * 0x10);
          // Address of the appropriate PIT Timer Control Register
          uint32_t* PIT_TCTRL = (uint32_t*)(0x40084108 + ch * 0x10);

          // Timer Load Value Register (page 3046)
          // Set up timer for 2400 cycles (24 MHz clock / 2400 clock cycles = 1 kHz timer)
          *PIT_LDVAL = 2400 - 1;

          // Timer Control Register (page 3048)
          // Timer Interrupt Enable: Interrupt is requested whenever TIF is set (bit 1)
          // Timer Enable (bit 0)
          *PIT_TCTRL = 0b11;
      }
    }
};
    
#endif
