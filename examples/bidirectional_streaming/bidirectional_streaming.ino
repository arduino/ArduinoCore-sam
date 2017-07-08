#include <DueTC.h>

// Requires modified USB files: CDC.cpp, USBCore.cpp, USBAPI.h.  
// These are found in the "board support package". On linux they usually
// install in a hidden file in the user's home directory, somewhere under:
// ~/.arduino15/
// and the above files are found in:
// .arduino15/packages/arduino/hardware/sam/1.6.11/cores/arduino/USB/

// Thanks to the following.
// https://github.com/ivanseidel/DueTimer/issues/11
// https://github.com/OliviliK/DueTC
// Note that in the following I've corrected an error in updating buffer pointers.
// https://gist.github.com/pklaus/5921022 
// https://forum.arduino.cc/index.php?topic=224672.0

// Header information:
// nS: Number of DAC samples
// adcF: ADC clock in Hz
// clkDiv: Divider for conversion timer (base clock is at 42 MHz)
// It seems that ADC clock frequency must be at least 42x the 
// conversion clock frequency.
uint32_t header [3];

// Quarter buffer size is in bytes.
const int qsize = CDC_SERIAL_BUFFER_SIZE/4;
// Pointer to the SerialUSB ring_buffer.
ring_buffer * rxb = &(SerialUSB.cdc_rx_buffer);
// Number of 2-byte samples remaining to queue.
uint32_t dacrem;
// Keep track of DMA buffer sizes **in samples**.
uint32_t dacbs1, dacbs2;
// Receive count in samples.
uint32_t rxc = 0;

bool live = false;
const uint16_t hold = 2047;
#define HOLDBUF_SIZE 256
uint16_t holdbuf[HOLDBUF_SIZE];

// Size of each buffer in samples.
const int bufsize = 1024;
// Number of buffers. Use a power of two to enable "and" which is quicker than generic "mod".
const int nbuf = 4;
volatile int adcbufn, adcobufn;
// Buffers of bufsize readings
uint16_t adcbuf[nbuf][bufsize];
// Number of ADC channels.
uint32_t nadcChan = 2;

// ADC samples remaining to be scheduled.
uint32_t adcrem;
// ADC samples transmitted.
uint32_t txc;

bool errorFlag = 0;

void blockingRead(uint8_t * b, uint32_t n) {
  while (n) {
    n -= SerialUSB.read(b, n);
  } 
}

void blockingWrite(uint8_t * b, uint32_t n) {
  while (n) {
    n -= SerialUSB.write(b, n);
  } 
}

void signalLED(int o = 1) {
  digitalWrite(13, o);
}

void ADC_Handler(){     // move DMA pointers to next buffer
  int f=ADC->ADC_ISR;
  // 1<<27 is ENDRX buffer.
  if (f&(1<<27)){     
    adcbufn++;
    ADC->ADC_RNPR=(uint32_t)adcbuf[(adcbufn+1)%nbuf];
    ADC->ADC_RNCR=bufsize;
    if (adcrem > bufsize) {
      adcrem -= bufsize;
    } else {
      adcrem = 0;
    }
    if (adcbufn > adcobufn + nbuf) errorFlag = true;
  } 
}

void setupADC(uint32_t adcF=42000000) {
  // Prepare analog input.
  analogReadResolution(12);
  pmc_enable_periph_clk(ID_ADC);
  // Max 42000000 for ADC frequency. Seems to need 42 cycles per conversion.
  adc_init(ADC, SystemCoreClock, adcF, ADC_STARTUP_FAST);
  // Disable hardware triggers.
  ADC->ADC_MR &= ~1;
  // 0x80000000 = 1<<31 sets USEQ; User Sequence?
  ADC->ADC_MR |= ADC_TRIG_TIO_CH_0 | 1<<31;
  // Alternate A0 and A1. Twisted SAM -> DUE channel mapping.
  ADC->ADC_CHER=0xC0;
  ADC->ADC_SEQR1 = 0x67000000;
  
  adcbufn = 0;
  adcobufn = 0;
  txc = 0;
  ADC->ADC_RPR = (uint32_t)adcbuf[0];   // DMA buffer
  ADC->ADC_RCR = bufsize;
  adcrem -= bufsize;
  ADC->ADC_RNPR = (uint32_t)adcbuf[1]; // next DMA buffer
  ADC->ADC_RNCR = bufsize;
  adcrem -= bufsize;
  // I think this is PDC receiver transfer enable RXTEN.
  ADC->ADC_PTCR=1; 
  // Enable hardware trigger.
  ADC->ADC_MR |= 1;
  // Enable trigger after writing to RCR or RNCR, otherwise spurious ENDRX interrupt
  // is raised.
  ADC->ADC_IER=1<<27;
  ADC->ADC_IDR=~(1<<27);
  NVIC_EnableIRQ(ADC_IRQn);
}

// A non-zero value must be written to either TCR or TNCR to clear ENDTX interrupt 
// and prevent immediate refiring:
// http://community.atmel.com/forum/how-can-you-do-peripheral-dma-fixed-number-blocks
void DACC_Handler() {
  if (dacc_get_interrupt_status(DACC) & DACC_ISR_ENDTX) { 
    if (live) {
      // We just got through a buffer.
      rxb->tail += dacbs1*2;
      rxc += dacbs1;
      if (rxb->tail > rxb->head) errorFlag = true;
      dacbs1 = dacbs2;
      if (dacbs1 == 0) live = false;
      if (dacrem) {
        DACC->DACC_TNPR = (uint32_t) &(rxb->buffer[(rxb->tail + dacbs2*2)%CDC_SERIAL_BUFFER_SIZE]);
        dacbs2 = min(dacrem, qsize/2);
        DACC->DACC_TNCR = dacbs2;
        dacrem -= dacbs2;
      } else {
        DACC->DACC_TNPR = (uint32_t) &holdbuf;
        DACC->DACC_TNCR = HOLDBUF_SIZE;
        dacbs2 = 0;
      }
    } else {
      DACC->DACC_TNPR = (uint32_t) &holdbuf;
      DACC->DACC_TNCR = HOLDBUF_SIZE;          
    }
  }
}

void setupDAC(uint32_t clkDiv=84) {
  // Prepare analog input on DAC0.
  pinMode(DAC0, OUTPUT);
  analogWriteResolution(12);
  pmc_enable_periph_clk (DACC_INTERFACE_ID) ; // start clocking DAC
  dacc_reset(DACC);
  // Controls whether "word" mode (32-bit or 16-bit transfers?). 0 -> 16 bits.
  dacc_set_transfer_mode(DACC, 0);
  dacc_set_power_save(DACC, 0, 1);            // sleep = 0, fastwkup = 1
  dacc_set_analog_control(DACC, DACC_ACR_IBCTLCH0(0x02) | DACC_ACR_IBCTLCH1(0x02) | DACC_ACR_IBCTLDACCORE(0x01));
  dacc_set_trigger(DACC, 1); // 1 means TC0.
  dacc_set_channel_selection(DACC, 0);
  dacc_enable_channel(DACC, 0);

  DACC->DACC_TPR = (uint32_t) &holdbuf; // DMA buffer  
  DACC->DACC_TCR = HOLDBUF_SIZE;
  DACC->DACC_TNPR = (uint32_t) &holdbuf;
  DACC->DACC_TNCR = HOLDBUF_SIZE;
  DACC->DACC_PTCR =  0x00000100;
  dacc_enable_interrupt(DACC, DACC_IER_ENDTX);
  NVIC_EnableIRQ(DACC_IRQn);
}

void prepareOutput() {
  rxc = 0;
  DACC->DACC_TPR = (uint32_t) &(rxb->buffer[rxb->tail%CDC_SERIAL_BUFFER_SIZE]); // DMA buffer
  uint32_t fillq = (qsize - rxb->tail%qsize)/2;
  dacbs1 = min(dacrem, fillq);
  DACC->DACC_TCR = dacbs1;
  dacrem -= dacbs1;
  if (dacrem) { 
    DACC->DACC_TNPR = (uint32_t) &(rxb->buffer[(rxb->tail+(dacbs1*2))%CDC_SERIAL_BUFFER_SIZE]);
    dacbs2 = min(dacrem, qsize/2);
    DACC->DACC_TNCR = dacbs2;
    dacrem -= dacbs2;
  } else {
    dacbs2 = 0;
  }
  live = true;   
}

void stopTimer() {
  // Stop timer.
  REG_TC0_CCR0=2;
}

void startTimer(uint32_t clkDiv=42) {
  setupTC_Pin2_Timing(clkDiv,0);  
}

void setup() { 
  // Error signal.
  pinMode(13, OUTPUT);
  // For checking TC0.
  pinMode(2, OUTPUT);
  // Output trigger if required.
  pinMode(52, OUTPUT);
  digitalWrite(52, LOW);
  // Input trigger if required.
  pinMode(22, INPUT);
  digitalWrite(13, LOW);
  for (int i = 0; i < 256; i++) {
    holdbuf[i] = hold;
  }
  setupDAC();
  startTimer();
  SerialUSB.begin(0);
  while(!SerialUSB);
  SerialUSB.enableInterrupts(); 
}

void loop() {
  // Get header
  blockingRead((uint8_t*) header, 12);
  // Number of samples.
  uint32_t nS = header[0];
  // ADC clock frequency.
  uint32_t adcF = header[1];
  // Conversion clock frequency divider.
  uint32_t clkDiv = header[2];
  blockingWrite((uint8_t*) &nadcChan, 4);
  dacrem = nS;
  adcrem = nadcChan*nS;
  // Make sure that DAC data is primed.
  uint32_t threshold = min(nS*2, 3*CDC_SERIAL_BUFFER_SIZE/4);
  while(SerialUSB.available() < threshold) SerialUSB.accept();
  errorFlag = false;
  digitalWrite(13, LOW);
  // Wait for trigger on pin 22. The following code will delay start somewhat,
  // but the DAC output would drift if not refreshed.
  // while (digitalRead(22)); // Or !digitalRead(22)
  stopTimer();
  prepareOutput();
  setupADC(adcF);
  digitalWrite(52, HIGH);
  startTimer(clkDiv);
  // Transfer until all input and output is accounted for.
  while (txc < nadcChan*nS | rxc < nS) {
    SerialUSB.accept();
    if (adcbufn > adcobufn) { 
      uint32_t n = min((nadcChan*nS-txc), bufsize);
      blockingWrite((uint8_t*) adcbuf[adcobufn%nbuf], n*2);//
      txc += n;
      adcobufn += 1;
    }
    if (errorFlag) signalLED();
  }
  digitalWrite(52, LOW);
  SerialUSB.write((uint8_t*) &errorFlag, 1);//
}
