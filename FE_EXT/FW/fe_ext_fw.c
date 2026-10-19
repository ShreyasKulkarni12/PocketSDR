//-----------------------------------------------------------------------------
//
//  Pocket SDR - EZ-USB FX2LP firmware for external real-sampling frontend
//
//  Captures the frontend ADC outputs on every ADC CLK edge into the EP6 bulk
//  IN FIFO (slave FIFO, 8-bit, external IFCLK). One byte per sample:
//
//    FD2 (PB2) <- SIGN,  FD1 (PB1) <- MAGH,  FD0 (PB0) <- MAGL
//    FD7-FD3   <- GND,   IFCLK     <- ADC CLK
//
//  2-bit mode uses (SIGN,MAGH) = bit2-1 of each byte. Packing 4 x 2-bit
//  samples into one byte is done by the host (fe_dump/fe_dump.py), since the
//  8051 core is too slow to pack samples at MHz rates.
//
//  Static straps (no control signals from frontend):
//
//    RDY1/SLWR -> GND,  RDY0/SLRD -> 3.3V,  PA2/SLOE -> 3.3V,
//    PA4/FIFOADR0 -> GND,  PA5/FIFOADR1 -> 3.3V,  PA6/PKTEND -> 3.3V
//
//  Sampling runs only while ADC CLK is applied to IFCLK.
//
//  References:
//
//  [1] Cypress, EZ-USB Technical Refrence Manual, Rev.G , January 31, 2019
//
//  History:
//  2026-10-19  1.0  new, derived from FE_2CH/FW/v2.1/pocket_fw.c
//
#pragma NOIV

#include "Fx2.h"
#include "fx2regs.h"
#include "fx2sdly.h"

// constants and macros -------------------------------------------------------
#define VER_FW       0x01       // Firmware version
#ifndef F_TCXO
#define F_TCXO       26000      // Frontend TCXO frequency (kHz)
#endif
#ifndef CLK_POL
#define CLK_POL      0          // ADC CLK sampling edge (0:rising,1:falling)
#endif

#define VR_STAT      0x40       // USB vendor request: Get device info and status
#define VR_START     0x44       // USB vendor request: Start bulk transfer
#define VR_STOP      0x45       // USB vendor request: Stop bulk transfer
#define VR_RESET     0x46       // USB vendor request: Reset device

// type defintions -------------------------------------------------------------
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

static uint8_t bulk_on = 0;     // Bulk transfer status (0:stop,1:start)

// start bulk trasfer ----------------------------------------------------------
static void start_bulk(void) {
  FIFORESET  = 0x80; SYNCDELAY; // NAK-ALL
  EP6FIFOCFG = 0x00; SYNCDELAY; // manual mode
  FIFORESET  = 0x06; SYNCDELAY; // reset EP6 FIFO
  EP6FIFOCFG = 0x0C; SYNCDELAY; // EP6FIFO: AUTOIN=ON, ZEROLENIN=1, WORDWIDE=BYTE
  FIFORESET  = 0x00; SYNCDELAY; // release NAK-ALL
  bulk_on = 1;
}

// stop bulk trasfer -----------------------------------------------------------
static void stop_bulk(void) {
  EP6FIFOCFG = 0x04; SYNCDELAY; // EP6FIFO: AUTOIN=OFF, ZEROLENIN=1, WORDWIDE=BYTE
  bulk_on = 0;
}

// SETUP routine ---------------------------------------------------------------
void setup(void) {
  CPUCS         = 0x10; SYNCDELAY; // CPU: CLKSPD=48MHz, CLKOUT=OFF
  EP2FIFOCFG    = 0x00; SYNCDELAY; // EPxFIFO: WORDWIDE=BYTE (FD[15:8]=PD free)
  EP4FIFOCFG    = 0x00; SYNCDELAY;
  EP6FIFOCFG    = 0x00; SYNCDELAY;
  EP8FIFOCFG    = 0x00; SYNCDELAY;
  IFCONFIG      = 0x03 | (CLK_POL << 4); SYNCDELAY; // IFCLK=EXT, OUT_DIS, SYNC, SLAVE_FIFO
  REVCTL        = 0x03; SYNCDELAY; // SLAVE-FIFO enabled
  PORTACFG      = 0x00; SYNCDELAY; // PA7: no SLCS
  EP2CFG        = 0x20; SYNCDELAY; // EP2: OFF, DIR=OUT, TYPE=BULK
  EP4CFG        = 0x20; SYNCDELAY; // EP4: OFF, DIR=OUT, TYPE=BULK
  EP6CFG        = 0xE0; SYNCDELAY; // EP6: ON, DIR=IN, TYPE=BULK, SIZE=512, BUF=4X
  EP8CFG        = 0x60; SYNCDELAY; // EP8: OFF, DIR=IN, TYPE=BULK
  FIFOPINPOLAR  = 0x00; SYNCDELAY; // SLAVE_FIFO_IF: SLWR,SLRD,SLOE,PKTEND=ACT_L
  EP6AUTOINLENH = 0x02; SYNCDELAY; // EP6AUTOIN: PACKETLEN=512
  EP6AUTOINLENL = 0x00; SYNCDELAY;
  FIFORESET     = 0x86; SYNCDELAY; // EP6FIFO: RESET
  FIFORESET     = 0x00; SYNCDELAY;

  start_bulk();
}

// MAIN loop -------------------------------------------------------------------
void loop(void) {
}

// handle USB vendor request ---------------------------------------------------
//
//  USB vendor request      code dir wValue   bytes data
//
//  Get device Info         0x40  I  -           4  Device info and status
//  Start bulk transfer     0x44  O  -           0  -
//  Stop bulk transfer      0x45  O  -           0  -
//  Reset device            0x46  O  -           0  -
//
BOOL handle_req(void) {
  if (SETUPDAT[1] == VR_STAT) {
    EP0BUF[0] = VER_FW;             // F/W version
    EP0BUF[1] = MSB(F_TCXO);        // TCXO Frequency (kHz)
    EP0BUF[2] = LSB(F_TCXO);
    EP0BUF[3] = bulk_on;            // Bulk transfer status (0:stop,1:start)
    EP0BCH = 0;
    EP0BCL = 4;
  }
  else if (SETUPDAT[1] == VR_START) {
    EP0BCH = EP0BCL = 0;
    start_bulk();
  }
  else if (SETUPDAT[1] == VR_STOP) {
    EP0BCH = EP0BCL = 0;
    stop_bulk();
  }
  else if (SETUPDAT[1] == VR_RESET) {
    EP0BCH = EP0BCL = 0;
    stop_bulk();
    setup();
  }
  else {
    return TRUE; // undefined vendor request
  }
  return FALSE;
}

// Cypress firmware framework functions ----------------------------------------
extern BOOL GotSUD, Sleep, Rwuen;
BYTE config, altset;

// task dispatcher hooks
void TD_Init            (void) {Rwuen = TRUE; setup();}
void TD_Poll            (void) {loop();}
BOOL TD_Suspend         (void) {return TRUE;}
BOOL TD_Resume          (void) {return TRUE;}
BOOL DR_GetDescriptor   (void) {return TRUE;}
BOOL DR_GetStatus       (void) {return TRUE;}
BOOL DR_ClearFeature    (void) {return TRUE;}
BOOL DR_SetFeature      (void) {return TRUE;}
BOOL DR_VendorCmnd      (void) {return handle_req();}
BOOL DR_SetConfiguration(void) {config = SETUPDAT[2]; return TRUE;}
BOOL DR_SetInterface    (void) {altset = SETUPDAT[2]; return TRUE;}
BOOL DR_GetConfiguration(void) {EP0BUF[0] = config; EP0BCH = 0; EP0BCL = 1; return TRUE;}
BOOL DR_GetInterface    (void) {EP0BUF[0] = altset; EP0BCH = 0; EP0BCL = 1; return TRUE;}

// interrupt handlers
void ISR_Sudav(void) interrupt 0 {
  SUDPTRCTL |= bmSDPAUTO;
  GotSUD = TRUE;
  EZUSB_IRQ_CLEAR();
  USBIRQ = bmSUDAV;
}
void ISR_Sutok(void) interrupt 0 {
  EZUSB_IRQ_CLEAR();
  USBIRQ = bmSUTOK;
}
void ISR_Sof(void) interrupt 0 {
  EZUSB_IRQ_CLEAR();
  USBIRQ = bmSOF;
}
void ISR_Ures(void) interrupt 0 {
  if (EZUSB_HIGHSPEED()) {
      pConfigDscr = pHighSpeedConfigDscr;
      pOtherConfigDscr = pFullSpeedConfigDscr;
  }
  else {
    pConfigDscr = pFullSpeedConfigDscr;
    pOtherConfigDscr = pHighSpeedConfigDscr;
  }
  EZUSB_IRQ_CLEAR();
  USBIRQ = bmURES;
}
void ISR_Susp(void) interrupt 0 {
  Sleep = TRUE;
  EZUSB_IRQ_CLEAR();
  USBIRQ = bmSUSP;
}
void ISR_Highspeed(void) interrupt 0 {
  if (EZUSB_HIGHSPEED()) {
    pConfigDscr = pHighSpeedConfigDscr;
    pOtherConfigDscr = pFullSpeedConfigDscr;
  }
  else {
    pConfigDscr = pFullSpeedConfigDscr;
    pOtherConfigDscr = pHighSpeedConfigDscr;
  }
  EZUSB_IRQ_CLEAR();
  USBIRQ = bmHSGRANT;
}
void ISR_Ep0ack      (void) interrupt 0 {}
void ISR_Stub        (void) interrupt 0 {}
void ISR_Ep0in       (void) interrupt 0 {}
void ISR_Ep0out      (void) interrupt 0 {}
void ISR_Ep1in       (void) interrupt 0 {}
void ISR_Ep1out      (void) interrupt 0 {}
void ISR_Ep2inout    (void) interrupt 0 {}
void ISR_Ep4inout    (void) interrupt 0 {}
void ISR_Ep6inout    (void) interrupt 0 {}
void ISR_Ep8inout    (void) interrupt 0 {}
void ISR_Ibn         (void) interrupt 0 {}
void ISR_Ep0pingnak  (void) interrupt 0 {}
void ISR_Ep1pingnak  (void) interrupt 0 {}
void ISR_Ep2pingnak  (void) interrupt 0 {}
void ISR_Ep4pingnak  (void) interrupt 0 {}
void ISR_Ep6pingnak  (void) interrupt 0 {}
void ISR_Ep8pingnak  (void) interrupt 0 {}
void ISR_Errorlimit  (void) interrupt 0 {}
void ISR_Ep2piderror (void) interrupt 0 {}
void ISR_Ep4piderror (void) interrupt 0 {}
void ISR_Ep6piderror (void) interrupt 0 {}
void ISR_Ep8piderror (void) interrupt 0 {}
void ISR_Ep2pflag    (void) interrupt 0 {}
void ISR_Ep4pflag    (void) interrupt 0 {}
void ISR_Ep6pflag    (void) interrupt 0 {}
void ISR_Ep8pflag    (void) interrupt 0 {}
void ISR_Ep2eflag    (void) interrupt 0 {}
void ISR_Ep4eflag    (void) interrupt 0 {}
void ISR_Ep6eflag    (void) interrupt 0 {}
void ISR_Ep8eflag    (void) interrupt 0 {}
void ISR_Ep2fflag    (void) interrupt 0 {}
void ISR_Ep4fflag    (void) interrupt 0 {}
void ISR_Ep6fflag    (void) interrupt 0 {}
void ISR_Ep8fflag    (void) interrupt 0 {}
void ISR_GpifComplete(void) interrupt 0 {}
void ISR_GpifWaveform(void) interrupt 0 {}

// end of Cypress firmware framework functions ---------------------------------
