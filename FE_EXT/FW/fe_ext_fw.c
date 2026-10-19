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
//  Build: SDCC + libfx2 (https://github.com/whitequark/libfx2), see Makefile.
//
//  References:
//
//  [1] Cypress, EZ-USB Technical Refrence Manual, Rev.G , January 31, 2019
//
//  History:
//  2026-10-19  1.0  new, derived from FE_2CH/FW/v2.1/pocket_fw.c
//
#include <fx2lib.h>
#include <fx2regs.h>
#include <fx2delay.h>
#include <fx2usb.h>

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
#define VR_RESET     0x46       // USB vendor request: Reset FIFO and restart

// USB descriptors -------------------------------------------------------------
usb_desc_device_c usb_device = {
  .bLength            = sizeof(struct usb_desc_device),
  .bDescriptorType    = USB_DESC_DEVICE,
  .bcdUSB             = 0x0200,
  .bDeviceClass       = USB_DEV_CLASS_PER_INTERFACE,
  .bDeviceSubClass    = USB_DEV_SUBCLASS_PER_INTERFACE,
  .bDeviceProtocol    = USB_DEV_PROTOCOL_PER_INTERFACE,
  .bMaxPacketSize0    = 64,
  .idVendor           = 0x04B4,
  .idProduct          = 0x1004,
  .bcdDevice          = VER_FW,
  .iManufacturer      = 1,
  .iProduct           = 2,
  .iSerialNumber      = 0,
  .bNumConfigurations = 1,
};

usb_desc_interface_c usb_iface = {
  .bLength            = sizeof(struct usb_desc_interface),
  .bDescriptorType    = USB_DESC_INTERFACE,
  .bInterfaceNumber   = 0,
  .bAlternateSetting  = 0,
  .bNumEndpoints      = 1,
  .bInterfaceClass    = USB_IFACE_CLASS_VENDOR,
  .bInterfaceSubClass = USB_IFACE_SUBCLASS_VENDOR,
  .bInterfaceProtocol = USB_IFACE_PROTOCOL_VENDOR,
  .iInterface         = 0,
};

usb_desc_endpoint_c usb_ep6_in = {
  .bLength            = sizeof(struct usb_desc_endpoint),
  .bDescriptorType    = USB_DESC_ENDPOINT,
  .bEndpointAddress   = 6 | USB_DIR_IN,
  .bmAttributes       = USB_XFER_BULK,
  .wMaxPacketSize     = 512,
  .bInterval          = 0,
};

usb_configuration_c usb_config = {
  {
    .bLength             = sizeof(struct usb_desc_configuration),
    .bDescriptorType     = USB_DESC_CONFIGURATION,
    .bNumInterfaces      = 1,
    .bConfigurationValue = 1,
    .iConfiguration      = 0,
    .bmAttributes        = USB_ATTR_RESERVED_1,
    .bMaxPower           = 50,
  },
  {
    { .interface = &usb_iface },
    { .endpoint  = &usb_ep6_in },
    { 0 }
  }
};

usb_configuration_set_c usb_configs[] = {
  &usb_config,
};

usb_ascii_string_c usb_strings[] = {
  [0] = "Pocket SDR",
  [1] = "Pocket SDR FE EXT",
};

usb_descriptor_set_c usb_descriptor_set = {
  .device       = &usb_device,
  .config_count = ARRAYSIZE(usb_configs),
  .configs      = usb_configs,
  .string_count = ARRAYSIZE(usb_strings),
  .strings      = usb_strings,
};

static volatile uint8_t bulk_on = 0; // Bulk transfer status (0:stop,1:start)
static volatile uint8_t req_cmd = 0; // Pending vendor request for main loop

// start bulk trasfer ----------------------------------------------------------
static void start_bulk(void) {
  FIFORESET  = _NAKALL;  SYNCDELAY; // NAK-ALL
  EP6FIFOCFG = 0;        SYNCDELAY; // manual mode
  FIFORESET  = _NAKALL|6; SYNCDELAY; // reset EP6 FIFO
  EP6FIFOCFG = _AUTOIN|_ZEROLENIN; SYNCDELAY; // AUTOIN=ON, WORDWIDE=BYTE
  FIFORESET  = 0;        SYNCDELAY; // release NAK-ALL
  bulk_on = 1;
}

// stop bulk trasfer -----------------------------------------------------------
static void stop_bulk(void) {
  EP6FIFOCFG = _ZEROLENIN; SYNCDELAY; // AUTOIN=OFF, WORDWIDE=BYTE
  bulk_on = 0;
}

// setup FX2LP slave FIFO ------------------------------------------------------
static void setup(void) {
  CPUCS        = _CLKSPD1; SYNCDELAY; // CPU: 48MHz, CLKOUT=OFF
  REVCTL       = _ENH_PKT|_DYN_OUT; SYNCDELAY; // SLAVE-FIFO enhanced
  IFCONFIG     = _IFCFG1|_IFCFG0|(CLK_POL ? _IFCLKPOL : 0); SYNCDELAY; // IFCLK=EXT, SYNC, SLAVE_FIFO
  PORTACFG     = 0;     SYNCDELAY; // PA7: no SLCS
  FIFOPINPOLAR = 0;     SYNCDELAY; // SLWR,SLRD,SLOE,PKTEND=ACT_L
  EP1INCFG    &= ~_VALID; SYNCDELAY;
  EP1OUTCFG   &= ~_VALID; SYNCDELAY;
  EP2CFG      &= ~_VALID; SYNCDELAY;
  EP4CFG      &= ~_VALID; SYNCDELAY;
  EP6CFG       = _VALID|_DIR|_TYPE1; SYNCDELAY; // EP6: IN, BULK, 512, BUF=4X
  EP8CFG      &= ~_VALID; SYNCDELAY;
  EP2FIFOCFG   = 0;     SYNCDELAY; // WORDWIDE=BYTE for all (PD free)
  EP4FIFOCFG   = 0;     SYNCDELAY;
  EP8FIFOCFG   = 0;     SYNCDELAY;
  EP6AUTOINLENH = 0x02; SYNCDELAY; // EP6AUTOIN: PACKETLEN=512
  EP6AUTOINLENL = 0x00; SYNCDELAY;
  start_bulk();
}

// handle USB vendor request ---------------------------------------------------
//
//  USB vendor request      code dir wValue   bytes data
//
//  Get device Info         0x40  I  -           4  Device info and status
//  Start bulk transfer     0x44  O  -           0  -
//  Stop bulk transfer      0x45  O  -           0  -
//  Reset FIFO and restart  0x46  O  -           0  -
//
void handle_usb_setup(__xdata struct usb_req_setup *req) {
  if (req->bmRequestType == (USB_RECIP_DEVICE|USB_TYPE_VENDOR|USB_DIR_IN) &&
      req->bRequest == VR_STAT) {
    EP0BUF[0] = VER_FW;             // F/W version
    EP0BUF[1] = F_TCXO >> 8;        // TCXO Frequency (kHz)
    EP0BUF[2] = F_TCXO & 0xFF;
    EP0BUF[3] = bulk_on;            // Bulk transfer status (0:stop,1:start)
    SETUP_EP0_IN_BUF(4);
    return;
  }
  if (req->bmRequestType == (USB_RECIP_DEVICE|USB_TYPE_VENDOR|USB_DIR_OUT) &&
      (req->bRequest == VR_START || req->bRequest == VR_STOP ||
       req->bRequest == VR_RESET)) {
    req_cmd = req->bRequest;
    ACK_EP0();
    return;
  }
  STALL_EP0();
}

// main ------------------------------------------------------------------------
int main(void) {
  setup();
  usb_init(/*disconnect=*/true);

  while (1) {
    uint8_t cmd = req_cmd;
    if (!cmd) continue;
    req_cmd = 0;
    if (cmd == VR_START) start_bulk();
    else if (cmd == VR_STOP) stop_bulk();
    else if (cmd == VR_RESET) setup();
  }
}
