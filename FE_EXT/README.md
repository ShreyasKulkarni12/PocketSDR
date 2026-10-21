# **Pocket SDR FE EXT - EZ-USB FX2LP F/W for External Real-Sampling Frontend**

## **Overview**

Firmware and host tools to stream samples from an external GNSS frontend
(own TCXO, own ADC) through a Cypress EZ-USB FX2LP (CY7C68013A) to a PC.

* ADC CLK of the frontend drives FX2LP IFCLK. Every ADC CLK rising edge, the
  FX2LP latches FD[7:0] into the EP6 bulk IN FIFO (slave FIFO, 8-bit,
  synchronous). No ADC CLK, no samples.
* One byte per sample on USB: bit2 = SIGN, bit1 = MAGH, bit0 = MAGL.
* 2-bit mode: host tools take (SIGN, MAGH) and pack 4 samples per byte.
* No control signals from frontend. FX2LP FIFO control pins are static straps.

Why packing is on host: 8051 core of FX2LP runs ~12 MIPS. Bit-packing per
sample in firmware tops out well under 1 Msps. Slave FIFO hardware moves one
byte per IFCLK with zero CPU work, up to 48 MHz IFCLK. Host packs 4:1 for free.
If USB bandwidth must drop 4x, pack in hardware (e.g. 2 x 74HC164 shift
registers + divide-by-4 of ADC CLK into IFCLK); this firmware works unchanged
with that.

--------------------------------------------------------------------------------

## **Directory Structure and Contents**
```
FE_EXT --+-- FW
         |   +-- fe_ext_fw.c     F/W source (SDCC + libfx2)
         |   +-- fe_ext_fw.ihex  prebuilt F/W, Intel HEX (fx2tool)
         |   +-- fe_ext_fw.iic   prebuilt F/W, EEPROM C2 image (CyControl)
         |   +-- Makefile        F/W build
         |   +-- hex2iic.py      Intel HEX to .iic converter
         |
         +-- util
             +-- fe_dump.c       C dump tool (libusb-1.0, async, full rate)
             +-- fe_dump.py      Python dump tool (pyusb, simple, lower rate)
             +-- makefile        makefile for fe_dump
```
--------------------------------------------------------------------------------

## **Connections**

All signals 3.3 V CMOS. Common GND between frontend and FX2LP board. Keep
ADC CLK wire short; 22-33 ohm series resistor at clock source helps.

| Frontend      | FX2LP pin         | Note                                   |
|---------------|-------------------|----------------------------------------|
| ADC CLK       | IFCLK             | 5 - 48 MHz (FX2LP external IFCLK range)|
| SIGN          | PB2 / FD2         |                                        |
| MAGH          | PB1 / FD1         |                                        |
| MAGL          | PB0 / FD0         | unused in 2-bit mode, tie GND if absent|
| GND           | GND               |                                        |

FX2LP static straps (on FX2LP board, direct wire or 1-10 kohm):

| FX2LP pin       | Tie to | Meaning                         |
|-----------------|--------|---------------------------------|
| PB7-PB3 / FD7-3 | GND    | unused data bits                |
| RDY1 / SLWR     | GND    | write always enabled (act. low) |
| RDY0 / SLRD     | 3.3 V  | read inactive                   |
| PA2 / SLOE      | 3.3 V  | FD output disabled              |
| PA4 / FIFOADR0  | GND    | FIFOADR = 10b selects EP6       |
| PA5 / FIFOADR1  | 3.3 V  |                                 |
| PA6 / PKTEND    | 3.3 V  | inactive                        |

FX2LP keeps its own 24 MHz crystal. Frontend 26 MHz TCXO is not connected to
FX2LP; only ADC CLK goes to IFCLK.

Timing: FX2LP samples FD at the selected IFCLK edge (setup ~9 ns, hold ~0 ns,
see FX2LP datasheet). Default is rising edge. If ADC outputs change on the
rising edge, rebuild with `CLK_POL` = 1 (falling edge) in `fe_ext_fw.c`.

Boot EEPROM: C2 boot needs I2C EEPROM at address 0x51 with 2-byte addressing
(24C32 .. 24C512, e.g. 24C128 on common CY7C68013A boards). 24C02 too small.

Blue CY7C68013A-56 mini board (LCSoft type, 24 MHz crystal, 8-pin EEPROM,
EEPROM jumper): all needed pins are on the two 2.54 mm headers, labelled
IFCLK, RDY0, RDY1, PA2, PA4, PA5, PA6, PB0-PB7. Use IFCLK, not CLK (CLKOUT).
Read EEPROM marking: 24C128 or 24C64/24C32 OK, 24C02 must be replaced.
EEPROM jumper off at power-up gives blank boot (`04B4:8613`) for recovery;
put jumper back before writing EEPROM.

--------------------------------------------------------------------------------

## **Build F/W**

Prebuilt `FW/fe_ext_fw.ihex` and `FW/fe_ext_fw.iic` are included (rising edge,
04B4:1004). Rebuild only to change options. Toolchain is free: SDCC [4] +
libfx2 [5]. No Keil needed.

Linux (Debian/Ubuntu):
```
sudo apt install sdcc make python3 git
cd FE_EXT/FW
git clone https://github.com/whitequark/libfx2
make
```
Windows (MSYS2 UCRT64 shell [3]):
```
pacman -S mingw-w64-ucrt-x86_64-sdcc make python git
cd FE_EXT/FW
git clone https://github.com/whitequark/libfx2
make
```
If MSYS2 has no sdcc package, install SDCC for Windows [4] and add its `bin`
to PATH. Output: `fe_ext_fw.ihex` and `fe_ext_fw.iic`. Tested with SDCC 4.2.0,
libfx2 commit 16cdf94.

Options: `make clean && make CFLAGS="-DSYNCDELAYLEN=16 -DCLK_POL=1"`
* `CLK_POL=1` sample on falling edge.
* `F_TCXO=26000` TCXO kHz reported by vendor request 0x40 (info only).

--------------------------------------------------------------------------------

## **Write F/W to EEPROM**

Blank FX2LP (EEPROM empty or EEPROM jumper open at power-up) enumerates as
`04B4:8613`. After flashing, device enumerates as `04B4:1004` (same as Pocket
SDR FE 2CH). To reflash a board whose firmware misbehaves: unplug, open EEPROM
jumper (or lift EEPROM SDA), plug in (now `04B4:8613`), close jumper, flash.

### Option A: fx2tool (Linux and Windows)

```
pip install fx2
fx2tool -d 04b4:8613 -B program -W 2 -V 04b4 -P 1004 -f fe_ext_fw.ihex
```
Unplug and replug. `lsusb` (Linux) shows `04b4:1004`.

Linux permission: run with `sudo`, or install udev rules (from repo root):
```
sudo cp driver/99-pocket-sdr.rules /etc/udev/rules.d/
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="04b4", ATTRS{idProduct}=="8613", MODE="0660", GROUP="plugdev", TAG+="uaccess"' | sudo tee /etc/udev/rules.d/98-fx2-blank.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG plugdev $USER     # re-login
```
Windows: bind WinUSB driver to `04B4:8613` with Zadig [2] first.

### Option B: Cypress USB Control Center (Windows)

1. Bind Cypress CyUSB driver to the device (see `driver/readme.txt`; the
   EZ-USB FX3 SDK driver also covers FX2LP).
2. Run CyControl.exe (`FE_2CH\FW\v2.1\cypress\CyControl.exe`, or USB
   Control Center from the FX3 SDK).
3. Select device, menu Program - FX2 - 64KB EEPROM, select
   `FE_EXT\FW\fe_ext_fw.iic`.
4. Status bar "Programming succeeded." Replug.

--------------------------------------------------------------------------------

## **Host Driver**

* Linux / macOS: libusb-1.0, no kernel driver. udev rule above for non-root.
* Windows: dump tools use libusb, so `04B4:1004` needs WinUSB driver. Run
  Zadig [2], Options - List All Devices, pick `04B4 1004`, driver WinUSB,
  Replace Driver. (This replaces CyUSB for that device; CyControl then no
  longer sees it. Use fx2tool to reflash, or switch driver back.)

--------------------------------------------------------------------------------

## **Dump Data**

Output formats:
* default `pack2`: 4 samples per byte, sample n in bits 7-6, n+1 in 5-4,
  n+2 in 3-2, n+3 in 1-0. Each 2-bit field = (SIGN << 1) | MAGH.
* `-r` raw: 1 byte/sample as from USB (bit2 SIGN, bit1 MAGH, bit0 MAGL), keeps
  MAGL for 3-bit use.
* `-i` int8: 1 byte/sample, (SIGN,MAGH) = 00:+1, 01:+3, 10:-1, 11:-3. Sign
  polarity only flips carrier phase by 180 deg, no effect on acquisition or
  tracking.

Rate shown on screen must equal ADC CLK in MB/s (raw bytes). Lower rate means
clock missing or USB overrun. FX2LP drops samples silently on overrun (SLWR
tied, FIFO full ignored).

### C tool (recommended, full rate, async libusb)

Linux (Debian/Ubuntu/Raspberry Pi OS):
```
sudo apt install build-essential libusb-1.0-0-dev
cd FE_EXT/util
make
./fe_dump -t 10 ifdata.bin          # 10 s, pack2
./fe_dump -r ifdata_raw.bin         # raw, until Ctrl-C
```
Windows (MSYS2 UCRT64 shell [3]):
```
pacman -S mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-libusb make
cd FE_EXT/util
make
./fe_dump.exe -t 10 ifdata.bin
```
macOS: `brew install libusb`, then `make CFLAGS="-std=c99 -O2 -I/opt/homebrew/include" LDLIBS="-L/opt/homebrew/lib -lusb-1.0"`.

### Python tool (simple, lower rate)

```
pip install pyusb numpy libusb-package
python fe_dump.py -t 10 ifdata.bin
python fe_dump.py -i ifdata_int8.bin
```
`libusb-package` bundles the libusb-1.0 DLL for Windows; on Linux system
libusb-1.0 (`sudo apt install libusb-1.0-0`) also works. Python reads
synchronously; at high sample rates (> ~10 Msps) gaps between reads can
overflow FX2LP FIFO (2 KB). Use C tool for full rate.

### Read pack2 file in Python
```python
import numpy as np
b = np.fromfile('ifdata.bin', dtype=np.uint8)
s = np.stack([(b >> 6) & 3, (b >> 4) & 3, (b >> 2) & 3, b & 3], axis=1).ravel()
x = np.array([1, 3, -1, -3], dtype=np.int8)[s]   # int8 samples
```

--------------------------------------------------------------------------------

## **Troubleshooting: no data (0 MB/s)**

1. `./fe_dump -s` (or `python fe_dump.py -s`) prints EP6 FIFO status without
   reading data. With ADC CLK running on IFCLK and the straps correct, FIFO
   fills while the host is not reading: `EP6CS ... FULL`, npak=4. `EMPTY`
   means the chip sees no IFCLK edges: check clock wire on IFCLK, common GND,
   RDY1 to GND, RDY0/PA2/PA5/PA6 to 3.3 V, PA4 to GND, clock level >= 2 V,
   frequency 5-48 MHz.
2. Self-test: disconnect ADC CLK wire from IFCLK, then `./fe_dump -x -t 2 t.bin`.
   The FX2LP drives its own 48 MHz onto IFCLK pin, data flows if USB, FIFO
   and straps are right (rate 30-40 MB/s, samples are bus noise). IFCLK is
   restored to external on exit. Never run `-x` with ADC CLK connected.
3. After self-test passes, reconnect ADC CLK and repeat step 1.

F/W 1.1 or later is needed for `-s`, `-x` and `-B`. `./fe_dump -B` releases the
8051 so F/W can be reloaded without touching jumpers:
```
./fe_dump -B
fx2tool -d 04b4:1004 -B program -W 2 -V 04b4 -P 1004 -f fe_ext_fw.ihex
```
(`-B` is untested on hardware; if it fails, use the jumper method above.)

--------------------------------------------------------------------------------

## **USB Vendor Requests (optional)**

Firmware starts streaming at power-up; requests not needed.

| Request             | Code | Dir | Data                                     |
|---------------------|------|-----|------------------------------------------|
| Get device info     | 0x40 | IN  | 8 B: F/W ver, TCXO kHz (MSB, LSB), bulk status, EP6CS, EP6FIFOFLGS, EP6 FIFO bytes (MSB, LSB) |
| Start bulk transfer | 0x44 | OUT | - (also resets EP6 FIFO)                 |
| Stop bulk transfer  | 0x45 | OUT | -                                        |
| Reset device        | 0x46 | OUT | -                                        |
| IFCLK source        | 0x47 | OUT | wValue 0: external (ADC CLK), 1: internal 48 MHz on IFCLK pin (self-test) |
| Release 8051        | 0x4F | OUT | - (device re-enumerates; `fx2tool` can reload F/W, no jumpers) |

Dump tools send 0x44 at start, so FIFO restarts clean even if ADC CLK came up
after USB enumeration.

--------------------------------------------------------------------------------

## **References**

[1] Cypress, CY3684 EZ-USB FX2LP Development Kit

[2] Zadig, https://zadig.akeo.ie/

[3] MSYS2, https://www.msys2.org/

[4] SDCC, https://sdcc.sourceforge.net/

[5] libfx2, https://github.com/whitequark/libfx2
