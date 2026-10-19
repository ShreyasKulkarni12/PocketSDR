#!/usr/bin/env python3
#
#  Convert Intel HEX firmware to EZ-USB FX2LP EEPROM C2 boot image (.iic)
#
#  usage: hex2iic.py [-v VID] [-p PID] input.ihex output.iic
#
import sys, argparse

MAX_REC = 1023 # max data length of a C2 record

# read Intel HEX file to {address: byte} ---------------------------------------
def read_ihex(file):
    mem, base = {}, 0
    for line in open(file):
        line = line.strip()
        if not line.startswith(':'):
            continue
        b = bytes.fromhex(line[1:])
        if sum(b) & 0xFF:
            raise SystemExit('checksum error: %s' % line)
        n, addr, typ, data = b[0], (b[1] << 8) | b[2], b[3], b[4:4+b[0]]
        if typ == 0:
            for i in range(n):
                mem[base + addr + i] = data[i]
        elif typ == 1:
            break
        elif typ == 2:
            base = ((data[0] << 8) | data[1]) << 4
        elif typ == 4:
            base = ((data[0] << 8) | data[1]) << 16
    return mem

# split memory into contiguous segments ----------------------------------------
def segments(mem):
    segs = []
    for addr in sorted(mem):
        if segs and addr == segs[-1][0] + len(segs[-1][1]) and len(segs[-1][1]) < MAX_REC:
            segs[-1][1].append(mem[addr])
        else:
            segs.append((addr, bytearray([mem[addr]])))
    return segs

# generate C2 boot image -------------------------------------------------------
def gen_iic(mem, vid, pid, did=0, cfg=0x00):
    out = bytearray([0xC2, vid & 0xFF, vid >> 8, pid & 0xFF, pid >> 8,
        did & 0xFF, did >> 8, cfg])
    for addr, data in segments(mem):
        out += bytes([len(data) >> 8, len(data) & 0xFF, addr >> 8, addr & 0xFF])
        out += data
    out += bytes([0x80, 0x01, 0xE6, 0x00, 0x00]) # CPUCS = 0: release 8051 reset
    return out

# main -------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description='Intel HEX to FX2LP C2 EEPROM image')
    ap.add_argument('-v', type=lambda s: int(s, 16), default=0x04B4, help='VID (hex)')
    ap.add_argument('-p', type=lambda s: int(s, 16), default=0x1004, help='PID (hex)')
    ap.add_argument('input')
    ap.add_argument('output')
    args = ap.parse_args()
    mem = read_ihex(args.input)
    if not mem or max(mem) >= 0x4000:
        raise SystemExit('invalid firmware address range')
    img = gen_iic(mem, args.v, args.p)
    open(args.output, 'wb').write(img)
    print('%s: %d bytes code, %d bytes image' % (args.output, len(mem), len(img)))

if __name__ == '__main__':
    main()
