#!/usr/bin/env python3
#
#  Pocket SDR Python AP - Dump IF data from external frontend via EZ-USB FX2LP
#
#  Device sends 1 byte/sample (bit2=SIGN, bit1=MAGH, bit0=MAGL). Default
#  output packs 4 x 2-bit samples (SIGN,MAGH) per byte, first sample in
#  bit7-6.
#
#  History:
#  2026-10-19  1.0  new
#  2026-10-21  1.1  add -s (status), -x (self-test), -p (pins) and -f (falling edge)
#
import sys, time, signal, argparse, threading, queue
import numpy as np
import usb.core, usb.util

# constants --------------------------------------------------------------------
DEV_VID   = 0x04B4      # USB vendor ID
DEV_PID   = 0x1004      # USB product ID
DEV_IF    = 0           # USB interface
DEV_EP    = 0x86        # USB bulk IN end point
VR_STAT   = 0x40        # USB vendor request: get device info and status
VR_START  = 0x44        # USB vendor request: start bulk transfer
VR_IFCLK  = 0x47        # USB vendor request: IFCLK source (0:ext,1:int)
VR_PINS   = 0x48        # USB vendor request: read PA, PB, RDY pin levels
SIZE_READ = 1 << 20     # size of USB read (bytes)

# int8 sample values for (SIGN,MAGH) = 0,1,2,3 ---------------------------------
LUT_INT8 = np.array([1, 3, -1, -3], dtype=np.int8)

# get libusb backend (libusb-package bundles libusb-1.0 DLL for Windows) -------
def get_backend():
    try:
        import libusb_package
        return libusb_package.get_libusb1_backend()
    except ImportError:
        return None

# open device ------------------------------------------------------------------
def open_dev(start=True, mode=0):
    dev = usb.core.find(idVendor=DEV_VID, idProduct=DEV_PID, backend=get_backend())
    if dev is None:
        raise SystemExit('no device %04X:%04X (check connection/driver/permission)' %
            (DEV_VID, DEV_PID))
    try:
        dev.set_configuration()
    except usb.core.USBError:
        pass
    usb.util.claim_interface(dev, DEV_IF)
    if start:
        if mode:
            dev.ctrl_transfer(0x40, VR_IFCLK, mode, 0, None, 1000)
        dev.ctrl_transfer(0x40, VR_START, 0, 0, None, 1000)
    return dev

# print pin levels and check straps --------------------------------------------
def print_pins(dev):
    chk = [('RDY0 (SLRD)', 2, 0, 1), ('RDY1 (SLWR)', 2, 1, 0),
           ('PA2  (SLOE)', 0, 2, 1), ('PA4  (FIFOADR0)', 0, 4, 0),
           ('PA5  (FIFOADR1)', 0, 5, 1), ('PA6  (PKTEND)', 0, 6, 1)]
    hi, lo = 0, 0xFF
    for i in range(50):
        b = bytes(dev.ctrl_transfer(0xC0, VR_PINS, 0, 0, 6, 1000))
        if len(b) < 6:
            raise SystemExit('pin read error (needs F/W 1.2 or later)')
        hi, lo = hi | b[4], lo & b[4]
    print('%-18s expect  read' % 'strap')
    for name, port, bit, expect in chk:
        v = (b[3 + port] >> bit) & 1
        print('%-18s %d       %d  %s' % (name, expect, v, 'OK' if v == expect else 'BAD'))
    print('\nPB (data) bits over 50 reads: 7..0 = ' + ''.join(
        '0' if not (hi >> i) & 1 else '1' if (lo >> i) & 1 else 'T'
        for i in range(7, -1, -1)) + '   (0=stuck low, 1=stuck high, T=toggling)')
    print('SIGN=PB2, MAGH=PB1, MAGL=PB0. Unused PB3-PB7 should be 0.')

# print device and EP6 FIFO status ---------------------------------------------
def print_status(dev):
    for i in range(3):
        b = bytes(dev.ctrl_transfer(0xC0, VR_STAT, 0, 0, 8, 1000))
        if len(b) < 8:
            raise SystemExit('status read error (old F/W?)')
        print('F/W ver=%d.%d  bulk=%d  EP6CS=0x%02X (%s%s npak=%d)  EP6FIFOFLGS=0x%02X  '
            'EP6 FIFO bytes=%d' % (b[0] >> 4, b[0] & 15, b[3], b[4],
            'EMPTY ' if b[4] & 4 else '', 'FULL ' if b[4] & 8 else '',
            (b[4] >> 4) & 7, b[5], (b[6] << 8) | b[7]))
    print('FIFO FULL/NPAK>0 while host not reading = IFCLK edges are arriving.\n'
        'FIFO EMPTY = no IFCLK edges (check clock wire, GND, RDY1, level).')

# USB read thread --------------------------------------------------------------
def read_thread(dev, que, stop):
    while not stop.is_set():
        buff = usb.util.create_buffer(SIZE_READ)
        try:
            n = dev.read(DEV_EP, buff, 1000)
        except usb.core.USBTimeoutError:
            continue
        except usb.core.USBError as e:
            print('\nUSB read error: %s' % e, file=sys.stderr)
            break
        que.put(np.frombuffer(buff, dtype=np.uint8, count=n))
    que.put(None)

# convert raw samples to output format -----------------------------------------
def conv_data(data, fmt, rem):
    if fmt == 'raw':
        return data.tobytes(), rem
    s = (data >> 1) & 3 # (SIGN,MAGH)
    if fmt == 'int8':
        return LUT_INT8[s].tobytes(), rem
    s = np.concatenate((rem, s))
    n = len(s) // 4 * 4
    p = s[:n].reshape(-1, 4)
    out = (p[:,0] << 6) | (p[:,1] << 4) | (p[:,2] << 2) | p[:,3]
    return out.astype(np.uint8).tobytes(), s[n:]

# main -------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description='Dump IF data from external frontend')
    ap.add_argument('-t', type=float, default=0.0,
        help='capture time in seconds (default: 0 = until Ctrl-C)')
    ap.add_argument('-r', action='store_true',
        help='raw output, 1 byte/sample (bit2=SIGN,bit1=MAGH,bit0=MAGL)')
    ap.add_argument('-i', action='store_true',
        help='int8 output, 1 byte/sample (+1,+3,-1,-3)')
    ap.add_argument('-q', action='store_true', help='suppress status output')
    ap.add_argument('-s', action='store_true',
        help='print device/FIFO status and exit (shows if IFCLK edges arrive)')
    ap.add_argument('-p', action='store_true',
        help='print PA/PB/RDY pin levels and check straps (needs no clock)')
    ap.add_argument('-f', action='store_true',
        help='sample on ADC CLK falling edge (default: rising)')
    ap.add_argument('-x', action='store_true',
        help='self-test: use FX2LP internal 48MHz IFCLK (disconnect ADC CLK!)')
    ap.add_argument('file', nargs='?', default='fe_dump.bin',
        help="output file (default: fe_dump.bin, '-': stdout)")
    args = ap.parse_args()
    fmt = 'raw' if args.r else 'int8' if args.i else 'pack2'

    if args.s or args.p:
        dev = open_dev(start=False)
        print_pins(dev) if args.p else print_status(dev)
        usb.util.release_interface(dev, DEV_IF)
        usb.util.dispose_resources(dev)
        return
    fp = sys.stdout.buffer if args.file == '-' else open(args.file, 'wb')
    dev = open_dev(mode=1 if args.x else 2 if args.f else 0)
    que, stop = queue.Queue(), threading.Event()
    th = threading.Thread(target=read_thread, args=(dev, que, stop), daemon=True)
    signal.signal(signal.SIGINT, lambda sig, frame: stop.set())
    th.start()

    rem = np.zeros(0, dtype=np.uint8)
    t0 = t1 = time.time()
    n_byte = n_prev = 0
    while True:
        try:
            data = que.get(timeout=0.2)
        except queue.Empty:
            data = np.zeros(0, dtype=np.uint8)
        if data is None:
            break
        out, rem = conv_data(data, fmt, rem)
        fp.write(out)
        n_byte += len(data)
        t = time.time()
        if args.t > 0 and t - t0 >= args.t:
            stop.set()
        if not args.q and t - t1 >= 1.0:
            print('\r%6.0f s  %12d bytes  %7.3f MB/s' % (t - t0, n_byte,
                (n_byte - n_prev) * 1e-6 / (t - t1)), end='', file=sys.stderr)
            n_prev, t1 = n_byte, t
    if not args.q:
        print('', file=sys.stderr)
    if args.x or args.f:
        dev.ctrl_transfer(0x40, VR_IFCLK, 3, 0, None, 1000)
    usb.util.release_interface(dev, DEV_IF)
    usb.util.dispose_resources(dev)
    if fp is not sys.stdout.buffer:
        fp.close()

if __name__ == '__main__':
    main()
