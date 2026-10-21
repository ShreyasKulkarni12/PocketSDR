//
//  Pocket SDR - Dump IF data from external frontend via EZ-USB FX2LP.
//
//  Device sends 1 byte/sample (bit2=SIGN, bit1=MAGH, bit0=MAGL). Default
//  output packs 4 x 2-bit samples (SIGN,MAGH) per byte, first sample in
//  bit7-6.
//
//  History:
//  2026-10-19  1.0  new
//  2026-10-21  1.1  add -s (status), -x (self-test) and -B (release 8051)
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <libusb-1.0/libusb.h>
#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#endif

// constants -------------------------------------------------------------------
#define PROG_NAME   "fe_dump"
#define DEV_VID     0x04B4      // USB vendor ID
#define DEV_PID     0x1004      // USB product ID
#define DEV_IF      0           // USB interface
#define DEV_EP      0x86        // USB bulk IN end point
#define VR_STAT     0x40        // USB vendor request: get device info and status
#define VR_START    0x44        // USB vendor request: start bulk transfer
#define VR_IFCLK    0x47        // USB vendor request: IFCLK source (0:ext,1:int)
#define VR_BOOT     0x4F        // USB vendor request: release 8051 for F/W reload
#define VR_TYPE     (LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | \
                     LIBUSB_ENDPOINT_OUT)
#define VR_TYPE_IN  (LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_VENDOR | \
                     LIBUSB_ENDPOINT_IN)
#define NUM_XFER    16          // number of USB transfers in flight
#define SIZE_XFER   (256*1024)  // size of USB transfer (bytes)

#define FMT_PACK2   0           // output: 4 x 2-bit samples per byte
#define FMT_RAW     1           // output: raw 1 byte/sample
#define FMT_INT8    2           // output: int8 per sample (+-1,+-3)

// global variables ------------------------------------------------------------
static volatile int stop = 0;   // stop flag
static int fmt = FMT_PACK2;     // output format
static FILE *fp_out = NULL;     // output file
static long long n_byte = 0;    // number of received bytes
static int n_active = 0;        // number of active transfers
static int err_xfer = 0;        // transfer error flag
static unsigned char acc = 0;   // pack accumulator
static int n_acc = 0;           // samples in accumulator
static unsigned char obuf[SIZE_XFER]; // output buffer

static const char *usage =
    "usage: " PROG_NAME " [-t sec] [-r | -i] [-q] [-s | -x | -B] [file]\n"
    "  -t sec  capture time in seconds (default: 0 = until Ctrl-C)\n"
    "  -r      raw output, 1 byte/sample (bit2=SIGN,bit1=MAGH,bit0=MAGL)\n"
    "  -i      int8 output, 1 byte/sample (+1,+3,-1,-3)\n"
    "  -q      suppress status output\n"
    "  -s      print device/FIFO status and exit (shows if IFCLK edges arrive)\n"
    "  -x      self-test: use FX2LP internal 48MHz IFCLK (disconnect ADC CLK!)\n"
    "  -B      release 8051 so F/W can be reloaded without jumpers, then exit\n"
    "  file    output file (default: fe_dump.bin, '-': stdout)\n";

// signal handler --------------------------------------------------------------
static void sig_func(int sig)
{
    stop = 1;
    signal(sig, sig_func);
}

// convert received data to output format and write ----------------------------
static void write_data(const unsigned char *buff, int len)
{
    int i, n = 0;

    if (fmt == FMT_RAW) {
        fwrite(buff, 1, len, fp_out);
        return;
    }
    for (i = 0; i < len; i++) {
        int s = (buff[i] >> 1) & 3; // (SIGN,MAGH)
        if (fmt == FMT_INT8) {
            obuf[n++] = (unsigned char)(signed char)((s & 1 ? 3 : 1) * (s & 2 ? -1 : 1));
            continue;
        }
        acc = (unsigned char)((acc << 2) | s);
        if (++n_acc == 4) {
            obuf[n++] = acc;
            acc = 0;
            n_acc = 0;
        }
    }
    fwrite(obuf, 1, n, fp_out);
}

// USB transfer callback -------------------------------------------------------
static void LIBUSB_CALL xfer_cb(struct libusb_transfer *xfer)
{
    if (xfer->status == LIBUSB_TRANSFER_COMPLETED) {
        write_data(xfer->buffer, xfer->actual_length);
        n_byte += xfer->actual_length;
    }
    else if (xfer->status != LIBUSB_TRANSFER_CANCELLED &&
             xfer->status != LIBUSB_TRANSFER_TIMED_OUT) {
        fprintf(stderr, "transfer error (status=%d)\n", xfer->status);
        err_xfer = 1;
    }
    if (!stop && !err_xfer && libusb_submit_transfer(xfer) == 0) return;
    n_active--;
}

// print device and EP6 FIFO status --------------------------------------------
static int print_status(libusb_device_handle *h)
{
    unsigned char b[8] = {0};
    int i;
    
    if (libusb_control_transfer(h, VR_TYPE_IN, VR_STAT, 0, 0, b, 8, 1000) < 8) {
        fprintf(stderr, "status read error (old F/W?)\n");
        return 1;
    }
    for (i = 0; i < 3; i++) {
        printf("F/W ver=%d.%d  bulk=%d  EP6CS=0x%02X (%s%s npak=%d)  EP6FIFOFLGS=0x%02X  "
               "EP6 FIFO bytes=%d\n", b[0] >> 4, b[0] & 15, b[3], b[4],
            b[4] & 4 ? "EMPTY " : "", b[4] & 8 ? "FULL " : "", (b[4] >> 4) & 7, b[5],
            (b[6] << 8) | b[7]);
        libusb_control_transfer(h, VR_TYPE_IN, VR_STAT, 0, 0, b, 8, 1000);
    }
    printf("FIFO FULL/NPAK>0 while host not reading = IFCLK edges are arriving.\n"
           "FIFO EMPTY = no IFCLK edges (check clock wire, GND, RDY1, level).\n");
    return 0;
}

// main ------------------------------------------------------------------------
int main(int argc, char **argv)
{
    libusb_context *ctx = NULL;
    libusb_device_handle *h = NULL;
    struct libusb_transfer *xfer[NUM_XFER] = {0};
    struct timeval tv = {0, 100000};
    const char *file = "fe_dump.bin";
    double tmax = 0.0;
    int i, ret = 1, quiet = 0, status = 0, selftest = 0, boot = 0;
    time_t t0, t1 = 0;
    long long n_prev = 0;

    for (i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-t") && i + 1 < argc) tmax = atof(argv[++i]);
        else if (!strcmp(argv[i], "-r")) fmt = FMT_RAW;
        else if (!strcmp(argv[i], "-i")) fmt = FMT_INT8;
        else if (!strcmp(argv[i], "-q")) quiet = 1;
        else if (!strcmp(argv[i], "-s")) status = 1;
        else if (!strcmp(argv[i], "-x")) selftest = 1;
        else if (!strcmp(argv[i], "-B")) boot = 1;
        else if (argv[i][0] == '-' && argv[i][1]) {
            fprintf(stderr, "%s", usage);
            return 1;
        }
        else file = argv[i];
    }
    if (status || boot) file = NULL;
    if (!file) {
        fp_out = NULL;
    }
    else if (!strcmp(file, "-")) {
#ifdef _WIN32
        _setmode(_fileno(stdout), _O_BINARY);
#endif
        fp_out = stdout;
    }
    else if (!(fp_out = fopen(file, "wb"))) {
        fprintf(stderr, "file open error: %s\n", file);
        return 1;
    }
    if (libusb_init(&ctx)) {
        fprintf(stderr, "libusb_init error\n");
        goto exit;
    }
    if (!(h = libusb_open_device_with_vid_pid(ctx, DEV_VID, DEV_PID))) {
        fprintf(stderr, "no device %04X:%04X (check connection/driver/permission)\n",
            DEV_VID, DEV_PID);
        goto exit;
    }
    if (libusb_claim_interface(h, DEV_IF)) {
        fprintf(stderr, "libusb_claim_interface error\n");
        goto exit;
    }
    if (status) {
        ret = print_status(h);
        goto exit;
    }
    if (boot) {
        libusb_control_transfer(h, VR_TYPE, VR_BOOT, 0, 0, NULL, 0, 1000);
        printf("8051 released. Device now serves F/W load, e.g.:\n"
               "  fx2tool -d 04b4:1004 -B program -W 2 -V 04b4 -P 1004 -f fe_ext_fw.ihex\n");
        ret = 0;
        goto exit;
    }
    if (selftest) {
        libusb_control_transfer(h, VR_TYPE, VR_IFCLK, 1, 0, NULL, 0, 1000);
    }
    libusb_control_transfer(h, VR_TYPE, VR_START, 0, 0, NULL, 0, 1000);

    for (i = 0; i < NUM_XFER; i++) {
        unsigned char *buff = (unsigned char *)malloc(SIZE_XFER);
        if (!buff || !(xfer[i] = libusb_alloc_transfer(0))) {
            fprintf(stderr, "memory allocation error\n");
            free(buff);
            stop = 1;
            break;
        }
        libusb_fill_bulk_transfer(xfer[i], h, DEV_EP, buff, SIZE_XFER, xfer_cb,
            NULL, 0);
        if (libusb_submit_transfer(xfer[i])) {
            fprintf(stderr, "libusb_submit_transfer error\n");
            stop = 1;
            break;
        }
        n_active++;
    }
    signal(SIGINT, sig_func);
    signal(SIGTERM, sig_func);
    t0 = time(NULL);

    while (n_active > 0) {
        time_t t = time(NULL);
        if (!stop && (err_xfer || (tmax > 0.0 && difftime(t, t0) >= tmax))) {
            stop = 1;
        }
        if (stop) {
            for (i = 0; i < NUM_XFER; i++) {
                if (xfer[i]) libusb_cancel_transfer(xfer[i]);
            }
        }
        if (!quiet && t != t1) {
            fprintf(stderr, "\r%6.0f s  %12lld bytes  %7.3f MB/s", difftime(t, t0),
                n_byte, (n_byte - n_prev) * 1e-6 / (t1 ? difftime(t, t1) : 1.0));
            n_prev = n_byte;
            t1 = t;
        }
        libusb_handle_events_timeout(ctx, &tv);
    }
    if (!quiet) fprintf(stderr, "\n");
    ret = err_xfer;

exit:
    for (i = 0; i < NUM_XFER; i++) {
        if (!xfer[i]) continue;
        free(xfer[i]->buffer);
        libusb_free_transfer(xfer[i]);
    }
    if (h) {
        if (selftest) {
            libusb_control_transfer(h, VR_TYPE, VR_IFCLK, 0, 0, NULL, 0, 1000);
        }
        libusb_release_interface(h, DEV_IF);
        libusb_close(h);
    }
    if (ctx) libusb_exit(ctx);
    if (fp_out && fp_out != stdout) fclose(fp_out);
    return ret;
}
