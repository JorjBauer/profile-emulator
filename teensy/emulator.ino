#include <Arduino.h>
#include <SdFat.h>
#include "timing.h"
#include "selector.h"

/*
 * ProFile drive EMULATOR for the Teensy 4.1: CLEAN / FAST build.
 *
 * The Teensy pretends to be the ProFile DRIVE and answers a real Apple host (an
 * Apple /// or //e with its ProFile interface card), serving block images off the
 * built-in microSD. This is the scaffolding-free version: faithful protocol, fast
 * bus I/O, and only a once-a-second idle heartbeat for liveness. No recorder, no
 * settle delays, no probes.
 *
 * Wiring (see ../HARDWARE.md): the data '245's A-side = Teensy, B-side = ProFile.
 * Data bus D7..D0 = Teensy pins 2..9 (pin 2 = MSB). The data '245's DIR pin is wired
 * to PR/-W in HARDWARE, so the buffer reverses with the host automatically; our only
 * job is to make pins 2..9 follow PR/-W (INPUT when the host writes, OUTPUT when it
 * reads) so we never fight the '245. /OE (pin 11) enables both '245s and is held low.
 */

// ---- Pin map -------------------------------------------------------------
#define D0PIN   9   // pin 2 = D7 (MSB) ... pin 9 = D0 (LSB)
#define PH0IN   31  // host phase clock (buffered input; optional)
#define PRESIN  30  // /PRES  host reset   (active low, buffered input)
#define PCMDIN  29  // /PCMD  host command (active low, buffered input)
#define PSTRBIN 28  // /PSTRB host strobe  (active low, buffered input)
#define PRWIN   27  // PR/-W  host read/write; ALSO the hardware data-'245 DIR
#define PBSYOUT 25  // /PBSY  drive busy   (active low, DIRECT output)
#define PPAROUT 26  // /PPARITY drive parity (active low, DIRECT output)
#define OE      11  // /OE for both '245s  (active low)

// ---- Media ---------------------------------------------------------------
// Last-resort image candidates, tried in order, and ONLY if the card scan found
// nothing at all: the 32MB volume wins if it is present, so the 10MB image can stay
// on the card untouched as a fallback and a known-good control. Both extensions are
// listed because either is a valid hard-disk image; the scan in selector.h matches
// them case-insensitively, but a literal open() does not, so the common spellings
// are enumerated here.
//
// The companion .tag and .bak names are NOT tabulated. They are derived from the
// path of whatever image actually opened, by selSibling(); a fixed table indexed by
// position was the bug that made every image look for profile32.tag.
static const char *imageNames[] = {
  "profile32.img", "profile32.hdv", "profile32.IMG", "profile32.HDV",
  "profile10.img", "profile10.hdv", "profile10.IMG", "profile10.HDV",
};
static const char *openedImage  = "(none)";
#define BLOCK_DATA 512
#define BLOCK_TAG  20
#define BLOCK_TOTAL (BLOCK_DATA + BLOCK_TAG)   // 532 payload bytes per block, ON DISK

// Payload bytes actually clocked over the BUS, which is not the same thing.
// Measured 2026-08-21: the //e card takes 4 status + 512 data bytes and then stops
// strobing. Sending the 20 tag bytes it never asked for cost ~1.06 s per block:
// the emulator sat in sendReadByte until BYTE_TIMEOUT_US (50 ms), was deaf through
// the host's next /PCMD, mis-read the following command frame as op=$FF, and then
// burned a full 1 s handshake timeout resyncing. 71 blocks took 119 s, of which
// 478 ms was real data transfer. The reader's h/r commands showed the same
// 20-bytes-short signature all along (see README, loopback test).
//
// So the wire length is a variable, not BLOCK_TOTAL: 512 for the //e (default),
// 532 for a host that really wants tags (Apple ///, or the reader's 't' test).
// 'g' toggles it. Disk layout is untouched either way; tags are always stored.
volatile uint16_t wireBlockBytes = BLOCK_DATA;

// A WRITE is longer than its data. Traced 2026-08-22 in the card ROM: after the
// data transfer, $CD49 falls into $CD82, which writes SIX more bytes to $C080,X:
//     $00, $47, $46, $FF, $FF EOR $47, $46 EOR $FF
// i.e. a block-number echo with complements, and only THEN calls $CD5D, the
// post-frame parity check. Consuming only 512 leaves those six bytes strobed at a
// drive that is not listening and not generating parity for them, so $CD5D latches
// a parity error ($0778 bit 0) and the ROM returns $27 even though the block was
// written correctly. That is the whole write bug.
#define BLOCK_WRITE_TRAILER 6
volatile uint16_t wrWireBytes = BLOCK_DATA + BLOCK_WRITE_TRAILER;

SdFat sd;
FsFile img;
FsFile tag;
bool   haveTag = false;
bool   haveImage = false;

// TWO block counts, deliberately separate.
//
//   volumeBlocks  what the drive CLAIMS in the synthesized spare table. Must be a
//                 multiple of 4 or the //e's ROM boot path never hands off
//                 (finding 22), and must fit in 16 bits, so it tops out at $FFFC.
//   imageBlocks   what the image file actually HOLDS, capped at $FFFF because the
//                 host cannot express a bigger block number anyway.
//
// For any image of 65532 blocks or fewer these are equal and none of this matters.
// They diverge for a full 32MB ProDOS volume: 65535 blocks, which is three too many
// AND misaligned, so it can never be claimed. See serveFullImage below.
uint32_t volumeBlocks = 0x004C00;  // 10MB ProFile default; recomputed from file size
uint32_t imageBlocks  = 0;

// DECOUPLED GEOMETRY: AUTOMATIC for an image of 65533..65535 blocks.
//
// Serve every block the image really has, while continuing to CLAIM the aligned,
// boot-safe count. The case that needs it is a full 32MB ProDOS volume (65535
// blocks, e.g. Total Replay): three blocks too many AND misaligned, so it can never
// be claimed, and if the volume is full it cannot be shrunk to fit either.
//
// Why this works at all: the multiple-of-4 rule is ONE instruction in the ROM's
// boot path ($CBFD: lda $46 / and #$03 / bne, guarding the jsr $CC75 handoff),
// where $46 holds the low byte of the count we reported. The data path does not
// bounds-check the block number against that count: measured in finding 22, with
// the drive claiming $4C00 = 19456 blocks, a READ of block 40000 succeeded. So the
// claim only has to satisfy the boot, and ProDOS takes the volume's real size from
// total_blocks in block 2 of the image, not from us.
//
// WHY THE BAND IS NARROW. Rounding down means ANY misaligned image has 1-3 blocks
// above its claim, not just the big ones. Only 65533..65535 gets this treatment,
// because that is the only range where the aligned claim is unreachable: anywhere
// below it, `wozit resize` to a multiple of 4 is available, costs nothing, and
// keeps the drive on the fully tested path. Up here there is no such option.
//
// STILL UNVERIFIED ON HARDWARE, in this order of risk:
//   1. Only READS above the claim have been measured. Writes have not.
//   2. Whether ProDOS 8 cross-checks the driver's device size against the volume's
//      total_blocks at mount time.
//   3. Utilities that ask the driver how big the device is will see the claim.
// Test 1 first: write block 65534, read it back, compare. 'V' forces the old
// behavior (stop at the claim) so a suspect failure can be A/B'd against it.
volatile uint8_t serveFullImage = 1;

// True when we are serving past the claim. The band test is what makes this
// automatic: 65533..65535 are exactly the counts that cannot be claimed.
static inline bool geometryDecoupled() {
  return serveFullImage && imageBlocks > 0xFE00 && imageBlocks > volumeBlocks;
}

// The last block we will read or write. Everything else stays on volumeBlocks.
static inline uint32_t servedBlocks() {
  return geometryDecoupled() ? imageBlocks : volumeBlocks;
}

// One place that turns a file size into the two counts, so the three callers that
// (re)open an image cannot drift apart.
static void applyGeometry(uint64_t bytes) {
  if (bytes >= 512) {
    uint64_t blocks = bytes / 512;
    if (blocks > 0xFFFF) blocks = 0xFFFF;   // the host's block number is 16-bit
    imageBlocks  = (uint32_t)blocks;
    volumeBlocks = (uint32_t)blocks;
  } else {
    imageBlocks = volumeBlocks;
  }
  if (volumeBlocks > 0xFE00) volumeBlocks = 0xFE00;  // $FE00: highest claim CONFIRMED to boot (2026-08-30); $FFFC BRKs, see PROTOCOL.md 9.2
  volumeBlocks &= ~3u;                      // finding 22
}
static uint32_t writesSinceSync = 0;
static uint32_t lastWriteMs = 0;   // when the most recent block write landed
static uint32_t lastTxnMs = 0;     // when the bus last went idle (ANY transaction)
uint8_t  blockBuf[BLOCK_TOTAL];    // 532: 512 data + 20 tag
uint8_t  ramBuffer[BLOCK_TOTAL];   // the drive's block $FFFFFE scratch buffer
uint8_t  status[4] = {0, 0, 0, 0}; // 4 status bytes prepended to every read/write
bool     wasReset = true;          // report "drive was reset" on the first command
int8_t   idlePRW  = -1;            // last PR/-W state applied to the data pins at idle
volatile uint8_t idleDriveByte = 0xA5;   // value driven on the bus at idle. $A5 (1010_0101)
// Static line-state controls for bench tests (serial commands, see loop()):
// reproduce the mid-transaction posture (pins input, chosen BSY level) while the
// //e monitor pokes the card, to test whether a drive-line state blocks the card's
// $C50x soft switches. 'z' tri-states the data pins like the confirm-wait does;
// keep that window brief: the data '245's A-side floats while tri-stated.
// Second write confirmation, the value answered after the write data phase.
// PROTOCOL.md documents $06 ("perform the write") and the ROM disassembly agrees
// ($CCC5: lda #$06 / jsr $CDAC). The //e NAKs it anyway, and that is UNRESOLVED.
//
// CORRECTION (2026-08-22): an earlier sweep concluded the //e "expects $01, not
// $06", because $01 was the only value that did not draw an $AA NAK. That was
// WRONG. $CDAC's mismatch/error path re-handshakes expecting $01, so answering
// $01 matches the ERROR path, not the write path: the host then drives PR/-W low
// and strobes a fresh command frame at us while we are still driving status
// bytes. $06 is believed correct; something before this point is putting the ROM
// into its error path. 'y' commits idleDriveByte here for re-sweeping.
volatile uint8_t wrConfirmByte = 0x06;
// 0 = host writes reach the image. DEFAULT CHANGED 2026-08-29, after this cost
// a real debugging session: with it set, the drive ACCEPTS a write transaction
// and throws the data away, so ProDOS reports success and the file does not
// change. "SAVE STARTUP" appeared to work and "LOAD STARTUP" returned
// the old program, with nothing anywhere saying why.
//
// The hazard the old default guarded against is real and still worth knowing: a
// floating bus while the card is out of the slot can complete thousands of
// phantom write transactions and scribble the image. The protection is now
// opt-in ('w' toggles) instead of the default. A drive that discards writes
// without reporting it is the more expensive failure of the two. It looks like
// a filesystem bug on the //e side, which is where the time goes.
//
// A better fix than either default would be to refuse writes only while no host
// is actually present, rather than always or never.
volatile uint8_t writeProtect = 0;
volatile uint8_t nakIgnore = 0;          // 1 = ignore the card's $AA NAK and push on
                                         // into the data phase (pre-debug behavior)
volatile uint8_t confirmOverride = 0;    // 1 = answer the CONFIRM with idleDriveByte
                                         // instead of the op-derived intent byte, so a
                                         // failing-context run can value-sweep the
                                         // confirm ('k' toggles).
volatile uint8_t respOverride = 0;       // 1 = answer the phase-1 handshake with
                                         // idleDriveByte instead of $01. Bench hook: a
                                         // //e monitor stub can JSR the card ROM's real
                                         // $CDAC handshake against ANY response value,
                                         // reproducing the live confirm's fetch context.
volatile uint8_t idleTristate = 0;       // 1 = do not idle-drive (mimic confirm-wait)
                                         // is distinctive: a working $C0D1 read returns THIS,
                                         // showing the read path works (not stuck at $00).
                                         // Set live over USB serial: type two hex digits.

// Liveness counters, reported by the idle heartbeat only. A plain increment costs no
// meaningful time, so these are safe to bump mid-transaction.
volatile uint32_t nReadOK = 0, nWriteOK = 0, nAbort = 0, nNak = 0, nRunt = 0;
// Writes that were ACCEPTED on the wire and then went nowhere. Either of
// these being non-zero explains a file that will not save.
volatile uint32_t nWrRefused = 0, nWrFailed = 0;

// Serial-free diagnostics: which phase we failed in, plus a change-triggered recorder of
// the confirm + read handshake (the emulator's own view; RAM stores don't perturb timing).
volatile const char *lastEv = "start";
volatile int32_t  lastByte = -1;
volatile uint8_t  lastOp   = 0xEE;
volatile uint32_t lastBlk  = 0;
#define EV(s) do { lastEv = (s); } while (0)
struct RSamp { uint32_t cyc; uint8_t flags; uint8_t data; };  // flags b0=/PSTRB b1=PR/-W b2=/PBSY b3=driving
#define RBUF_N 400
DMAMEM RSamp rbuf[RBUF_N];
uint32_t rn = 0;
uint8_t  rlastF = 0xFF, rlastD = 0xFF;
volatile bool rdump = false;
volatile bool rfree = false;    // 'R': free-running line recorder
volatile bool rfreeze = false;   // latch the buffer at the first failure until dumped
// 'B': one-shot boundary capture. Arming waits for a read to COMPLETE, then records
// everything from that instant (including the idle gap, via loop()) until the buffer
// fills. That frames exactly the window we cannot explain: what the //e does between a
// finished read and the phase-1 handshake that then times out. Arming mid-boot with the
// plain free-run 'R' is useless here, since it fills 400 samples inside one data phase.
volatile uint8_t rtrig = 0;      // 0 = off, 1 = armed, 2 = capturing
volatile uint8_t lastResp = 0, lastAck = 0;
static inline void rrec(bool driving, uint8_t data) {
  if (rfreeze) return;
  // Boundary capture stops itself the moment the buffer is full, so the window stays
  // framed on the transition instead of being overwritten by the next transaction.
  if (rtrig == 2 && rn >= RBUF_N) { rfreeze = true; rdump = true; rtrig = 0; rfree = false; return; }
  uint8_t f = (digitalReadFast(PSTRBIN) ? 1 : 0) | (digitalReadFast(PRWIN) ? 2 : 0) |
              (digitalReadFast(PBSYOUT) ? 4 : 0) | (driving ? 8 : 0) |
              (digitalReadFast(PCMDIN) ? 16 : 0) |
              (digitalReadFast(PPAROUT) ? 32 : 0);   // what WE are presenting on /PPARITY
  if ((f != rlastF || data != rlastD) && rn < RBUF_N) {
    rbuf[rn].cyc = ARM_DWT_CYCCNT; rbuf[rn].flags = f; rbuf[rn].data = data; rn++;
    rlastF = f; rlastD = data;
  }
}

// Latch the buffer at a failure so it survives until dumped. NOT while a boundary
// capture is armed or running: this boot aborts routinely (a garbage op=$FF command
// frame every block), and an early latch would freeze the recorder before the
// trigger ever fired, or truncate the capture right in the middle of the window we
// are trying to see. The boundary capture ends itself when the buffer is full.
static inline void failFreeze() { if (!rtrig) rfreeze = true; }

// The //e interface card idles /PRES LOW and holds it low even while issuing commands,
// so treating /PRES-low as a reset makes the drive ignore every command. /PRES handling
// is therefore disabled here. This is a known necessity for the //e, not leftover debug.
#define HONOR_PRES 0

// QUIET 1 = no unsolicited serial during operation (heartbeat and recorder dumps
// suppressed), to test the "instrumentation breaks the boot" hypothesis directly
// (2026-08-20). Type '?' or return on the serial monitor for ONE status line plus
// any pending recorder dump. Counters still accumulate normally.
#define QUIET 1

// Per-byte guard so a stuck bus can't hang a wait loop forever.
#define BYTE_TIMEOUT_US 50000UL    // 50 ms; real strobes are sub-microsecond

// Guard for one step of a handshake (wait for PR/-W to turn, for the host to take
// the byte, for /PCMD to release). This used to be a flat 1 second, which meant a
// single desync cost a full second of idle bus: 71 of them accounted for 71.0 s of
// the 119 s boot measured on 2026-08-21. Observed normal steps are 52 us (phase 1)
// and 190 us (confirm), so 20 ms is still a 100x margin, and a real drive recovers
// far faster than a second.
#define HS_TIMEOUT_US 20000UL

// Deadlines from the DWT cycle counter, NOT micros(). micros() on the Teensy 4 is a
// critical section (interrupts off, systick + cycle counter), roughly 200 ns a call.
// Calling it once per poll iteration set the sampling interval of the data-phase
// loops to ~250 ns while the //e's strobe low period measures 355-660 ns, so those
// loops were barely sampling faster than the edge they had to catch. That is a race,
// and it showed up as ~1.6% of blocks NAKing at the write confirm. A CYCCNT read is
// a single load. The counter wraps every 7.16 s at 600 MHz; a signed compare handles
// the wrap, and every deadline here is milliseconds.
#define CYC_PER_US (F_CPU_ACTUAL / 1000000)
static inline uint32_t cycDeadline(uint32_t us) { return ARM_DWT_CYCCNT + us * CYC_PER_US; }
static inline bool cycExpired(uint32_t dl) { return (int32_t)(ARM_DWT_CYCCNT - dl) > 0; }

// Bytes where the strobe-low window was never observed (the loop body never ran), so
// parity for that byte may have been presented late. Should stay 0.
volatile uint32_t nShortStrobe = 0;

// ---- Timing instrumentation ---------------------------------------------
// Segment accumulators, stamp macros and the reading guide live in timing.h.
Acc aP1, aCmd6, aCf, aLoad, aRd, aWrRecv, aWrCf, aStore, aWrStat;
Acc aTxn, aTxnBad, aGap;
uint32_t tTxn0 = 0, tSeg0 = 0;
Acc     *curSeg = nullptr;
uint32_t txnPrevEndUs = 0;
uint32_t timingStartMs = 0;

volatile uint8_t parForce = 0;   // fwd: 0=computed 1=force LOW 2=force HIGH
// ---- Fast bus I/O --------------------------------------------------------
// Constant pin numbers let digitalReadFast/digitalWriteFast collapse to single register
// ops, so a whole byte moves in well under the 500 ns strobe budget.
static inline void busWriteFast(uint8_t v) {
  digitalWriteFast(2, (v >> 7) & 1);
  digitalWriteFast(3, (v >> 6) & 1);
  digitalWriteFast(4, (v >> 5) & 1);
  digitalWriteFast(5, (v >> 4) & 1);
  digitalWriteFast(6, (v >> 3) & 1);
  digitalWriteFast(7, (v >> 2) & 1);
  digitalWriteFast(8, (v >> 1) & 1);
  digitalWriteFast(9, (v >> 0) & 1);
}

static inline uint8_t busReadFast() {
  uint8_t v = 0;
  if (digitalReadFast(2)) v |= 0x80;
  if (digitalReadFast(3)) v |= 0x40;
  if (digitalReadFast(4)) v |= 0x20;
  if (digitalReadFast(5)) v |= 0x10;
  if (digitalReadFast(6)) v |= 0x08;
  if (digitalReadFast(7)) v |= 0x04;
  if (digitalReadFast(8)) v |= 0x02;
  if (digitalReadFast(9)) v |= 0x01;
  return v;
}

// Odd parity on the active-low /PPARITY line (odd popcount -> LOW). The inverse polarity
// was tried on hardware and made the read stall earlier, so this is the correct one.
static inline void driveParity(uint8_t v) {
  if (parForce) { digitalWriteFast(PPAROUT, parForce == 1 ? LOW : HIGH); return; }
  v ^= v >> 4;
  v ^= v >> 2;
  v ^= v >> 1;          // v&1 == XOR of all 8 bits == 1 for an ODD number of 1s
  digitalWriteFast(PPAROUT, (v & 1) ? LOW : HIGH);
}

static inline void presentByte(uint8_t v) { busWriteFast(v); driveParity(v); }


// Data bus = pins 2..9. Flip direction with ONE GDIR write per port (not 8x pinMode,
// ~16 us), so the first read byte is driven before the //e's back-to-back $C081 reads.
// The pads are configured for output ONCE in setup(); thereafter GDIR alone toggles
// drive vs high-Z, and digitalReadFast still reads them (PSR is live in GPIO mode).
// Pins 2-5 = GPIO9 bits 4,5,6,8; pins 6-9 = GPIO7 bits 10,11,16,17.
#define DATABUS_GDIR9  0x00000170u   // pins 2,3,4,5
#define DATABUS_GDIR7  0x00030C00u   // pins 6,7,8,9
static inline void dataPinsOutput() { GPIO9_GDIR |= DATABUS_GDIR9;  GPIO7_GDIR |= DATABUS_GDIR7;  }
static inline void dataPinsInput()  { GPIO9_GDIR &= ~DATABUS_GDIR9; GPIO7_GDIR &= ~DATABUS_GDIR7; }

// Parity straight off the port registers, for HOST-driven bytes.
// On a write the drive must present /PPARITY for a byte the host is driving, and
// the card samples it at the strobe's rising edge. Assembling the byte with eight
// digitalReadFast() calls and then folding it costs far more than the sub-microsecond
// strobe allows, and the resulting late parity is what makes the card's post-frame
// check fail (measured 2026-08-21: 12.5% of blocks NAKed at the write confirm).
// Parity is the XOR of all eight bits and does not care about bit ORDER, so it can
// be taken from the two masked port registers without ever building the byte.
static inline void driveParityFast() {
  if (parForce) { digitalWriteFast(PPAROUT, parForce == 1 ? LOW : HIGH); return; }
  uint32_t a = GPIO9_PSR & DATABUS_GDIR9;   // pins 2,3,4,5
  uint32_t b = GPIO7_PSR & DATABUS_GDIR7;   // pins 6,7,8,9
  digitalWriteFast(PPAROUT, (__builtin_parity(a) ^ __builtin_parity(b)) ? LOW : HIGH);
}

static inline void assertBSY()   { digitalWriteFast(PBSYOUT, LOW);  }  // busy
static inline void deassertBSY() { digitalWriteFast(PBSYOUT, HIGH); }  // ready

static inline bool abortReq() {
#if HONOR_PRES
  if (digitalReadFast(PRESIN) != LOW) return false;
  delayMicroseconds(3);
  return digitalReadFast(PRESIN) == LOW;
#else
  return false;
#endif
}

// ---- Control-line waits (slow path; timeout + reset-abort) ---------------
static bool waitPin(int pin, int level, uint32_t timeout_us = HS_TIMEOUT_US) {
  uint32_t t = micros();
  while (digitalReadFast(pin) != level) {
    if (abortReq()) return false;
    if (micros() - t > timeout_us) return false;
  }
  return true;
}
#define waitPCMDlow()   waitPin(PCMDIN, LOW)
#define waitPRWlow()    waitPin(PRWIN,  LOW)
#define waitPRWhigh()   waitPin(PRWIN,  HIGH)

// ---- Data phase (tight loops, /PSTRB-paced) ------------------------------
// One host strobe pulse: falling (data latched) then rising (advance).
static inline bool strobePulse() {
  // Cycle-counter deadlines, not micros(). This routine was missed when the data
  // path was converted: micros() is a ~200 ns critical section, so polling here
  // ran at ~250 ns intervals against a 355-660 ns strobe, i.e. fast enough to
  // MISS a pulse. The only caller is the 4-byte status phase at the end of a
  // write, and a missed strobe there shifts the status frame, which is exactly
  // what the ROM inspects before deciding a write failed.
  uint32_t dl = cycDeadline(BYTE_TIMEOUT_US);
  while (digitalReadFast(PSTRBIN)) {
    if (rfree || rtrig) rrec(true, busReadFast());
    if (cycExpired(dl)) return false;
  }
  while (!digitalReadFast(PSTRBIN)) {
    if (rfree || rtrig) rrec(true, busReadFast());
    if (cycExpired(dl)) return false;
  }
  return true;
}

// Clock ONE read byte to the host, FOLLOWING PR/-W. The data '245's DIR is wired to
// PR/-W in hardware, and the ROM writes the control register ($CD5D: STA $C083) once
// after the 4-byte header, which briefly reverses the buffer. Driving OUTPUT into a
// reversed buffer corrupts the read. Rules learned on hardware:
//   * Check /PSTRB FIRST: a read pulses /PSTRB while PR/-W is high, but PR/-W can read
//     LOW for a sub-us skew right at the pulse; a /PSTRB-low always means "byte latched".
//   * Release the bus only on a SUSTAINED PR/-W low (a control write holds it low for
//     microseconds), never on the transient read-strobe skew. PRW_DIP_US debounces them.
#define PRW_DIP_US 2
static bool sendReadByte(uint8_t b) {
  uint32_t t = micros();
  bool driving, rwLow; uint32_t rwLowT = 0;
  if (digitalReadFast(PRWIN)) { presentByte(b); dataPinsOutput(); driving = true;  rwLow = false; }
  else                        { dataPinsInput();                  driving = false; rwLow = true; rwLowT = t; }
  for (;;) {
    rrec(driving, busReadFast());
    if (!digitalReadFast(PCMDIN)) return false;           // host abandoned the read (new handshake)
    if (!digitalReadFast(PSTRBIN)) break;                 // /PSTRB low == byte latched: catch FIRST
    if (digitalReadFast(PRWIN)) {                         // PR/-W high: (re)drive b
      rwLow = false;
      if (!driving) { presentByte(b); dataPinsOutput(); driving = true; }
    } else {                                              // PR/-W low + /PSTRB high == control write
      uint32_t now = micros();
      if (!rwLow) { rwLow = true; rwLowT = now; }
      else if (driving && (uint32_t)(now - rwLowT) >= PRW_DIP_US) { dataPinsInput(); driving = false; }
    }
    if (abortReq() || micros() - t > BYTE_TIMEOUT_US) return false;
  }
  while (!digitalReadFast(PSTRBIN)) {                      // wait for /PSTRB to RISE (advance)
    if (!digitalReadFast(PCMDIN)) return false;            // host abandoned mid-strobe
    if (abortReq() || micros() - t > BYTE_TIMEOUT_US) return false;
  }
  return true;
}

// Drive -> host: present each byte, then let the host strobe it out.
// Assumes PR/-W is HIGH and the data pins are OUTPUT.
static bool sendBytes(const uint8_t *b, int n) {
  for (int i = 0; i < n; i++) { presentByte(b[i]); if (!strobePulse()) return false; }
  return true;
}

// Host -> drive: sample each byte while /PSTRB is low, then wait for the rise.
// Assumes PR/-W is LOW and the data pins are INPUT.
static bool recvBytes(uint8_t *b, int n) {
  // Drain an in-progress strobe first (e.g. the host's just-written handshake ack):
  // counting it as byte 0 desyncs the whole frame (the op=$55 artifact). Byte 0 must
  // begin on a FRESH falling edge.
  { uint32_t dl = cycDeadline(BYTE_TIMEOUT_US);
    while (!digitalReadFast(PSTRBIN)) {
      if (!digitalReadFast(PRWIN)) driveParityFast();   // host-driven byte on the bus
      if (rfree || rtrig) rrec(false, busReadFast());
      if (cycExpired(dl)) return false;
    } }
  for (int i = 0; i < n; i++) {
    uint32_t dl = cycDeadline(BYTE_TIMEOUT_US);
    for (;;) {
      // Drive parity while WAITING for the strobe to fall, not only while it is low.
      // The host sets its byte up before the strobe falls, so a loop that only
      // updates parity during the low window leaves the PREVIOUS byte's parity on
      // the line through the whole setup time. Every byte whose parity differs from
      // its predecessor is then briefly wrong, the card latches a parity error, and
      // the ROM takes the $CC21 error path (which re-handshakes expecting $01, and
      // is why $01 looked like the "correct" write confirm).
      // driveParityFast() is 2 loads, 2 masks, a parity fold and a store, so this
      // does not meaningfully slow the fall detection.
      while (digitalReadFast(PSTRBIN)) {
        driveParityFast();     // NOTHING else here: this loop has to track the host's
        if (cycExpired(dl)) return false;   // setup window, and the recorder's ~13 GPIO
      }                        // reads in it are enough to make parity late.
      if (!digitalReadFast(PRWIN)) break;   // real host-write strobe: RW already low
      // Phantom: this card can fire a strobe BEFORE its direction flip lands (recorded
      // 2026-08-20: /PSTRB fell with PR/-W still high, RW dropped mid-strobe). Swallow it.
      while (!digitalReadFast(PSTRBIN)) { if (cycExpired(dl)) return false; }
    }
    // Latch at the strobe's RISING edge: the '374 can start driving the byte mid-strobe,
    // so the falling-edge value can be floating bus.
    // The drive owns /PPARITY in BOTH directions: on a host write it must report the
    // parity of the byte it is receiving. Without this the line stays wherever the last
    // presentByte() left it, the card's post-frame parity check ($CD5D, status bit 6)
    // fails, $CE4A returns carry set, and $CBA4 branches to the $CC21 error path, which
    // re-handshakes expecting $01. That is the whole "card NAKs the confirm" symptom:
    // the confirm was never reached. Drive it live so it is valid at the strobe's rise.
    driveParityFast();
    uint8_t v = busReadFast();
    uint32_t spins = 0;
    while (!digitalReadFast(PSTRBIN)) {
      driveParityFast();                     // parity FIRST: it is the timing-critical output
      v = busReadFast();
      spins++;
      if (cycExpired(dl)) return false;
    }
    if (!spins) nShortStrobe++;              // low window never observed: parity was late
    b[i] = v;
  }
  return true;
}

// ---- Media access --------------------------------------------------------
// Fill a 532-byte payload with the drive's spare/status table (block $FFFFFF).
static void buildSpareTable(uint8_t *buf) {
  memset(buf, 0, BLOCK_TOTAL);
  memcpy(buf, "ProFile 10M  ", 13);       // 0x00: 13-byte device name
  buf[0x0D] = 0x00; buf[0x0E] = 0x00; buf[0x0F] = 0x10; // device number: 10MB
  buf[0x10] = 0x03; buf[0x11] = 0x98;                   // firmware rev "3.98"
  buf[0x12] = (volumeBlocks >> 16) & 0xFF;              // 0x12: total blocks
  buf[0x13] = (volumeBlocks >>  8) & 0xFF;
  buf[0x14] = (volumeBlocks      ) & 0xFF;
  buf[0x15] = (BLOCK_TOTAL >> 8) & 0xFF;                // 0x15: bytes/block = 532
  buf[0x16] = (BLOCK_TOTAL     ) & 0xFF;
  buf[0x17] = 0x20;                        // 0x17: 32 spares available
  buf[0x18] = 0x00;                        // 0x18: spares used
  buf[0x19] = 0x00;                        // 0x19: bad blocks
  buf[0x1A] = 0xFF; buf[0x1B] = 0xFF; buf[0x1C] = 0xFF; // empty spared list
  buf[0x1D] = 0xFF; buf[0x1E] = 0xFF; buf[0x1F] = 0xFF; // empty bad-block list
}

// Load a logical block into buf (532 bytes). Returns false on an invalid block number.
static bool loadBlock(uint32_t blk, uint8_t *buf) {
  memset(buf, 0, BLOCK_TOTAL);
  // Remember the previous block FIRST: $FFFFFF returns early below, and it is
  // precisely the get-status read that marks a boot.
  uint32_t prevBlk = selPrevBlk;
  selPrevBlk = blk;
  if (blk == 0xFFFFFF) { buildSpareTable(buf); return true; }
  if (blk == 0xFFFFFE) { memcpy(buf, ramBuffer, BLOCK_TOTAL); return true; }
  // Selector magic blocks. MEASURED 2026-08-22: the card ROM maps a 16-bit ProDOS
  // block of $FFxx onto ProFile block $FFFFxx, not $00FFxx: it treats $47 = $FF as
  // placing the request in the drive's high special range (see $CB2D, which tests
  // $47 against $FF). So the window is $FFFF00-$FFFFFD, just below the two real
  // special blocks, and it can never collide with data (a volume tops out at
  // $FFFC blocks).
  if (blk >= 0xFFFF00 && blk <= 0xFFFFFD) {
    if (selMagicBlock((uint8_t)(blk & 0xFF), buf, BLOCK_TOTAL)) return true;
  }
  // Block 0 before a selection is the synthetic loader, not the image's own
  // boot block. With 0 or 1 images there is nothing to choose, so we do not
  // intercept and the common case is identical to having no selector at all.
  if (blk == 0 && prevBlk == 0xFFFFFF && selWantSyntheticBoot()) {
    memcpy(buf, SEL_BOOT, sizeof(SEL_BOOT) < BLOCK_DATA ? sizeof(SEL_BOOT) : BLOCK_DATA);
    return true;
  }
  if (blk >= servedBlocks()) {
    // Lenient reads for trimmed images (2026-08-30, NOX.IMG: a 2880-block
    // image whose loader reads absolute block 8330). Software emulators all
    // serve zeros past EOF and images in the wild depend on it; an error
    // status here put NOX into an endless retry loop on real hardware.
    // Writes below stay refused.
    memset(buf, 0, BLOCK_TOTAL);
    return true;
  }
  if (!haveImage) {
    // No image: serve a self-verifying test pattern so a loopback reader can confirm
    // both the protocol AND block addressing.
    buf[0] = 0xA5;
    buf[1] = (blk >> 16) & 0xFF;
    buf[2] = (blk >>  8) & 0xFF;
    buf[3] = (blk      ) & 0xFF;
    for (int i = 4; i < BLOCK_TOTAL; i++) buf[i] = (uint8_t)(blk + i);
    return true;
  }
  img.seek((uint64_t)blk * BLOCK_DATA);
  img.read(buf, BLOCK_DATA);
  if (haveTag) { tag.seek((uint64_t)blk * BLOCK_TAG); tag.read(buf + BLOCK_DATA, BLOCK_TAG); }
  return true;
}

static bool storeBlock(uint32_t blk, const uint8_t *buf) {
  if (blk == 0xFFFFFE) { memcpy(ramBuffer, buf, BLOCK_TOTAL); return true; }
  if (blk == 0xFFFFFF) return false;      // spare table is read-only over the bus
  if (blk >= servedBlocks()) return false;
  img.seek((uint64_t)blk * BLOCK_DATA);
  img.write(buf, BLOCK_DATA);
  // Sync periodically, not per block. The image is a FIXED-SIZE file overwritten in
  // place, so no FAT metadata changes and a per-block sync() bought nothing while
  // costing 35 ms/block (worst 202 ms) with BSY asserted the whole time.
  lastWriteMs = millis();
  if (++writesSinceSync >= 512) { writesSinceSync = 0; img.sync(); }
  // Only persist tags if the host actually sent them. With wireBlockBytes == 512 the
  // tag region of blockBuf still holds whatever the last READ left there, and writing
  // that would corrupt the tag file with stale data.
  if (haveTag && wireBlockBytes >= BLOCK_TOTAL) {
    tag.seek((uint64_t)blk * BLOCK_TAG); tag.write(buf + BLOCK_DATA, BLOCK_TAG); tag.sync();
  }
  return true;
}

// ---- Selector glue --------------------------------------------------------
// Open selCatalog[idx] and recompute the geometry from it. Called at startup and
// whenever the //e issues a SELECT.
static bool selApplySelection(int idx) {
  if (idx < 0 || idx >= selCount) return false;
  img.close();
  // O_RDWR, not O_RDONLY: the image must stay writable or a host save cannot
  // land. But say so when it fails, because the caller turns a false return
  // into one byte the 6502 blob reads and nothing else ever mentions it.
  if (!img.open(selCatalog[idx].path, O_RDWR)) {          // put the old one back
    Serial.print("SELECT FAILED: cannot open "); Serial.println(selCatalog[idx].path);
    if (selCurrent < selCount) img.open(selCatalog[selCurrent].path, O_RDWR);
    return false;
  }
  selCurrent = idx;
  haveImage  = true;
  openedImage = selCatalog[idx].path;
  applyGeometry(img.fileSize());
  selectorArmed = false;      // block 0 is now the image's own boot block
  selSaveConfig();
  wasReset = true;            // the "drive" has effectively changed underneath
  return true;
}

// ---- Image backup / restore ----------------------------------------------
// A surface test scribbles over the whole volume, so the image can be copied on
// the card itself rather than pulling the SD. Both directions close the image,
// copy, and reopen it. IDLE ONLY: the copy takes seconds and the bus is not
// serviced during it, so never do this with the host mid-transaction.
DMAMEM static uint8_t copyBuf[16384];

static bool copyFile(const char *src, const char *dst) {
  FsFile in, out;
  if (!in.open(src, O_RDONLY)) { Serial.print("cannot open "); Serial.println(src); return false; }
  if (!out.open(dst, O_WRONLY | O_CREAT | O_TRUNC)) {
    Serial.print("cannot create "); Serial.println(dst); in.close(); return false;
  }
  uint64_t total = in.fileSize(), done = 0;
  uint32_t t0 = millis();
  bool ok = true;
  for (;;) {
    int n = in.read(copyBuf, sizeof(copyBuf));
    if (n < 0) { ok = false; break; }
    if (n == 0) break;
    if (out.write(copyBuf, n) != n) { ok = false; break; }
    done += n;
    if ((done % (4u * 1024u * 1024u)) == 0) {          // a dot per 4 MB
      Serial.print("."); 
    }
  }
  out.sync(); out.close(); in.close();
  Serial.println();
  if (!ok || done != total) {
    Serial.print("COPY FAILED after "); Serial.print((uint32_t)done); Serial.println(" bytes");
    return false;
  }
  Serial.print("copied "); Serial.print((uint32_t)(done / 1024));
  Serial.print(" KB in "); Serial.print(millis() - t0); Serial.println(" ms");
  return true;
}

// dir 0 = image -> backup (save), 1 = backup -> image (restore)
static void backupRestore(uint8_t dir) {
  if (!haveImage) { Serial.println("no image open"); return; }
  // Derive the backup name from the image that is OPEN, not from a fixed table:
  // with the selector, any image can be mounted and a fixed name would back up
  // or, far worse, RESTORE OVER the wrong volume.
  char img_n[96], bak_n[96];
  strncpy(img_n, openedImage, sizeof(img_n) - 1); img_n[sizeof(img_n) - 1] = 0;
  selSibling(openedImage, ".bak", bak_n, sizeof(bak_n));
  Serial.print(dir ? "RESTORE " : "SAVE ");
  Serial.print(dir ? bak_n : img_n); Serial.print(" -> ");
  Serial.println(dir ? img_n : bak_n);
  img.sync();
  img.close();
  bool ok = dir ? copyFile(bak_n, img_n) : copyFile(img_n, bak_n);
  if (!img.open(img_n, O_RDWR)) {          // ALWAYS reopen, even on failure
    haveImage = false;
    Serial.println("!! could not reopen the image; reset the Teensy");
    return;
  }
  applyGeometry(img.fileSize());
  Serial.println(ok ? "OK" : "FAILED (image reopened unchanged)");
}

// ---- Handshake -----------------------------------------------------------
// Assert BSY with one byte on the bus, let the host read it and write its ack, then wait
// for the host to finish the step (/PCMD high). We do NOT gate on the ack value: the //e's
// DMA ack isn't a plain $55 where a simple sampler catches it, and BSY keeps the command /
// confirm / data phases in step regardless.
// respondAndAck has three distinct waits and the caller can only report that the whole
// thing failed ("cmd:no-resp01"), which is not enough to find a desync. Count each wait's
// timeouts separately; the distribution over a boot says which one is actually hanging.
volatile uint32_t nHsPrwHigh = 0;   // host never went to read mode
volatile uint32_t nHsPrwLow  = 0;   // host never took the byte we presented
volatile uint32_t nHsPcmdHigh = 0;  // host never released /PCMD
volatile uint32_t nHsAbandon  = 0;  // host released /PCMD without taking the byte

static bool respondAndAck(uint8_t responseByte) {
  if (!waitPRWhigh()) { nHsPrwHigh++; return false; }   // host in read mode -> we may drive
  presentByte(responseByte);           // byte + parity valid BEFORE we signal ready
  dataPinsOutput();
  assertBSY();                         // signal "byte ready"
  { uint32_t tt = micros();            // record the byte-present window (host reads it here)
    while (digitalReadFast(PRWIN) != LOW) {
      rrec(true, busReadFast());
      // The host can ABANDON a command it has opened: /PCMD returns high without it
      // ever turning the bus round to read the byte we are presenting. Measured
      // 2026-08-21: a 59 us /PCMD pulse, then we sat here for the full 20 ms guard.
      // That happened once per block and was 1.4 s of a 1.9 s boot. A real drive
      // sees /PCMD release and drops the handshake immediately. sendReadByte has
      // always had this test; the handshake was missing it.
      // Re-read PR/-W after /PCMD to close the race where both move together: on a
      // normal handshake PR/-W goes low FIRST and /PCMD releases after, so an
      // abandonment is only real if PR/-W is still high on the second look.
      if (digitalReadFast(PCMDIN) && digitalReadFast(PRWIN) != LOW) { nHsAbandon++; return false; }
      if (abortReq() || micros() - tt > HS_TIMEOUT_US) { nHsPrwLow++; return false; }
    } }
  dataPinsInput();
  uint8_t ack = 0xFF;
  uint32_t t = micros();
  while (digitalReadFast(PCMDIN) == LOW) {
    if (digitalReadFast(PRWIN) == LOW) {
      ack = busReadFast();
      driveParity(ack);          // the drive owns /PPARITY for HOST bytes too:
                                 // the ack write strobes, and FF1 samples the
                                 // ack byte's parity at that strobe's rise
    }
    rrec(false, busReadFast());
    if (micros() - t > HS_TIMEOUT_US) { nHsPcmdHigh++; return false; }
  }
  lastResp = responseByte; lastAck = ack;
  return true;
}

// ---- One full transaction ------------------------------------------------
static void goIdle();   // forward decl; defined below

// Entered when the host has asserted /PCMD (PCMDIN == LOW).
static void handleTransaction() {
  EV("cmd:phase1");
  TTXN0();
  // Host think-time since the previous transaction closed: idle time we do not control.
  if (txnPrevEndUs) accAdd(aGap, (uint64_t)(micros() - txnPrevEndUs) * (F_CPU_ACTUAL / 1000000));
  if (!rfreeze && !rfree) { rn = 0; rlastF = rlastD = 0xFF; }  // free-run: keep recording across transactions
  // Phase 1: command handshake. Respond $01, then clock in the 6-byte command frame.
  TSEG(aP1);
  bool ok = respondAndAck(respOverride ? idleDriveByte : 0x01);
  TSEGEND();
  if (!ok) { EV("cmd:no-resp01"); rdump = true; goto abort; }
  if (lastAck == 0xAA) { nNak++; EV("cmd:NAK"); TTXNEND(false); goIdle(); return; }  // host refused the response
  deassertBSY();                       // host waits for this before sending the frame

  uint8_t cmd6[6];
  TSEG(aCmd6);
  ok = recvBytes(cmd6, 6);
  TSEGEND();
  if (!ok) { EV("cmd:no-cmd6"); rdump = true; failFreeze(); goto abort; } // host strobes 6 bytes (PR/-W low)

  {
    uint8_t  op  = cmd6[0];
    uint32_t blk = ((uint32_t)cmd6[1] << 16) | ((uint32_t)cmd6[2] << 8) | cmd6[3];
    lastOp = op; lastBlk = blk;
    bool     isRead  = (op == 0x00);
    bool     isWrite = (op == 0x01 || op == 0x02);   // WRITE or WRITE/VERIFY
    uint8_t  intent  = isRead ? 0x02 : (op == 0x02 ? 0x04 : 0x03);
    if (confirmOverride) intent = idleDriveByte;
    if (!isRead && !isWrite) { EV("cmd:badop"); rdump = true; failFreeze(); goto abort; } // unknown opcode

    status[0] = status[1] = status[2] = status[3] = 0;
    if (wasReset) { status[2] |= 0x80; wasReset = false; }   // "drive was reset"

    // Phase 2: confirmation handshake. Host re-asserts CMD (bus released, PR/-W high);
    // we answer the intent byte.
    TSEG(aCf);   // the abort label closes whatever segment is open, so bail-outs still book
    if (!waitPCMDlow())         { EV("cf:no-pcmd"); goto abort; }
    if (!respondAndAck(intent)) { EV("cf:no-respint"); goto abort; }
    TSEGEND();
    // $AA is the //e ROM's deliberate NAK (the $CDAC mismatch write). A real drive
    // aborts to idle on any reply other than $55; doing the same keeps us in step with
    // the ROM's full retry ($CBB2 BNE $CB80 restarts at phase 1), instead of entering
    // the data phase and miscounting the retry's write strobes as read bytes.
    if (lastAck == 0xAA && !nakIgnore) { nNak++; EV("cf:NAK"); rdump = true; failFreeze(); TTXNEND(false); goIdle(); return; }
    if (lastAck == 0xAA) { nNak++; EV("cf:NAK-ignored"); }   // 'n': pre-debug behavior,
                                                             // push on into the data phase
    // BSY stays ASSERTED across the disk work below; the host polls it.

    if (isRead) {
      // -------- READ data phase --------
      // BSY stays HIGH the whole read. The ROM reads the 4 header bytes back-to-back the
      // instant PR/-W rises, then free-runs the payload, so byte 0 must already be DRIVEN
      // when PR/-W goes high. Pre-stage it into the latch while the pins are still input
      // (no '245 contention: the buffer is host->Teensy until PR/-W flips), then flip to
      // output with a single GDIR write.
      TSEG(aLoad);                       // pure SdFat cost: seek + read, nothing else
      if (!loadBlock(blk, blockBuf)) status[2] |= 0x40;   // block# invalid
      TSEGEND();
      presentByte(status[0]);            // pre-stage byte 0 (pins still input)
      deassertBSY();                     // seek-ack: BSY high, stays high through the read
      TSEG(aRd);                         // 536 host-paced bytes: divide by 536 for us/byte
      if (!waitPRWhigh()) { EV("rd:PRW-never-high"); goto abort; }
      dataPinsOutput();                  // byte 0 hits the bus the instant PR/-W is high
      if (!sendReadByte(status[0])) { EV("rd:no-strobe-byte0"); rdump = true; failFreeze(); goto abort; }
      for (int i = 1; i < 4; i++)           if (!sendReadByte(status[i]))   { lastByte = i; EV("rd:stall-in-header"); rdump = true; failFreeze(); goto abort; }  // rest of header
      // wireBlockBytes, NOT BLOCK_TOTAL: sending tags the host never strobes for is
      // what made the boot slow. 512 for the //e, 532 where tags are wanted.
      for (int i = 0; i < (int)wireBlockBytes; i++) if (!sendReadByte(blockBuf[i])) { lastByte = i; EV("rd:stall-in-block"); goto abort; }
      TSEGEND();
      dataPinsInput();
      nReadOK++; EV("rd:FULL-BLOCK-OK");
      // Boundary trigger: a read just completed cleanly. Start recording HERE and let
      // loop() keep sampling through the idle gap (rfree) so the capture spans the gap
      // and the next handshake, whatever it turns out to be.
      // Clearing rfreeze is essential: an abort earlier in the boot may already have
      // latched it, and arming into a frozen recorder captures nothing.
      if (rtrig == 1) { rtrig = 2; rn = 0; rlastF = rlastD = 0xFF; rfree = true; rfreeze = false; }
    } else {
      // -------- WRITE data phase --------
      deassertBSY();
      dataPinsInput();                   // PR/-W already low (host still driving)
      TSEG(aWrRecv);
      // Symmetric with the read: waiting for 20 tag bytes the host never sends would
      // stall the write the same way.
      if (!recvBytes(blockBuf, wrWireBytes)) goto abort;   // 512 data + 6-byte trailer
      TSEGEND();

      // Second confirmation: host re-asserts CMD, we answer $06 ("perform write").
      // Reframe the recorder here: the 512-byte data phase would otherwise fill the
      // buffer long before the confirm, which is the part that NAKs.
      if (!rfreeze) { rn = 0; rlastF = rlastD = 0xFF; }
      TSEG(aWrCf);
      if (!waitPCMDlow())       goto abort;
      if (!respondAndAck(wrConfirmByte)) goto abort;
      TSEGEND();
      if (lastAck == 0xAA) { nNak++; EV("wr:NAK"); rdump = true; failFreeze();
                             TTXNEND(false); goIdle(); return; }

      TSEG(aStore);                      // SdFat write + sync(): the metadata flush lives here
      if (writeProtect) {
        status[2] |= 0x40;
        nWrRefused++;                    // not reporting this is what cost the session
      } else if (!storeBlock(blk, blockBuf)) {
        status[2] |= 0x40;
        nWrFailed++;
      }
      TSEGEND();
      deassertBSY();

      // Reframe the recorder onto the STATUS phase: the 512-byte data phase would
      // otherwise fill the buffer long before it. This is the last thing the ROM
      // inspects before deciding a write failed, and the only part of the write
      // path never looked at.
      if (!rfreeze) { rn = 0; rlastF = rlastD = 0xFF; rtrig = 2; rfree = false; }
      TSEG(aWrStat);
      if (!waitPRWhigh()) goto abort;    // host reads the 4 status bytes back
      dataPinsOutput();
      if (!sendBytes(status, 4)) goto abort;
      TSEGEND();
      rtrig = 0; rdump = true; rfreeze = true;   // latch the status-phase capture
      nWriteOK++;
    }
  }

  TTXNEND(true);
  goIdle();
  return;

abort:
  TSEGEND();          // book whatever segment was in flight when we bailed
  TTXNEND(false);
  nAbort++;
  goIdle();
}

// ---- Geometry sweep ------------------------------------------------------
// 'v' cycles the CLAIMED volume size so the ROM's geometry ceiling can be tested
// without reflashing or rebuilding an image. The claim is what goes into the
// spare table at $12/$13/$14, which is the only thing the ROM reads; it is
// deliberately allowed to exceed the image file, since blocks past EOF simply
// read back as zeros and a boot never goes there (ProDOS sizes the VOLUME from
// the volume directory, not from the device's reported block count).
//
// The three interesting cases, in cycle order:
//   $4C00 (19456)  the known-good 10MB claim, our control
//   $FFFE (65534)  32MB minus one block: the real ProDOS ceiling, no sentinel alias
//   $FFFF (65535)  finding 20's poison value; aliases the $FFFFFF get-status block
// Geometries for testing the ROM's boot-path constraint (see MULTIPLE OF 4, below).
// $FE00 boots; $FEFF and $FFFE do not. Size is not the discriminator, alignment is.
static const uint32_t geoPresets[] = { 0x4C00, 0xFE00, 0xFEFF, 0xFFFE, 0xFFFF };
static uint8_t geoIdx = 0;

static void printGeometry() {
  Serial.print("claimed volume = "); Serial.print(volumeBlocks);
  Serial.print(" blocks ($"); Serial.print(volumeBlocks, HEX);
  Serial.print(") = "); Serial.print((volumeBlocks * 512UL) / (1024UL * 1024UL));
  Serial.print(" MB");
  if (volumeBlocks % 4) Serial.print("   <-- NOT a multiple of 4: the //e CANNOT BOOT from this");
  if (volumeBlocks > imageBlocks) { Serial.print("   [image is "); Serial.print(imageBlocks);
                                    Serial.print(" blocks; the rest reads as zeros]"); }
  Serial.println();
  Serial.print("served volume  = "); Serial.print(servedBlocks());
  Serial.print(" blocks (image holds "); Serial.print(imageBlocks); Serial.print(")");
  if (geometryDecoupled()) {
    Serial.print("   DECOUPLED: claiming "); Serial.print(volumeBlocks);
    Serial.print(", serving "); Serial.print(imageBlocks);
  } else if (!serveFullImage && imageBlocks > 0xFFFC) {
    Serial.print("   [decoupling FORCED OFF by 'V'; ");
    Serial.print(imageBlocks - volumeBlocks); Serial.print(" blocks unreachable]");
  }
  Serial.println();
}

// ---- Idle / reset --------------------------------------------------------
static void goIdle() {
  lastTxnMs = millis();  // every transaction ends here, completed or aborted
  deassertBSY();
  idlePRW = -1;          // loop() re-asserts data-pin direction vs PR/-W below
}

static void handleReset() {
  goIdle();
  wasReset = true;
  static uint32_t lastMsg = 0;
  if (millis() - lastMsg > 500) { lastMsg = millis(); Serial.println("!! reset (PRES low)"); }
}

// ---- Arduino entry points ------------------------------------------------
void setup() {
  // Cycle counter for the recorder's nanosecond timestamps.
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  // Control outputs first, deasserted, so we present a sane idle to the host.
  pinMode(PBSYOUT, OUTPUT); digitalWriteFast(PBSYOUT, HIGH);   // not busy
  pinMode(PPAROUT, OUTPUT); digitalWriteFast(PPAROUT, HIGH);

  // Buffered host inputs (read-only).
  pinMode(PH0IN, INPUT);
  pinMode(PRESIN, INPUT);
  pinMode(PCMDIN, INPUT);
  pinMode(PSTRBIN, INPUT);
  pinMode(PRWIN, INPUT);

  // Configure the data pads (2..9) for output drive ONCE (drive strength + GPIO mux) so
  // later direction changes are a single GDIR write; then idle them as inputs.
  for (int i = 2; i <= 9; i++) pinMode(i, OUTPUT);
  dataPinsInput();

  pinMode(OE, OUTPUT);
  digitalWriteFast(OE, HIGH);          // keep '245s disabled until we're ready

  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && millis() - t < 1500) { }
  Serial.println("ProFile emulator starting (clean/fast build)...");

  if (!sd.begin(SdioConfig(FIFO_SDIO))) Serial.println("SD init FAILED");
  // Scan the whole card for .img/.hdv files and restore the remembered choice. The
  // named-image fallback below only runs if the scan finds nothing at all.
  selScan();
  selLoadConfig();
  if (selCount > 0) {
    // Only open an image if the saved PATH resolved. With no valid selection the
    // menu is forced, and the selector body reads nothing but magic blocks, so
    // there is deliberately nothing mounted until the user chooses.
    if (selValid && img.open(selCatalog[selCurrent].path, O_RDWR)) {
      haveImage = true;
      openedImage = selCatalog[selCurrent].path;
    }
    Serial.print("Images found: "); Serial.println(selCount);
    for (int i = 0; i < selCount; i++) {
      Serial.print((selValid && i == selCurrent) ? " *" : "  ");
      Serial.print(i); Serial.print("  ");
      Serial.print(selCatalog[i].name);
      Serial.print("   "); Serial.print(selCatalog[i].blocks); Serial.print(" blocks   ");
      Serial.println(selCatalog[i].path);
    }
    if (!selValid)
      Serial.println("no valid saved selection: the MENU IS FORCED at boot");
    else if (selCount > 1)
      Serial.println("selector armed (hold open-apple at boot for the menu)");
    else
      Serial.println("one image and a valid choice: serving its real boot block");
  }
  if (!haveImage && selCount == 0)
  for (unsigned i = 0; i < sizeof(imageNames) / sizeof(imageNames[0]); i++) {
    if (img.open(imageNames[i], O_RDWR)) { haveImage = true; openedImage = imageNames[i]; break; }
  }
  if (haveImage) {
    Serial.print("Image: "); Serial.println(openedImage);
    applyGeometry(img.fileSize());
    // Geometry ceiling. Finding 20 capped this at $4C00 (a standard 10MB ProFile)
    // because a 65535-block ($FFFF) total killed the boot. Disassembling the card
    // ROM (341-0299-B) shows why, and that the cap was drawn too tight:
    //
    //   $CD24 reads 21 spare-table bytes, DISCARDS offset $12 (total-blocks bits
    //   23:16) and keeps only $13/$14 in $46/$47, which it hands to ProDOS as a
    //   16-bit block count (ldx $46 / ldy $47). So the ROM is natively 16-bit:
    //   65535 blocks x 512 = 32 MB, exactly the ProDOS block-device limit.
    //
    // THE CLAIMED BLOCK COUNT MUST BE A MULTIPLE OF 4, or the //e cannot boot from
    // the drive. Measured 2026-08-21 across four geometries:
    //
    //     $4C00 = 19456  low byte $00  mod 4 = 0   BOOTS
    //     $FE00 = 65024  low byte $00  mod 4 = 0   BOOTS
    //     $FEFF = 65279  low byte $FF  mod 4 = 3   fails
    //     $FFFE = 65534  low byte $FE  mod 4 = 2   fails
    //
    // Size is NOT the discriminator: 65024 boots and 65279 does not. In the failing
    // cases the boot never reads a single real block (sd-load n=14 all at 0.5 us,
    // i.e. the synthesized spare table only) and the //e then executes unloaded
    // memory, crashing at a different address every attempt.
    //
    // The ROM path: $CD24 stores the reported 16-bit size into $46/$47, which in the
    // boot path is also the block-number variable. $CBFD then tests
    // "lda $46 / and #$03 / bne (skip) / jsr $CC75", so $CC75 runs only when the low
    // two bits are clear, and $CC75 is the jmp ($004C) boot handoff. A count that is
    // not a multiple of 4 skips the handoff.
    //
    // This supersedes finding 20's reading, which blamed the size and capped at
    // $4C00: $4C00 worked because it happens to be a multiple of 4, not because it
    // is small. Round DOWN so an arbitrary image size can never break the boot; the
    // few discarded blocks (at most 3, under 2 KB) cost nothing.
    // (the clamp itself now lives in applyGeometry(), above.)
    Serial.print("Image open: "); Serial.print(volumeBlocks); Serial.println(" blocks");
    if (imageBlocks > volumeBlocks) {
      Serial.print("NOTE: the image holds "); Serial.print(imageBlocks);
      Serial.print(" blocks, ");             Serial.print(imageBlocks - volumeBlocks);
      Serial.println(" more than can be CLAIMED (multiple-of-4 boot rule).");
      if (geometryDecoupled())
        Serial.println("      DECOUPLED GEOMETRY IS ACTIVE: all of them are served anyway.");
      else if (!serveFullImage)
        Serial.println("      Decoupling is forced off by 'V'; those blocks are unreachable.");
      else
        Serial.println("      Those blocks are unreachable. 'wozit resize' will trim the volume to fit.");
    }
  } else {
    if (selCount > 0)
      Serial.println("nothing mounted yet: waiting for the menu to choose an image");
    else
      Serial.println("No .img or .hdv on SD at all; serving a self-verifying TEST PATTERN");
  }
  // Pair the tag file with the image that is actually OPEN, derived from its own
  // path. Tags are real data describing a specific volume's blocks, so handing
  // one image's tags to another would be wrong, with no error, once tags cross
  // the bus (they do not at wire=512, but that is a setting, not a guarantee).
  {
    char tn[96];
    selSibling(openedImage, ".tag", tn, sizeof(tn));
    if (haveImage && tag.open(tn, O_RDWR)) haveTag = true;
    Serial.print(haveTag ? "Tag file: " : "No tag file for this image (tags served as zeros)");
    if (haveTag) Serial.print(tn);
    Serial.println();
  }

  memset(ramBuffer, 0, sizeof(ramBuffer));

  clearTiming();
  digitalWriteFast(OE, LOW);           // engage the bus buffers
  Serial.println("Emulator ready.  ('T' = timing report, 'X' = clear it)");
}

// One status line: counters, last event, live line states, modes.
static void printStatus() {
  Serial.print("idle  rdOK="); Serial.print(nReadOK);
  Serial.print(" wrOK=");      Serial.print(nWriteOK);
  Serial.print(" abort=");     Serial.print(nAbort);
  Serial.print(" nak=");       Serial.print(nNak);
  Serial.print(" runt=");      Serial.print(nRunt);
  Serial.print(" last=");      Serial.print((const char *)lastEv);
  Serial.print(" byte=");      Serial.print(lastByte);
  Serial.print(" op=");        Serial.print(lastOp, HEX);
  Serial.print(" blk=");       Serial.print(lastBlk, HEX);
  Serial.print("  PRES=");     Serial.print(digitalReadFast(PRESIN));
  Serial.print(" CMD=");       Serial.print(digitalReadFast(PCMDIN));
  Serial.print(" RW=");        Serial.print(digitalReadFast(PRWIN));
  Serial.print(" drive=$");    Serial.print(idleDriveByte, HEX);
  Serial.print(" hs=$");       Serial.print(lastResp, HEX);   // last handshake:
  Serial.print(":$");          Serial.print(lastAck, HEX);    // response:ack
  if (respOverride) Serial.print(" RESP-OVERRIDE");
  if (confirmOverride) Serial.print(" CF-OVR");
  if (nakIgnore) Serial.print(" NAK-IGN");
  // Which handshake wait timed out, over the whole window: PR/-W never went high /
  // host never took the byte / host never released /PCMD.
  Serial.print(" hsF=");       Serial.print(nHsPrwHigh);
  Serial.print("/");           Serial.print(nHsPrwLow);
  Serial.print("/");           Serial.print(nHsPcmdHigh);
  Serial.print(" aband=");     Serial.print(nHsAbandon);
  Serial.print(" vol=$");      Serial.print(volumeBlocks, HEX);
  if (geometryDecoupled()) { Serial.print(" srv=$"); Serial.print(imageBlocks, HEX); }
  else if (!serveFullImage)  { Serial.print(" srv=capped"); }
  Serial.print(" short=");     Serial.print(nShortStrobe);
  Serial.print(" wrcf=$");     Serial.print(wrConfirmByte, HEX);
  Serial.print(" wire=");      Serial.print(wireBlockBytes);
  if (writeProtect) Serial.print(" RO");
  if (nWrRefused) { Serial.print(" wrREFUSED="); Serial.print(nWrRefused); }
  if (nWrFailed)  { Serial.print(" wrFAILED=");  Serial.print(nWrFailed); }
  Serial.println(idleTristate ? " (TRI-STATE)" : "");
}

// ---- Timing report -------------------------------------------------------
// One row per segment: count, total ms, mean us, worst us. Printed only at idle.
// Integer formatting throughout: no float printing, no log10 padding games.
// Values are held in cycles; hundredths of a microsecond is plenty of resolution
// (one cycle at 600 MHz is 1.67 ns).
static void trow(const char *name, const Acc &a, uint32_t perByte = 0) {
  const uint64_t cpu = F_CPU_ACTUAL / 1000000;     // cycles per microsecond
  char line[96];
  uint32_t totKus = (uint32_t)(a.sum / cpu / 10);            // total, hundredths of a ms
  uint32_t meanCu = a.n ? (uint32_t)(a.sum * 100 / a.n / cpu) : 0;  // mean, hundredths of us
  uint32_t maxCu  = (uint32_t)(a.max * 100 / cpu);           // worst, hundredths of us
  snprintf(line, sizeof(line), "  %-9s %-8lu %6lu.%02lu %8lu.%02lu %9lu.%02lu",
           name, (unsigned long)a.n,
           (unsigned long)(totKus / 100),  (unsigned long)(totKus % 100),
           (unsigned long)(meanCu / 100),  (unsigned long)(meanCu % 100),
           (unsigned long)(maxCu  / 100),  (unsigned long)(maxCu  % 100));
  Serial.print(line);
  if (perByte && a.n) {                            // per-byte cost of a data phase
    uint32_t pb = (uint32_t)(a.sum * 100 / a.n / cpu / perByte);   // hundredths of us
    snprintf(line, sizeof(line), "   %lu.%02lu us/byte",
             (unsigned long)(pb / 100), (unsigned long)(pb % 100));
    Serial.print(line);
  }
  Serial.println();
}

static void printTiming() {
  const uint64_t cpu = F_CPU_ACTUAL / 1000000;
  uint32_t winMs = millis() - timingStartMs;
  uint32_t busyMs = (uint32_t)((aTxn.sum + aTxnBad.sum) / cpu / 1000);
  char hdr[112];
  snprintf(hdr, sizeof(hdr), "---- timing over %lu ms (in-transaction %lu ms = %lu%%) ----",
           (unsigned long)winMs, (unsigned long)busyMs,
           (unsigned long)(winMs ? busyMs * 100UL / winMs : 0));
  Serial.println(hdr);
  snprintf(hdr, sizeof(hdr), "  %-9s %-8s %9s %11s %12s",
           "segment", "n", "total_ms", "mean_us", "worst_us");
  Serial.println(hdr);
  trow("txn-OK",  aTxn);
  trow("txn-BAD", aTxnBad);      // time burned in aborts and timeouts
  trow("gap",     aGap);         // host think-time between transactions
  trow("p1-hs",   aP1);
  trow("cmd6",    aCmd6, 6);
  trow("confirm", aCf);
  trow("sd-load", aLoad);        // SdFat seek + read
  trow("rd-data", aRd, 4 + wireBlockBytes);   // divisor tracks what we actually send
  trow("wr-recv", aWrRecv, wireBlockBytes);
  trow("wr-cf",   aWrCf);
  trow("sd-store", aStore);      // SdFat write + sync
  trow("wr-stat", aWrStat, 4);
  // A bailed-out segment is still booked (that is where the lost time went), so a
  // segment's n counts ATTEMPTS, not completions. If rd-data n exceeds the rdOK
  // counter on the '?' line, the difference is partial blocks and the us/byte
  // figure is diluted; read it only when the two agree.
  Serial.println("---- end ('X' clears, '?' for the OK/abort counters) ----");
}

static void clearTiming() {
  aP1 = aCmd6 = aCf = aLoad = aRd = aWrRecv = aWrCf = aStore = aWrStat = Acc{};
  aTxn = aTxnBad = aGap = Acc{};
  txnPrevEndUs = 0;
  timingStartMs = millis();
}

// The pending confirm+read recorder dump (cleared once printed).
static void printRecorder() {
  rdump = false; rfreeze = false;
  Serial.print("---- recorder ("); Serial.print(rn);
  Serial.print(" samples) resp="); Serial.print(lastResp, HEX);
  Serial.print(" ack=");           Serial.print(lastAck, HEX);
  Serial.println("  [+dt_ns STB RW BSY drv CMD D] ----");
  uint32_t prev = rn ? rbuf[0].cyc : 0;
  for (uint32_t i = 0; i < rn; i++) {
    uint32_t dt = (uint32_t)(rbuf[i].cyc - prev) * 10u / 6u;   // cycles -> ns @600 MHz
    uint8_t f = rbuf[i].flags;
    Serial.print("  +");   Serial.print(dt);
    Serial.print(" STB="); Serial.print((f & 1) ? 1 : 0);
    Serial.print(" RW=");  Serial.print((f & 2) ? 1 : 0);
    Serial.print(" BSY="); Serial.print((f & 4) ? 1 : 0);
    Serial.print(" drv="); Serial.print((f & 8) ? 1 : 0);
    Serial.print(" CMD="); Serial.print((f & 16) ? 1 : 0);
  Serial.print(" PAR="); Serial.print((f & 32) ? 1 : 0);
    Serial.print(" D=");   Serial.println(rbuf[i].data, HEX);
    prev = rbuf[i].cyc;
  }
  Serial.println("---- end ----");
}

void loop() {
  // Service the bus BEFORE any serial output: after a NAK abort the //e ROM re-enters
  // phase 1 within tens of microseconds, and serial queued here would make us deaf to
  // the retry's /PCMD.
  if (abortReq())                     { handleReset(); return; }
  if (rfree) rrec(false, busReadFast());
  if (digitalReadFast(PCMDIN) == LOW) {
    // Debounce: the '259 can glitch-latch Q0 on E259-window exits (measured
    // 2026-08-21: a runt /PCMD 100 us before the ROM's real assertion). A real
    // command holds /PCMD low for milliseconds while the host polls BSY; a
    // 3 us re-check rejects the runts the way a real drive's poll loop would.
    delayMicroseconds(3);
    if (digitalReadFast(PCMDIN) == LOW) { handleTransaction(); return; }
    nRunt++;
  }

#if !QUIET
  // Liveness heartbeat once a second, plus deferred recorder dumps (idle only;
  // no serial during a transfer). Suppressed entirely in a QUIET build.
  static uint32_t lastBeat = 0;
  if (millis() - lastBeat > 1000) { lastBeat = millis(); printStatus(); }
  if (rdump) printRecorder();
#endif

  // Serial commands. Non-hex letters are line-state controls for static bench
  // tests; two hex digits still set the idle-drive byte ($C0D1 TRACKS it).
  //   z = tri-state data pins (mimic the confirm-wait posture)   i = idle-drive
  //   s = assert /PBSY (low)                                     h = release /PBSY
  //   o = toggle RESP-OVERRIDE: answer phase-1 with idleDriveByte, so a monitor
  //       stub (JSR $CDAC) can run the card's real handshake against any value
  if (Serial.available()) {
    int c = Serial.read(), v = -1;
    if (c == 'z')      { idleTristate = 1; idlePRW = -1; Serial.println("data = TRI-STATE (confirm-wait posture)"); }
    else if (c == 'i') { idleTristate = 0; idlePRW = -1; Serial.println("data = idle-drive"); }
    else if (c == 's') { digitalWriteFast(PBSYOUT, LOW);  Serial.println("/PBSY = LOW (asserted)"); }
    else if (c == 'h') { digitalWriteFast(PBSYOUT, HIGH); Serial.println("/PBSY = HIGH (released)"); }
    else if (c == 'o') { respOverride ^= 1;
                         Serial.println(respOverride ? "phase-1 response = idleDriveByte (RESP-OVERRIDE ON)"
                                                     : "phase-1 response = $01 (override off)"); }
    else if (c == 'w') { writeProtect ^= 1;
                         Serial.println(writeProtect ? "image WRITE-PROTECTED" : "image WRITABLE"); }
    else if (c == 'R') { rfreeze = false; rn = 0; rlastF = rlastD = 0xFF;
                         rfree = true; rdump = true;
                         Serial.println("free-run recorder ARMED (dump with ?)"); }
    else if (c == 'p') { parForce = (parForce + 1) % 4;
                         if (parForce == 1) { digitalWriteFast(PPAROUT, LOW);  Serial.println("/PPARITY FORCED LOW"); }
                         else if (parForce == 2) { digitalWriteFast(PPAROUT, HIGH); Serial.println("/PPARITY FORCED HIGH"); }
                         else if (parForce == 3) { pinMode(PPAROUT, INPUT); Serial.println("/PPARITY FLOAT (high-Z; a card pull-up should take it to 5V)"); }
                         else { pinMode(PPAROUT, OUTPUT); Serial.println("/PPARITY = computed"); }
                         if (parForce == 1 || parForce == 2) pinMode(PPAROUT, OUTPUT); }
    else if (c == 'n') { nakIgnore ^= 1;
                         Serial.println(nakIgnore ? "confirm NAK = IGNORED (pre-debug behavior)"
                                                  : "confirm NAK = honored (abort to idle)"); }
    else if (c == 'k') { confirmOverride ^= 1;
                         Serial.println(confirmOverride ? "confirm response = idleDriveByte (CF-OVR ON)"
                                                        : "confirm response = intent (override off)"); }
    else if (c == 'v') { geoIdx = (geoIdx + 1) % (sizeof(geoPresets) / sizeof(geoPresets[0]));
                         volumeBlocks = geoPresets[geoIdx];
                         wasReset = true;   // next command reports "drive was reset",
                                            // which is the retry path finding 20 failed on
                         printGeometry(); }
    else if (c == 'V') { serveFullImage ^= 1;
                         // Decoupling is automatic for a 65533..65535-block image; this
                         // only exists to force the old capped behavior back, so a
                         // suspect failure can be compared against it on the bench.
                         Serial.println(serveFullImage
                           ? "decoupled geometry AUTOMATIC again (engages at 65533..65535 blocks)"
                           : "decoupled geometry FORCED OFF: I/O stops at the claimed count");
                         printGeometry(); }
    else if (c == 'B') { rfreeze = false; rn = 0; rlastF = rlastD = 0xFF;
                         rfree = false; rtrig = 1; rdump = false;
                         Serial.println("boundary capture ARMED (fires at the next completed READ; dump with ?)"); }
    else if (c == 'g') { wireBlockBytes = (wireBlockBytes == BLOCK_DATA) ? BLOCK_TOTAL : BLOCK_DATA;
                         Serial.print("wire payload = "); Serial.print(wireBlockBytes);
                         Serial.println(wireBlockBytes == BLOCK_TOTAL ? " bytes (data + TAGS)"
                                                                      : " bytes (data only, no tags)"); }
    else if (c == 'N') { selForget();
                         Serial.println("saved selection FORGOTTEN: menu forced at next boot"); }
    else if (c == 'm') { Serial.print("images: "); Serial.println(selCount);
                         for (int i = 0; i < selCount; i++) {
                           Serial.print((selValid && i == selCurrent) ? " *" : "  ");
                           Serial.print(i); Serial.print("  ");
                           Serial.print(selCatalog[i].name);
                           Serial.print("   "); Serial.print(selCatalog[i].blocks);
                           Serial.print(" blocks   "); Serial.println(selCatalog[i].path);
                         }
                         Serial.print("selector "); Serial.println(selectorArmed ? "ARMED" : "fired"); }
    else if (c == 'S') { backupRestore(0); }
    else if (c == 'L') { backupRestore(1); }
    else if (c == 'y') { wrConfirmByte = idleDriveByte;
                         Serial.print("write-confirm byte = $"); Serial.println(wrConfirmByte, HEX); }
    else if (c == 'T') { printTiming(); }
    else if (c == 'X') { clearTiming(); Serial.println("timing counters CLEARED"); }
    else if (c == '?' || c == '\r' || c == '\n') { printStatus(); if (rdump) printRecorder(); }
    else {
      if (c >= '0' && c <= '9') v = c - '0';
      else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
      else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
      if (v >= 0) { idleDriveByte = (uint8_t)((idleDriveByte << 4) | v);
                    Serial.print("idle-drive = $"); Serial.println(idleDriveByte, HEX); }
    }
  }

  // Flush any buffered writes once the bus has been idle for a moment. Keeps the
  // image consistent on the card without paying a sync inside a transaction.
  // Fires 500 ms after the LAST write, so a burst is never interrupted by a sync.
  //
  // The lastTxnMs term is the "bus has been idle" half, and it was missing until
  // 2026-08-29: the condition measured only time since the last WRITE, so reads did
  // not defer it. That is exactly the boot case. If the //e was reset within 500 ms
  // of a write, writesSinceSync is still set when the card starts its boot reads,
  // and the sync then fires BETWEEN two of those reads. img.sync() is a blocking
  // SdFat FAT+directory flush; a microSD that picks that moment to run an internal
  // erase can stall for well over the card's command timeout, and the ROM gives up
  // and drops to Applesoft with no boot device.
  //
  // This is a correctness fix, NOT the explanation for the intermittent reset-to-]
  // failure: that one reproduces in write-free boot loops, where writesSinceSync is
  // zero and this branch never arms. Keep the fix, look elsewhere for that bug.
  if (writesSinceSync && haveImage && millis() - lastWriteMs > 500
      && millis() - lastTxnMs > 250) {
    img.sync(); if (haveTag) tag.sync();
    writesSinceSync = 0;
  }

  // Faithful idle: pins OUTPUT (driving idleDriveByte) when PR/-W is high so the always-
  // enabled data '245's A-side never floats (a floating '245 input shoots through and
  // overheats); INPUT when PR/-W is low so the host owns the bus. Re-apply on a PR/-W
  // change OR when idleDriveByte was just changed over serial.
  int8_t prw = digitalReadFast(PRWIN) ? 1 : 0;
  static uint8_t lastDriven = 0x00;
  if (prw != idlePRW || (prw && !idleTristate && idleDriveByte != lastDriven)) {
    if (prw && !idleTristate) { presentByte(idleDriveByte); dataPinsOutput(); lastDriven = idleDriveByte; }
    else                      { dataPinsInput(); }
    idlePRW = prw;
  }
}
