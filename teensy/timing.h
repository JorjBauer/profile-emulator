#pragma once
#include <Arduino.h>

/*
 * Timing instrumentation for the ProFile emulator.
 *
 * Attribution for "it boots but it's slow". Every segment of a transaction is
 * timed with the DWT cycle counter and accumulated into RAM; nothing is printed
 * until the bus is idle and the operator asks ('T' dumps, 'X' clears). A stamp
 * is one CYCCNT load plus a 64-bit add, so this is far cheaper than the rrec()
 * recorder and does not perturb the protocol (same rule as the liveness
 * counters: RAM stores mid-transaction are safe, serial output is not).
 *
 * Reading the report:
 *   * rd-data us/byte is what the HOST is spending per byte. If that is already
 *     in the tens of microseconds we are host-bound and micro-optimizing our
 *     poll loops buys nothing.
 *   * sd-load is pure SdFat cost (seek + read). If it rivals rd-data, the
 *     cluster-chain walk is the problem, not the bus.
 *   * gap is host think-time between transactions: idle time we do not control.
 *   * txn-BAD is time burned in aborts. A one-second waitPin timeout hides
 *     here, and a handful per boot dwarfs everything else in the table.
 *
 * This lives in a header rather than the .ino because the Arduino preprocessor
 * hoists auto-generated prototypes above any struct defined in the sketch.
 */

#define TIMING 1

struct Acc { uint64_t sum; uint64_t max; uint32_t n; };   // all values in CPU CYCLES

// Segments of a transaction, in the order they occur.
extern Acc aP1, aCmd6, aCf, aLoad, aRd, aWrRecv, aWrCf, aStore, aWrStat;
// Whole-transaction rollups: completed, aborted, and the host gap between them.
extern Acc aTxn, aTxnBad, aGap;

extern uint32_t tTxn0, tSeg0;      // file scope so `goto abort` cannot cross an init
extern Acc     *curSeg;            // segment in flight, so the abort path still books it
extern uint32_t txnPrevEndUs;      // micros() at the end of the previous transaction
extern uint32_t timingStartMs;     // window start, for "of the last N ms, X was SD"

static inline void accAdd(Acc &a, uint64_t cyc) {
  a.sum += cyc;
  if (cyc > a.max) a.max = cyc;
  a.n++;
}

#if TIMING
  // Open a segment: remember the start and which bucket it belongs to.
  #define TSEG(a)   do { curSeg = &(a); tSeg0 = ARM_DWT_CYCCNT; } while (0)
  // Close it. Safe to call with no segment open (the abort label does exactly that).
  #define TSEGEND() do { if (curSeg) { accAdd(*curSeg, (uint32_t)(ARM_DWT_CYCCNT - tSeg0)); \
                                       curSeg = nullptr; } } while (0)
  #define TTXN0()   do { tTxn0 = ARM_DWT_CYCCNT; } while (0)
  #define TTXNEND(ok) do { accAdd((ok) ? aTxn : aTxnBad, (uint32_t)(ARM_DWT_CYCCNT - tTxn0)); \
                           txnPrevEndUs = micros(); } while (0)
#else
  #define TSEG(a)     do { } while (0)
  #define TSEGEND()   do { } while (0)
  #define TTXN0()     do { } while (0)
  #define TTXNEND(ok) do { } while (0)
#endif
