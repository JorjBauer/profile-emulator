#!/usr/bin/env node
// Check a block streamed onto the DB-25 against the card image.
//
// `make stream-prog SBLOCK=n` builds a bitstream that reads block n off the SD
// card and clocks all 512 of its bytes onto the ProFile bus, repeatedly, with no
// host involved. The sniffer captures them. This compares what came out of the
// DB-25 against the same block read from an image of that card, which checks the
// entire read path on real hardware:
//
//     SD card -> sd_spi -> block buffer -> block_router -> profile_bus
//             -> the data pads -> 74LVC245 -> DB-25 -> sniffer
//
// Usage:
//
//     (capture the sniffer's trace to a file, then)
//     ./check_stream.js trace.txt ~/Desktop/4gb-dump.img --block 0
//
// TEARING. The sniffer loads GPIO6, GPIO7 and GPIO9 as three separate reads, so
// the first sample of any transition can show a mix of the old and new byte. The
// settled value lands about 76 ns later and then persists for the whole dwell.
// So a sample counts only if the value HELD; filtering on the gap to the next
// sample reports a bus full of faults that are not there.

const fs = require('fs');

const SECTOR = 512;
const ROW = /^\+\s*(\d+)\s+([0-9A-F]{2})\s+(\d)\s+(\d)\s+(\d)\s+(\d)\s+(\d)\s+(\d)\s+(\d)\s+(\d)/;

// Return every sample, and the data bytes that actually sat on the bus.
//
// Grouping is by HOW LONG THE DATA BYTE HELD, not by the gap to the next
// sample. The sniffer records a sample whenever ANY tapped line changes, and
// with no host the host-driven lines float and flap continuously, so most
// samples are control-line noise with the data byte unchanged. A rule based on
// the gap to the next sample therefore discards perfectly good bytes that
// happen to be followed by a /PCMD glitch, and reports a bus full of
// mismatches against data that is in fact correct.
function parse(path, holdNs) {
  const rows = [];
  for (const raw of fs.readFileSync(path, 'latin1').split('\n')) {
    const m = ROW.exec(raw.trim());
    if (m) {
      const ctl = [];
      for (let i = 3; i <= 10; i++) ctl.push(parseInt(m[i], 10));
      rows.push([parseInt(m[1], 10), parseInt(m[2], 16), ctl]);
    }
  }

  // Absolute time of each sample, then runs of a constant data byte.
  let t = 0;
  const stamped = rows.map(([dt, d, ctl]) => { t += dt; return [t, d, ctl]; });

  const held = [];
  let i = 0;
  while (i < stamped.length) {
    let j = i;
    while (j + 1 < stamped.length && stamped[j + 1][1] === stamped[i][1]) j++;
    // How long this value survived: until the next DIFFERENT value appeared.
    if (j + 1 < stamped.length) {
      const dur = stamped[j + 1][0] - stamped[i][0];
      if (dur >= holdNs) held.push([dur, stamped[i][1], stamped[i][2]]);
    }
    i = j + 1;
  }
  return { rows, held };
}

// Longest stretch of the capture that matches the block exactly.
//
// A single global rotation is the WRONG metric here. The reconstruction
// occasionally inserts or drops a byte, and one insertion shifts every later
// byte. A perfectly good capture then scores near zero, while its mismatch list
// plainly shows the right data three places along. The block is treated as
// cyclic because the streamer repeats it.
function longestRun(seq, block) {
  const n = block.length;
  let best = [0, 0, 0];              // length, seq index, block offset
  for (let si = 0; si < seq.length; si++) {
    for (let off = 0; off < n; off++) {
      if (seq[si] !== block[off]) continue;
      let k = 0;
      while (si + k < seq.length && seq[si + k] === block[(off + k) % n]) k++;
      if (k > best[0]) best = [k, si, off];
    }
    if (best[0] > seq.length - si) break;   // nothing later can beat it
  }
  return best;
}

// How much of the capture matches the block somewhere, allowing resync.
function coverage(seq, block) {
  let matched = 0, runs = 0, i = 0;
  while (i < seq.length) {
    const [k, si] = longestRun(seq.slice(i), block);
    if (k < 4) { i += 1; continue; }        // too short to be anything but chance
    matched += k; runs += 1; i += si + k;
  }
  return [matched, runs];
}

const intAuto = (s) => (/^0[xXoObB]/.test(s) ? Number(s) : parseInt(s, 10));
const hx = (v) => v.toString(16).toUpperCase().padStart(2, '0');

function parseArgs(argv) {
  const a = { block: 0, minRun: 64, maxRun: 8, holdNs: 5000 };
  const rest = [];
  for (let i = 0; i < argv.length; i++) {
    switch (argv[i]) {
      case '--block':   a.block  = intAuto(argv[++i]); break;
      case '--min-run': a.minRun = intAuto(argv[++i]); break;
      case '--max-run': a.maxRun = intAuto(argv[++i]); break;
      case '--hold-ns': a.holdNs = intAuto(argv[++i]); break;
      case '-h': case '--help':
        console.log('usage: check_stream.js <trace> <image> [--block N]');
        console.log('       [--min-run N] [--max-run N] [--hold-ns N]');
        process.exit(0);
      default:
        if (argv[i].startsWith('-')) { console.error(`unknown option ${argv[i]}`); process.exit(2); }
        rest.push(argv[i]);
    }
  }
  [a.trace, a.image] = rest;
  return a;
}

function main() {
  const args = parseArgs(process.argv.slice(2));
  if (!args.trace || !args.image) {
    console.error('usage: check_stream.js <trace> <image> [--block N]');
    return 2;
  }

  const { rows, held } = parse(args.trace, args.holdNs);
  if (!rows.length) { console.error('no trace rows parsed: is this a sniffer capture?'); return 1; }

  const block = Buffer.alloc(SECTOR);
  const fd = fs.openSync(args.image, 'r');
  const got = fs.readSync(fd, block, 0, SECTOR, args.block * SECTOR);
  fs.closeSync(fd);
  if (got < SECTOR) { console.error(`image has no block ${args.block}`); return 1; }

  // RUN-LENGTH RECONSTRUCTION. Identical consecutive bytes produce no bus
  // transition, so the sniffer records nothing and a run of N looks like one
  // byte. The duration says how many there were. The dwell is inferred rather
  // than assumed, so this needs no agreement with SDWELL.
  //
  // Anything far longer than a plausible run is the gap between repeats, when
  // the bus is released and floats at its last value. Those are boundaries, not
  // data, which is why the block chosen for the test should be data-rich: on an
  // MBR, almost all zeros, a 400-byte run of $00 and the inter-repeat gap are
  // the same length and cannot be told apart.
  //
  // The MEDIAN group duration, not the minimum. Most groups are a single byte,
  // so the median IS one dwell. The minimum is whatever tear happened to land
  // just above the hold threshold, which would expand every byte into several.
  let dwell = args.holdNs;
  if (held.length) {
    const durs = held.map((h) => h[0]).sort((a, b) => a - b);
    dwell = durs[Math.floor(durs.length / 2)];
  }
  const seq = [];
  let dropped = 0;
  for (const [dur, d] of held) {
    let n = Math.round(dur / dwell);
    if (n < 1) n = 1;                    // shorter than a dwell: a tear, count once
    if (n > args.maxRun) { dropped += 1; continue; }   // the release gap
    for (let k = 0; k < n; k++) seq.push(d);
  }
  console.log(`-- ${rows.length} samples, ${held.length} held groups, ` +
              `${seq.length} bytes after run-length reconstruction`);
  console.log(`    inferred dwell ${dwell} ns, ${dropped} gap group(s) dropped`);
  if (!seq.length) {
    console.error('nothing was held long enough to be a real byte; is the dwell shorter than --hold-ns?');
    return 1;
  }

  const [runlen, si, off] = longestRun(seq, block);
  const [matched, nruns] = coverage(seq, block);
  const pct = seq.length ? (100.0 * matched / seq.length) : 0.0;
  console.log(`-- longest exact run : ${runlen} bytes, ` +
              `capture index ${si} against block offset ${off}`);
  console.log(`    total matched    : ${matched} of ${seq.length}  (${pct.toFixed(2)}%) ` +
              `in ${nruns} run(s)`);
  if (runlen) {
    const take = Math.min(runlen, 16);
    const gotS  = seq.slice(si, si + take).map(hx).join('');
    let wantS = '';
    for (let k = 0; k < take; k++) wantS += hx(block[(off + k) % SECTOR]);
    console.log(`    first 16 of that run, bus   : ${gotS}`);
    console.log(`    first 16 of that run, image : ${wantS}`);
  }

  // Byte pacing, as a free side effect of the ns timestamps.
  const gaps = held.map((h) => h[0]).sort((a, b) => a - b);
  if (gaps.length)
    console.log(`-- byte period: median ${gaps[Math.floor(gaps.length / 2)]} ns, ` +
                `min ${gaps[0]}, max ${gaps[gaps.length - 1]}`);

  // Parity has to be right for every settled byte, in this direction too.
  const popcount = (v) => { let c = 0; while (v) { c += v & 1; v >>= 1; } return c; };
  let parbad = 0;
  for (const [, d, c] of held) if (c[5] !== (popcount(d) % 2 ? 0 : 1)) parbad++;
  console.log(`-- parity wrong on ${parbad} of ${held.length} settled bytes`);

  // A long exact run is the real evidence. Demanding 100% would be demanding
  // that the sniffer never tears, which it does by construction.
  if (runlen >= args.minRun) {
    console.log(`\nPASS: ${runlen} consecutive bytes off the DB-25 match the image ` +
                `exactly (${pct.toFixed(1)}% of the capture matched overall)`);
    return 0;
  }
  console.log(`\nFAIL: longest exact run was only ${runlen} bytes ` +
              `(wanted at least ${args.minRun})`);
  return 1;
}

process.exit(main());
