#!/usr/bin/env node
// Compare what the FPGA reads off the SD card against an image of that card.
//
// This is the host half of src/verifier.v's job. The FPGA reads a range of
// sectors and streams one line per checkpoint:
//
//     000000:1D0F
//     000001:8C2B
//
// six hex digits of starting LBA, a separator, four hex digits of CRC16-CCITT
// over every byte in that checkpoint. The separator is ":" normally, and the
// sd_spi error code if a read in that checkpoint failed. That is NOT the same
// thing as the checksum looking wrong. A failed read leaves the CRC at its
// $FFFF seed, and $FFFF is also a perfectly ordinary checksum for real data.
// This script computes the same numbers from an image file and reports the
// first LBA where they disagree.
//
// Typical use, once the image exists:
//
//     ./verify_image.js ~/Desktop/4gb-dump.img --port /dev/cu.usbserial-FT4MG9OV1
//     ./verify_image.js ~/Desktop/4gb-dump.img --port ... --count 8388608 --stride 1024
//     ./verify_image.js ~/Desktop/4gb-dump.img --input capture.txt
//
// The start, count and stride MUST match how the bitstream was built
// (VERIFY_START, VERIFY_COUNT, VERIFY_STRIDE in src/top.v). They are not
// negotiated over the wire: the board just starts talking.

const fs = require('fs');
const { execFileSync } = require('child_process');

const SECTOR = 512;

// sd_spi.v's error codes, so a failed read names itself.
const ERR_NAMES = {
  1: 'E_CMD0', 2: 'E_CMD8', 3: 'E_ACMD41', 4: 'E_CMD58', 5: 'E_NOTSDHC',
  6: 'E_CMD17 rejected', 7: 'E_TOKEN no data token', 8: 'E_NOTREADY',
  9: 'E_WRITE', 10: 'E_NOR1', 11: 'E_WDATA', 12: 'E_WBUSY',
};
const LINE_RE  = /^([0-9A-F]{6})([:0-9A-F])([0-9A-F]{4})\s*$/;
const RETRY_RE = /^#([0-9A-F]{2})\s*$/;   // the run's closing retry count

// CRC16-CCITT, MSB first, init 0xFFFF, no final xor.
//
// THIS DEFINITION IS A CONTRACT with the bit-serial implementation in
// src/verifier.v. --selftest checks it against vectors printed by
// sim/tb_verifier.v, so the two cannot drift apart unnoticed.
const TABLE = new Uint16Array(256);
for (let b = 0; b < 256; b++) {
  let c = b << 8;
  for (let i = 0; i < 8; i++) c = (c & 0x8000) ? ((c << 1) ^ 0x1021) & 0xFFFF
                                              : (c << 1) & 0xFFFF;
  TABLE[b] = c;
}
function crc16(data, crc = 0xFFFF) {
  for (const byte of data) crc = ((crc << 8) & 0xFFFF) ^ TABLE[((crc >> 8) ^ byte) & 0xFF];
  return crc;
}

const hex = (n, w) => n.toString(16).toUpperCase().padStart(w, '0');
// 0x followed by six lowercase hex digits.
const hex08 = (n) => '0x' + n.toString(16).padStart(6, '0');

// Vectors printed by sim/tb_verifier.v. If these ever fail, the Verilog and
// this script have diverged and every comparison below is meaningless.
const VECTORS = [
  [Buffer.alloc(512), 0x1634, '512 bytes of $00'],
  [Buffer.from(Array.from({ length: 512 }, (_, i) => i & 0xFF)), 0x56EE,
   '512 bytes of j & $FF'],
];

function selftest() {
  let ok = true;
  for (const [data, want, name] of VECTORS) {
    const got = crc16(data);
    if (got !== want) ok = false;
    console.log(`    ${got === want ? 'ok  ' : 'FAIL'}  ${name}: got ${hex(got,4)}, want ${hex(want,4)}`);
  }
  return ok;
}

// ---- argument parsing -------------------------------------------------------
// 0x/0o/0b prefixes or plain decimal.
function intAuto(s) {
  const v = /^0[xXoObB]/.test(s) ? Number(s) : parseInt(s, 10);
  if (!Number.isFinite(v)) { console.error(`bad integer: ${s}`); process.exit(2); }
  return v;
}
function parseArgs(argv) {
  const a = { baud: 115200, start: 0, count: 65536, stride: 1, maxReport: 10 };
  const rest = [];
  for (let i = 0; i < argv.length; i++) {
    const t = argv[i];
    switch (t) {
      case '--port':       a.port = argv[++i]; break;
      case '--input':      a.input = argv[++i]; break;
      case '--baud':       a.baud = intAuto(argv[++i]); break;
      case '--start':      a.start = intAuto(argv[++i]); break;
      case '--count':      a.count = intAuto(argv[++i]); break;
      case '--stride':     a.stride = intAuto(argv[++i]); break;
      case '--max-report': a.maxReport = intAuto(argv[++i]); break;
      case '--selftest':   a.selftest = true; break;
      case '-h': case '--help':
        console.log('usage: verify_image.js <image> [--port DEV | --input FILE]');
        console.log('       [--baud N] [--start N] [--count N] [--stride N]');
        console.log('       [--max-report N] [--selftest]');
        process.exit(0);
      default:
        if (t.startsWith('-')) { console.error(`unknown option ${t}`); process.exit(2); }
        rest.push(t);
    }
  }
  a.image = rest[0];
  return a;
}

// ---- the serial port --------------------------------------------------------
// The line settings have to be applied while the fd is OPEN. Opening a /dev/cu.*
// resets them, so configuring beforehand is discarded before the first byte
// arrives. Hold the fd open and then run stty against it.
//
// "min 0 time 20" gives reads a 2 second timeout, so a board that stops talking
// mid-run ends the read instead of blocking forever. The quiet_stop counter
// below exists for exactly that case.
function openSerial(path, baud) {
  const fd = fs.openSync(path, 'r');
  try {
    execFileSync('stty', ['-f', path, String(baud), 'cs8', '-cstopb', '-parenb',
                          'raw', '-echo', 'min', '0', 'time', '20']);
  } catch (e) {
    fs.closeSync(fd);
    console.error(`could not configure ${path}: ${e.message}`);
    process.exit(2);
  }
  return fd;
}

function* linesFromFd(fd, quietStop) {
  const buf = Buffer.alloc(4096);
  let acc = '';
  let idle = 0;
  for (;;) {
    let n = 0;
    try { n = fs.readSync(fd, buf, 0, buf.length, null); } catch (e) { n = 0; }
    if (!n) {
      if (++idle >= quietStop) return;
      continue;
    }
    idle = 0;
    acc += buf.toString('latin1', 0, n);
    let k;
    while ((k = acc.indexOf('\n')) !== -1) {
      yield acc.slice(0, k);
      acc = acc.slice(k + 1);
    }
  }
}

function* linesFromFile(path) {
  const text = fs.readFileSync(path, 'latin1');
  const lines = text.split('\n');
  // A trailing newline must not produce a final empty line: it would be counted
  // as a skipped non-checksum line and inflate that number by one.
  if (lines.length && lines[lines.length - 1] === '') lines.pop();
  for (const ln of lines) yield ln;
}

// ---- main -------------------------------------------------------------------
function main() {
  const args = parseArgs(process.argv.slice(2));

  if (args.selftest) {
    console.log('-- CRC vectors, against sim/tb_verifier.v');
    return selftest() ? 0 : 1;
  }
  if (!args.image) { console.error('an image file is required (or use --selftest)'); return 2; }
  if (!args.port && !args.input) { console.error('one of --port or --input is required'); return 2; }

  // Always run the self-test first. It costs nothing and it is the difference
  // between "the card does not match the image" and "my two CRCs disagree".
  if (!selftest()) { console.error('CRC self-test failed; refusing to compare anything'); return 1; }

  const size = fs.statSync(args.image).size;
  const have = Math.floor(size / SECTOR);
  const wantEnd = args.start + args.count;
  if (wantEnd > have)
    console.log(`!! the image holds ${have} sectors but the run covers ` +
                `${args.start}..${wantEnd - 1}; the tail will be reported missing`);

  const stream = args.input ? linesFromFile(args.input)
                            : linesFromFd(openSerial(args.port, args.baud), 3);
  const img = fs.openSync(args.image, 'r');

  const expected = Math.floor((args.count + args.stride - 1) / args.stride);
  let checked = 0, bad = 0, failedReads = 0, skipped = 0;
  let firstBad = null, retriesUsed = null, grace = 8;
  const errHist = new Map();

  console.log(`-- comparing ${args.count} sectors from LBA ${args.start}, ` +
              `stride ${args.stride} (${expected} checkpoints)`);

  const chunk = Buffer.alloc(SECTOR * args.stride);
  for (const raw of stream) {
    const line = raw.trim();

    const rm = RETRY_RE.exec(line);
    if (rm) { retriesUsed = parseInt(rm[1], 16); continue; }

    const m = LINE_RE.exec(line);
    if (!m) { skipped += 1; continue; }   // reporter status lines, boot noise

    const lba = parseInt(m[1], 16);
    const sep = m[2];
    const readFailed = sep !== ':';
    const errCode = readFailed ? parseInt(sep, 16) : 0;
    const got = parseInt(m[3], 16);

    const n = fs.readSync(img, chunk, 0, chunk.length, lba * SECTOR);
    if (n < chunk.length) {
      console.log(`!! LBA ${hex08(lba)}: image ends before this checkpoint`);
      checked += 1; bad += 1;
      if (firstBad === null) firstBad = lba;
      continue;
    }

    const want = crc16(chunk);
    checked += 1;
    if (readFailed) {
      // Reported by the board itself, not inferred from the checksum. $FFFF is
      // both the CRC seed and a legitimate checksum, so a failed read cannot be
      // recognized from the number alone.
      failedReads += 1; bad += 1;
      if (firstBad === null) firstBad = lba;
      if (bad <= args.maxReport)
        console.log(`   READ FAILED at LBA ${hex08(lba)} (${lba}): ` +
                    `sd_spi error ${errCode} (${ERR_NAMES[errCode] || '?'})`);
      errHist.set(errCode, (errHist.get(errCode) || 0) + 1);
    } else if (got !== want) {
      bad += 1;
      if (firstBad === null) firstBad = lba;
      if (bad <= args.maxReport)
        console.log(`   MISMATCH LBA ${hex08(lba)} (${lba}): ` +
                    `board ${hex(got,4)}, image ${hex(want,4)}`);
    }

    // Do NOT stop the instant the last checkpoint arrives: the "#NN" retry
    // summary comes AFTER it, and leaving early means the count is never read.
    if (checked >= expected) {
      if (retriesUsed !== null) break;
      if (--grace <= 0) break;
    }
  }
  fs.closeSync(img);

  console.log();
  console.log(`    checkpoints compared : ${checked}`);
  console.log(`    expected             : ${expected}`);
  console.log(`    mismatches           : ${bad}`);
  console.log(`      of which read errors: ${failedReads}`);
  if (retriesUsed === null) {
    console.log('    transfers re-issued  : not reported (older bitstream?)');
  } else {
    // A retry that worked leaves no trace in the per-sector lines, so this is
    // the only place "it passed" and "it passed first time" come apart.
    const note = retriesUsed === 0 ? '' : '  <- transients were absorbed';
    console.log(`    transfers re-issued  : ${retriesUsed}${note}`);
  }
  for (const code of [...errHist.keys()].sort((x, y) => x - y))
    console.log(`        error ${code} (${ERR_NAMES[code] || '?'}): ${errHist.get(code)}`);
  if (skipped) console.log(`    non-checksum lines skipped: ${skipped}`);

  if (checked < expected) {
    console.log(`!! only ${checked} of ${expected} checkpoints arrived. ` +
                `The board may still be running, or the range does not match ` +
                `how the bitstream was built.`);
    return 2;
  }
  if (bad) { console.log(`FAIL: first mismatch at LBA ${hex08(firstBad)} (${firstBad})`); return 1; }
  console.log('PASS: every checkpoint matches the image');
  return 0;
}

process.exit(main());
