#!/usr/bin/env node
// import_blobs.js: bring the selector submodule's generated blobs into this build.
//
// The 6502 half of the boot menu lives in ../selector, which is a submodule and
// is responsible for building itself. Its committed, generated artifact is
// selector_blobs.h. This script does NOT run that generator: if the header is
// stale, someone re-makes it over there. All this does is notice that the
// header is newer than our copies and re-derive them, so the two consumers
// cannot drift apart without make saying so.
//
// Reads:  selector_blobs.h        static const uint8_t SEL_BOOT[512] = { 0x.., }
// Writes: <outdir>/sel_boot.hex   one uppercase byte per line, $readmemh format
//         <outdir>/sel_body.hex
//
// The declared array size is the authority on length: src/selector.v reads
// these into fixed-size ROMs, so a short file would leave the tail as x and a
// long one would overflow with no error. Both are padded or refused explicitly.

const fs = require('fs');
const path = require('path');

const [, , headerPath, outDir] = process.argv;
if (!headerPath || !outDir) {
  console.error('usage: import_blobs.js <selector_blobs.h> <outdir>');
  process.exit(2);
}

const src = fs.readFileSync(headerPath, 'utf8');

// One entry per blob the FPGA serves. The .hex name is what selector.v reads.
const BLOBS = [
  { symbol: 'SEL_BOOT', out: 'sel_boot.hex' },
  { symbol: 'SEL_BODY', out: 'sel_body.hex' },
];

let failed = false;

for (const { symbol, out } of BLOBS) {
  // uint8_t NAME[<len>] = { ... };  capture the declared length and the body.
  const re = new RegExp(
    String.raw`uint8_t\s+${symbol}\s*\[\s*(\d+)\s*\]\s*=\s*\{([^}]*)\}`, 'm');
  const m = src.match(re);
  if (!m) {
    console.error(`*** ${symbol} not found in ${headerPath}`);
    failed = true;
    continue;
  }

  const declared = parseInt(m[1], 10);
  const bytes = (m[2].match(/0x[0-9a-fA-F]{1,2}/g) || [])
    .map((h) => parseInt(h, 16));

  if (bytes.length > declared) {
    console.error(`*** ${symbol}: ${bytes.length} bytes for a [${declared}] array`);
    failed = true;
    continue;
  }
  // Pad to the declared size: selector.v's ROM is that big either way, and a
  // short file leaves the tail undefined rather than zero.
  while (bytes.length < declared) bytes.push(0);

  const text = bytes.map((b) => b.toString(16).toUpperCase().padStart(2, '0'))
                    .join('\n') + '\n';
  const dest = path.join(outDir, out);
  fs.writeFileSync(dest, text);
  console.log(`  ${symbol}: ${declared} bytes -> ${dest}`);
}

process.exit(failed ? 1 : 0);
