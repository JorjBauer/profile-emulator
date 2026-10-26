#!/usr/bin/env node
// sweep_decode.js: print the SD clock sweep's results from OpenOCD's output.
//
//   openocd -f tools/ecp5_er1.cfg 2>&1 | node tools/sweep_decode.js
//
// (`make r2-sweep-read` does that.) The words come out in shift order, word
// 0 first, between the markers "PESW" and "END!". src/sd_sweep.v has the
// layout of each word.

'use strict';

const text = require('fs').readFileSync(0, 'utf8');
const words = (text.match(/^[0-9a-f]{8}$/gm) || []).map((w) => parseInt(w, 16));

if (words.length < 24 || words[0] !== 0x50455357 || words[23] !== 0x454e4421) {
   console.log('no sweep frame here. Is this a SWEEP_ENABLE build?');
   console.log(text);
   process.exit(1);
}

const STATES = ['IDLE', 'OP', 'MBR', 'SIG', 'ZSIG', 'ZERO', 'SIGW', 'SIGV',
                'WW', 'WV', 'PW', 'PV', 'RD', 'CLR', 'CLRS', 'END'];
const WHY = ['finished', 'sector 0 unreadable, or no $55 $AA',
             'no gap before the partition (GPT, or it starts below 64)',
             'the sectors in the gap are not blank, so nothing was written',
             'the signature sector would not write',
             'the read pattern would not write'];
const SD = ['NONE', 'CMD0', 'CMD8', 'ACMD41', 'CMD58', 'NOTSDHC', 'CMD17',
            'TOKEN', 'NOTREADY', 'WRITE', 'NOR1', 'WDATA', 'WBUSY'];
const RATES = ['391 kHz', '758 kHz', '1.47 MHz', '2.78 MHz', '5.0 MHz',
               '8.33 MHz', '12.5 MHz', '25 MHz'];
const DIVS = [63, 32, 16, 8, 4, 2, 1, 0];

const st = words[1];
const hex = (v) => (v >>> 0).toString(16).toUpperCase().padStart(8, '0');

console.log(`build ${hex(words[3])}, partition at sector ${words[2]}`);
console.log(`sweep ${st & 1 ? 'done' : 'still running, in ' + STATES[st >>> 28]}:`
            + ` ${WHY[(st >>> 24) & 15] || 'reason ' + ((st >>> 24) & 15)}`);
console.log(`zeros put back: ${(st >>> 20) & 1 ? 'yes' : 'no'}`);

function table(name, first) {
   console.log(`\n${name}:`);
   for (let i = 0; i < 8; i++) {
      const w = words[first + i];
      const rate = `${RATES[i].padStart(8)} (DIV ${String(DIVS[i]).padStart(2)})`;
      if (!(w >>> 31)) { console.log(`  ${rate}  not tried`); continue; }
      const code = (w >>> 24) & 15, retries = (w >>> 16) & 255, bad = w & 0xffff;
      let out = `  ${rate}  ${(w >>> 30) & 1 ? 'pass' : 'FAIL'}`;
      if (bad) out += `, ${bad === 0xffff ? '65535 or more' : bad} bad bytes`;
      if (retries) out += `, ${retries} retries`;
      if (code) out += `, SD error ${SD[code] || code}`;
      console.log(out);
   }
}

table('writes (32 sectors each, read back at 391 kHz)', 4);
table('reads (32 sectors, 4 times each)', 12);

const sd = words[20];
console.log(`\nSD engine now: error ${SD[(sd >>> 16) & 15] || (sd >>> 16) & 15},`
            + ` ${(sd >>> 8) & 255} retries since reset, state ${sd & 31}`);
