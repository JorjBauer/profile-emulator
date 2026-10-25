#!/usr/bin/env node
// mkupdate.js: package r2 bitstreams for the flash, and for the SD update.
//
// The flash holds two images and a jump record (see src/fw_update.v):
//
//   0x000000   the primary: 256 bytes of $FF, then its bitstream
//   GOLDEN     the golden copy, laid out the same way
//   0xFFFF00   the jump record that points the ECP5 at the golden copy
//              when the primary fails to configure
//
// That's the layout ecpmulti writes. We don't use ecpmulti itself, because
// it re-serializes every bitstream and drops --compress, --freq and
// --spimode, which r2 needs to answer the //e in time.
//
//   mkupdate.js region <bitstream> <out>
//       the bytes for one image slot: the 256-byte pad, then the bitstream
//   mkupdate.js jump <golden address> <out>
//       the 256-byte jump record (Trellis's Bitstream::generate_jump)
//   mkupdate.js update <bitstream> <BUILD_ID, hex> <board> <out>
//       the SD update file: a 512-byte header, then the primary slot's
//       bytes, padded with $FF to a whole number of 512-byte sectors

const fs = require('fs');

function crc32(buf) {
  let c = 0xFFFFFFFF;
  for (const b of buf) {
    c ^= b;
    for (let k = 0; k < 8; k++) c = (c & 1) ? ((c >>> 1) ^ 0xEDB88320) : (c >>> 1);
  }
  return (c ^ 0xFFFFFFFF) >>> 0;
}

function region(bitPath) {
  const bit = fs.readFileSync(bitPath);
  return Buffer.concat([Buffer.alloc(256, 0xFF), bit]);
}

// 16 dummy bytes, the preamble, 4 more dummies, a zero write to control
// register 0, then JUMP with the read opcode (03h, a plain read) and the
// 24-bit address, then 18 dummies. Padded to 256 with $FF.
function jump(addr) {
  const b = Buffer.alloc(256, 0xFF);
  let i = 16;
  for (const x of [0xFF, 0xFF, 0xBD, 0xB3]) b[i++] = x;
  i += 4;
  for (const x of [0x22, 0, 0, 0, 0, 0, 0, 0]) b[i++] = x;   // LSC_PROG_CNTRL0, value 0
  for (const x of [0x7E, 0, 0, 0, 0x03]) b[i++] = x;         // JUMP
  b[i++] = (addr >> 16) & 0xFF;
  b[i++] = (addr >> 8) & 0xFF;
  b[i++] = addr & 0xFF;
  return b;
}

const [, , mode, ...args] = process.argv;

if (mode === 'region' && args.length === 2) {
  fs.writeFileSync(args[1], region(args[0]));
} else if (mode === 'jump' && args.length === 2) {
  fs.writeFileSync(args[1], jump(parseInt(args[0])));
} else if (mode === 'update' && args.length === 4) {
  const [bitPath, buildHex, board, out] = args;
  let payload = region(bitPath);
  const pad = (512 - (payload.length % 512)) % 512;
  payload = Buffer.concat([payload, Buffer.alloc(pad, 0xFF)]);
  const hdr = Buffer.alloc(512, 0);
  hdr.write('PEFW', 0, 'latin1');
  hdr.write((board + '    ').slice(0, 4), 4, 'latin1');
  hdr.writeUInt32LE(payload.length, 8);
  hdr.writeUInt32LE(crc32(payload), 12);
  hdr.writeUInt32LE(parseInt(buildHex, 16) >>> 0, 16);
  fs.writeFileSync(out, Buffer.concat([hdr, payload]));
  console.log(`${out}: ${payload.length} bytes to write, build ${buildHex.trim()}, ` +
              `CRC ${crc32(payload).toString(16).padStart(8, '0')}`);
} else {
  console.error('usage: mkupdate.js region <bit> <out>\n' +
                '       mkupdate.js jump <golden address> <out>\n' +
                '       mkupdate.js update <bit> <BUILD_ID hex> <board> <out>');
  process.exit(2);
}
