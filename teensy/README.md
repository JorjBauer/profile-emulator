# ProFile drive emulator (Teensy 4.1)

In 2021 I built a profile dumper around a Teensy 4.1; this is the
mirror image of that. Instead of pretending to be the Apple //e host
(and reading a real ProFile), it pretends to be a ***ProFile drive***
and answers a real Apple host (could be an Apple ][ series or an Apple
///, although I've only tested this on an Apple //e). It serves block
images from the built-in microSD.

The ProFile card I used for testing here came with a ProFile 10, and
is ROM version 341-0299-B. Despite coming with a 10MB drive, the ROM
actually supports up to 31.998 MB natively. Yes, that's a weird
number; it's the nearest block multiple of 4 (65532 blocks, to be
precise) that can be supported because of special block numbers used
by the ProFile drive to talk about spare blocks and status
information.

## Hardware

The board that I hand-wired was not documented at the time, and I'm
adding this 5 years later; so it's a description of "as-built".  Key
differences from the reader board:

- The '245 convention is **mirrored**: A-side = Teensy, B-side = ProFile.
- The **data '245's DIR pin is wired to PR/-W in hardware**, not to a GPIO. The
  buffer direction follows the host automatically; firmware's only job is to make
  the Teensy's own data pins follow PR/-W too (input when the host writes, output
  when it reads) so it never fights the '245.
- Host-driven control lines are buffered, **read-only** inputs; the drive-driven
  BSY/PARITY are **direct** Teensy outputs.

| Teensy | Signal | Dir | Notes |
|-------:|--------|-----|-------|
| 2 to 9 | D7 to D0 | bidir | pin 2 = MSB; A-side of the data '245; direction follows PR/-W |
| 27 | PR/-W | in | host read/write; **also** the hardware data-'245 DIR |
| 28 | /PSTRB | in | host strobe (paces every byte) |
| 29 | /PCMD | in | host command |
| 30 | /PRES | in | host reset |
| 31 | PH0 | in | host phase clock (buffered, available; not required) |
| 25 | /PBSY | out | drive busy (direct) |
| 26 | /PPARITY | out | drive odd parity (direct) |
| 11 | /OE | out | enables both '245s (active low) |

Pins 10 and 32 are unused in this role.

## How it works

The drive is the passive side: the host sets the pace and this firmware answers.
Each transaction is the inverse of the reader's sequence (see `PROTOCOL.md`):

1. **Command phase.** Host drops /PCMD; we assert BSY and put `$01` on the bus;
   host acks `$55`; we deassert BSY; host clocks in the **6-byte command frame**.
2. **Confirmation phase.** Host drops /PCMD again; we answer the intent byte
   (`$02` read, `$03` write, `$04` write/verify) and take the `$55` ack. BSY stays
   asserted while we hit the SD card, then we deassert it.
3. **Data phase.** READ: 4 status bytes then the payload, each byte clocked out by
   the host's /PSTRB. WRITE: we receive the payload, take a second confirmation
   (`$06`), write to SD, then send 4 status bytes.

The payload length is not the same in both directions on a //e, and neither is
532. A read is 4 status plus **512** data, because the card abandons the transfer
before the 20 tag bytes. A write is **512 plus a 6-byte trailer**. Both are card
behaviors, traced in `PROTOCOL.md` §9.1 and §9.3. The 532-byte path still
exists, and the `g` command switches to it for the reader loopback. The geometry
reported in the synthesized spare table is 532 bytes/block regardless, which is
what a real ProFile reports.

Because the data '245 direction is the host's PR/-W, the firmware
reads PR/-W and flips only its **own** pin direction to match, exactly
what a real ProFile's Z8 does with its `P01M` register. PH0 is not
needed (the drive is /PSTRB-paced); it is buffered on pin 31 only as
an optional timing reference.

## Building the firmware

```
make deps      # once: install the Teensy platform
make upload    # compile + flash a connected Teensy 4.1
make monitor   # watch the serial log
```

The "teensy-discovery not installed" notice during compile is cosmetic.

## Making an image, starting from nothing

You do not need a real ProFile, a dump, or any existing
image. Everything below builds a bootable volume from scratch on the
desktop and copies it to the microSD.

Images are made with `wozit`, part of **wozzle**:
<https://github.com/JorjBauer/wozzle>. Build it there and put it on your `PATH`.

**Format new volumes at 65532 blocks:** `$FFFC` blocks, **33,552,384 bytes,
31.998 MB**. That is the largest multiple of 4 that fits in the card ROM's 16-bit
block count, and the multiple-of-4 part is a hard requirement for booting (see
[Volume geometry](#volume-geometry-the-block-count-must-be-a-multiple-of-4)).

Note that it is possible to address a full 32MB volume (the largest ProDOS supports)  by [decoupled geometry](#a-full-32mb-volume-decoupled-geometry-automatic), but I don't recommend you use that if you don't have to.

This is the Wozit command to make the largest native volume:

```
wozit -p -c "format profile32.img PROFILE 65532 bootable"
```

Wozit will not create a bootable volume on its own; you need a ProDOS system, which you can take from any ProDOS disk image you already have:

```
wozit -I profile32.img -p \
  -c "cpin PRODOS PRODOS FF 0000" \
  -c "cpin BASIC.SYSTEM BASIC.SYSTEM FF 2000" \
  -c "save"
```

Then copy `profile32.img` to the **root of the microSD** and put the card in the
Teensy. That is the entire setup.

Note in particular:

- **Use `65532`, not wozit's `32m` preset.** `32m` is 65535 blocks, the ProDOS
  maximum, and 65535 is not a multiple of 4. The firmware would clamp the claimed
  geometry down to 65532 so the //e still boots, but ProDOS would believe in three
  blocks (65532 to 65534) that the emulator then refuses to read or write. Format
  to the size the drive will actually report.
- **Name it `.img` or `.hdv`.** wozit rejects any non-280-block image not named
  one of those two, and the firmware accepts either, in any mix of upper and lower
  case. `.po` will not work at this size, at either end. The examples here use
  `.img`; `wozit -p -c "format profile32.hdv PROFILE 65532 bootable"` is equally
  good.
- The volume name is 1 to 15 characters, letter first, then letters, digits or
  periods, and is upper-cased for you.
- `bootable` installs the standard ProDOS boot block. Booting also needs `PRODOS`
  and a `.SYSTEM` file in the root, which is what the second command supplies.

### Shrinking an image that is too big

A 32MB image that you didn't build is probably the wrong size:
anything over 65532 blocks, or any block count that is a multiple of
4. The firmware won't choke on it; it rounds the claimed geometry down
to a multiple of 4 and caps it at `$FFFC`, so the //e still boots. But
that only fixes what the *drive* reports: the ProDOS volume inside the
image still believes its original block count, and blocks above the
cap are ones the filesystem thinks it owns but the emulator refuses to
touch. This is a messy place to be in, and it's best if you can use
`wozit` to shrink the volume in place:

```
wozit -I profile32.img -p -c "resize 65532" -c "save"
```

That rewrites `total_blocks` in the volume directory header, rebuilds the bitmap
for the smaller volume, and truncates the image file itself.

- **It only shrinks.** Growing a volume means formatting a bigger image and
  copying the contents in.
- **It will not destroy data.** If anything is allocated at or past the
  new end, `resize` first tries to relocate those blocks down into free space. If
  there is not enough room it reports what is in the way and changes nothing:
  `94 blocks sit at or past block 700, but only 0 free blocks below it`. Add `-n`
  (`resize -n 65532`) to refuse outright rather than trying to relocate.
- It checks the volume bitmap *and* walks the directories and files, so a volume
  whose bitmap under-reports what is really allocated is still caught.
- It works on raw block images (`.img`, `.hdv`, `.po`), not on a 280-block floppy
  image, whose size is fixed.

### A full 32MB volume: decoupled geometry (automatic)

Time will probably come when you want to run a full 32MB volume that
can't be resized (e.g. Total Replay, an image that uses the whole
32MB).

**The firmware handles this by itself.** An image of **65533 to 65535 blocks** is
served in full while the drive continues to CLAIM the aligned 65532.

That works because the multiple-of-4 rule constrains only what the drive claims,
not what it can serve. The rule is one instruction in the card ROM's boot path,
`$CBFD: lda $46 / and #$03 / bne`, guarding the `jsr $CC75` handoff, and `$46`
holds the low byte of the count we reported. The data path does not bounds-check a
block number against that count: finding 22 measured a successful READ of **block
40000 while the drive was claiming 19456 blocks**. ProDOS takes the volume's real
size from `total_blocks` in block 2 of the image, not from the drive, so the
filesystem sees the whole volume and its requests for the top three blocks pass
through the driver unchecked.

**Why only 65533 to 65535.** Rounding the claim down means *any*
misaligned image has one to three blocks above its claim, not just the
big ones. Only this band is served past the claim, because it is the
only range where the aligned claim is unreachable. Anywhere below it,
`wozit resize` to a multiple of 4 is available, costs nothing, and
keeps the drive in a format that is known to work. A smaller
misaligned image is still capped, and the startup log says so and
points at `resize`.

At startup an image in the band reports:

```
Image open: 65532 blocks
NOTE: the image holds 65535 blocks, 3 more than can be CLAIMED (multiple-of-4 boot rule).
      DECOUPLED GEOMETRY IS ACTIVE: all of them are served anyway.
```

and the status line carries `srv=$FFFF` beside `vol=$FFFC`.

**Utilities that ask the driver how big the device is** will be told 65532. A copier or formatter could truncate or refuse on that basis.

`V` turns off decoupled geometry and enforces capped behavior, if you need it.

### What the firmware expects on the card

- Any number of **`.img` or `.hdv`** files, anywhere on the card. The two name
  the same thing, a flat stream of 512-byte ProDOS-order blocks, and the firmware
  treats them identically. **Matching is case-insensitive**, so `PROFILE32.IMG`,
  `profile32.img` and `Profile32.Hdv` are all found; a card written by a DOS-era
  tool that upper-cases everything works without renaming. `.po` is deliberately
  not accepted: nothing writes one bigger than an 800K floppy, so it could never
  be a ProFile volume.
- The firmware scans the whole card, validates each candidate as a real ProDOS
  volume, and offers a menu (see
  [Boot-time image selector](#boot-time-image-selector)). Only if the scan finds
  nothing at all does it fall back to opening a fixed name, trying
  `profile32.img` / `.hdv` and then `profile10.img` / `.hdv`.
- The volume size is taken from the image file's length, then rounded down to a
  multiple of 4 blocks and capped at `$FFFC`.
- A `.tag` file beside an image (`foo.img` or `foo.hdv` -> `foo.tag`, 20 bytes
  per block) is **optional**. Without one, tags are served as zeros. Tags are
  always paired with whichever image is open and never mixed across volumes. They
  do not cross the bus on a //e anyway. The sibling name comes from stripping the
  image's own extension, so don't put `foo.img` and `foo.hdv` in the same
  directory: two different volumes would share one `foo.tag` and one `foo.bak`.
- With **no** image on the card at all, the emulator serves a self-verifying test
  pattern instead, which is what makes the loopback test below work unattended.
- Two blocks are synthesized in firmware and are not backed by the image:
  `$FFFFFF` returns a spare/status table built to match the volume, and `$FFFFFE`
  is a RAM scratch buffer.

The original profile-dumper project has a `c` command that dumps exactly this format: a .img and a .tag file.

## Volume geometry: the block count MUST be a multiple of 4

**The //e cannot boot from the drive unless the block count reported to the host is
a multiple of 4.** This is a constraint of the card ROM's boot path, measured on
hardware (full detail and the ROM trace in `PROTOCOL.md` §9.2):

| claimed blocks | low byte | mod 4 | boots |
|---------------:|:--------:|:-----:|:-----:|
| `$4C00` = 19456 (10MB) | `$00` | 0 | YES |
| `$FE00` = 65024 (31.75MB) | `$00` | 0 | YES |
| `$FEFF` = 65279 | `$FF` | 3 | no |
| `$FFFE` = 65534 | `$FE` | 2 | no |

Size is **not** the limit: 65024 boots and 65279 does not. A full-size 32MB ProFile
works. The usable maximum is 65532 blocks (`$FFFC`); 65024 (`$FE00`) is the tested
value. The ceiling is 16-bit because the ROM discards the spare table's high byte,
which is also exactly the ProDOS block-device limit.

The firmware rounds the claimed geometry DOWN to a multiple of 4 and caps it at
`$FFFC`, so no image size can break the boot. `v` sweeps the geometries above for
testing; the status line reports `vol=$xxxx`, and a non-multiple-of-4 claim prints
an explicit warning.

When a boot fails this way it never reads a single real block. Note
this applies only to the ROM's **boot** path. Through the ProDOS
block-device entry (`JSR $C5EA`) a STATUS call returns the correct
count at `$FFFE` and `$FFFF` too, and reads at high block numbers
succeed, so a misaligned volume still works as a data volume mounted
by a ProDOS booted from elsewhere.

## Boot-time image selector

Hold **open-apple at boot** to choose which image the emulator serves.

The emulator scans the whole card for `.img` and `.hdv` files (either
case), validates each as a real ProDOS volume, and lists them by
PATH in the order they are found. The choice is persisted **by path**
in `profile.cfg`, so the card can be pulled and the catalog
rearranged and the same image is still selected. If the saved path no
longer resolves, or there is no saved choice, the menu is forced
regardless of open-apple and NOTHING is mounted until the user picks:
a stale config can never serve the wrong volume.

Serial commands: `m` lists the catalog, `N` forgets the saved selection.

Full protocol and the //e-side program are in the submodule project profile-selector.

## Write path

A write on the //e is **512 data bytes plus a 6-byte trailer**, not 512 and not
532. The trailer (`$00, $47, $46, $FF, $FF EOR $47, $46 EOR $FF`, a block-number
echo with complements) is sent by the card ROM at `$CD82` immediately after the
data, and the ROM's parity check at `$CD5D` runs only after it. Consuming just
512 leaves those six bytes unread and unparitied. The card then latches a
parity error, and every write returns ProDOS `$27` while appearing to succeed
on the drive side. The firmware receives `BLOCK_DATA + 6`; see `PROTOCOL.md`
§9.3, which also explains why a real ProFile never noticed the trailer.

Writes are OFF by default (`writeProtect`); press `w` to enable. The
status line shows `RO` while they are disabled. This project was a
PoC, not intended to be in general use as a writable system, and
writing is not well tested.

### Backing up and restoring the image

A surface test scribbles over the whole volume, so the firmware can copy the
image on the card itself, with no need to pull the SD:

- **`S`** save: copy the open image to its `.bak` sibling
- **`L`** load: restore that backup over the image

Both names are derived from the image that is actually OPEN (`/Games/foo.img` ->
`/Games/foo.bak`), not from a fixed table, so they follow the selector.

Both close the image, copy in 16 KB chunks with progress, and reopen it. Do this
only at idle: the copy takes seconds and the bus is not serviced during it.

## Loopback test (reader against emulator)

- **Cable:** straight-through DB-25, **pin N to pin N** (no crossover). A real
  ProFile system uses a straight host-to-drive cable with pin 19 cut; the reader mimics the host
  and the emulator mimics the drive, so every signal is driven by one side and
  read by the other on the same pin. Add a gender changer if both connectors are
  female. All the GND pins tie together, giving the two Teensys a common ground.
- **Power:** each Teensy over its own USB. PCHK/POCD (grounded on the emulator)
  make the reader see a "present" drive.
- **Sequence:** bring the reader up first (so it holds /PCMD high), then reset the
  emulator, then drive the test from the reader's serial monitor.
- **Run it:** send **`t`** on the reader. That reads block 0 with the 532-byte
  block size, which matches what the emulator sends (4 status + 512 data + 20 tag
  = 536 bytes). With no image on the emulator's SD you get the self-verifying test
  pattern, so block 0 should dump as `A5 00 00 00 04 05 06 ...`; press `t` again
  for block 1 (`A5 00 00 01 05 06 07 ...`), etc. That checks the handshake, the
  data phase, and block addressing end to end.
- **Note:** use `t`, not `h`/`r`: those read with the 512-byte block size and end
  20 bytes short of what the emulator sends, so the emulator will log a harmless
  timeout on the trailing tag bytes (the reader still gets valid data). Set
  `#define TRACE 1` in the sketch if you want the emulator to log phase
  transitions during this slow test.

## Serial command reference

All of this is safe to use at idle; NONE of it is safe
mid-transaction, because output takes tens of microseconds and the
protocol is hard real-time (see the `PROTOCOL.md` §3 timing note, and
note that a `?` during a run once caused 62 NAKs by itself).

| key | effect |
|-----|--------|
| `?` or RETURN | status line, plus any pending recorder dump |
| `T` | timing report |
| `X` | clear the timing counters |
| `m` | list the image catalog |
| `N` | forget the saved selection (forces the menu next boot) |
| `S` | save the open image to its `.bak` sibling |
| `L` | restore that backup over the image |
| `w` | toggle write protect (ON at reset; `RO` in the status line) |
| `v` | cycle the claimed geometry through the test values |
| `V` | force decoupled geometry OFF (it is automatic; this is for A/B on the bench) |
| `g` | toggle the wire payload between 512 and 532 bytes |
| `y` | commit the current idle-drive byte as the write-confirm byte |
| `R` | arm the free-running line recorder |
| `B` | arm the one-shot boundary capture (fires at the next completed read) |
| `p` | cycle `/PPARITY`: computed, forced LOW, forced HIGH, floating |
| `s` / `h` | assert / release `/PBSY` |
| `z` / `i` | tri-state the data pins / resume idle-drive |
| `n` | toggle NAK-ignore (pre-debug behavior) |
| `o` | toggle response override (answer phase 1 with the idle-drive byte) |
| `k` | toggle confirm override (answer the confirm with the idle-drive byte) |
| `0`-`9`, `a`-`f` | shift a hex digit into the idle-drive byte |

`n`, `o`, `k`, `p`, `R`, `B` are card-repair instruments and are not needed in
normal use.

## Timing instrumentation

Every segment of a transaction is timed with the DWT cycle counter and
accumulated in RAM (`timing.h`). Nothing prints during a transaction; the report
is on demand from the serial monitor:

- **`T`** dump the timing table
- **`X`** clear the counters (also cleared at reset)
- **`?`** the status line, including `rdOK` / `abort` / `nak` / `runt`

A stamp is one cycle-counter read plus a 64-bit add, so unlike serial output it
does not perturb the protocol. The table looks like this (illustrative numbers):

```
---- timing over 12000 ms (in-transaction 5700 ms = 47%) ----
  segment   n         total_ms     mean_us     worst_us
  txn-OK    300        2700.00     9000.00     12000.00
  txn-BAD   3          3000.00  1000000.00   1000000.00
  rd-data   300        2400.00     8000.00      9000.00   14.92 us/byte
  sd-load   300         120.00      400.00      2200.00
```

How to read it:

- **`rd-data` us/byte** is what the *host* spends per byte. If that is already in
  the tens of microseconds, the bus is host-paced and micro-optimizing the poll
  loops buys nothing.
- **`sd-load`** is pure SdFat cost (`seek` + `read`). If it rivals `rd-data`, the
  FAT cluster-chain walk is the bottleneck and the fix is a contiguous image plus
  raw `readSector()` calls, not protocol work.
- **`gap`** is host think-time between transactions: idle time the emulator
  does not control.
- **`txn-BAD`** is time burned in aborts. Note `waitPin`'s default timeout is a
  full **second** and `BYTE_TIMEOUT_US` is 50 ms, so a handful of retries per boot
  dwarfs every other row in the table. Check this first.

Caveat: a segment that bails out is still booked (that is where the lost time
went), so a segment's `n` counts *attempts*, not completions. If `rd-data` `n`
exceeds `rdOK` on the `?` line, the difference is partial blocks and the us/byte
figure is diluted.

## Status

Working and measured on hardware (Apple //e, ProFile interface card in slot 5):

- **Boot**: 0.52 s of in-transaction time. See `PROTOCOL.md` §9.
- **Reads**: host-paced at ~12.8 us/byte, which is the //e's speed, not ours.
- **Writes**: the full 31.75 MB surface written and verified, **64,950 blocks,
  zero errors, zero NAKs**.
- **Geometry**: up to 65,532 blocks claimed, and the count MUST be a multiple of 4.
  The firmware rounds down, so no image size can break the boot.
  A 65,533 to 65,535-block volume is served in full automatically while the
  claim stays aligned. Implemented and self-tested, NOT yet verified on hardware
  for writes above the claim.
- **Selector**: works on the stock card with no ROM changes.

Known limitations:

- Write-side parity from the host is received but not verified or reported.
- Only READ / WRITE / WRITE-VERIFY opcodes are handled; the Formatter/Debugger
  extended set is not (this is a drive *emulator*, not a maintenance target).
- Status bytes are minimal (success, "was reset", "block invalid"); no seek or
  CRC modeling.
- Tags never cross the bus on a //e (`wire=512`); the 532-byte path exists and is
  exercised only by the reader loopback.
- Untested selector paths: more than 16 images (the scrolling code), the digit
  quick-select, and ESC.
- Only the //e has been tested. An Apple /// drives the bus faster and may need
  the timing revisited. If so, it's unlikely the Teensy will be able to keep up;
  it just barely keeps up on the Apple //e.
