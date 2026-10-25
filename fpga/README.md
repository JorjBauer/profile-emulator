# ProFile emulator in an FPGA

This is the FPGA version of the drive. It answers the ProFile's DB-25
protocol and serves blocks from an SD card, on a ULX3S or on the r2 board in
`hw-r2/`. The top-level README has the backstory and says where to start; this
is the technical notebook.

`PROTOCOL.md` and `HARDWARE.md` below are the ones in
[profile-dumper](https://github.com/JorjBauer/profile-dumper).

## Preparing the SD card: a file you have to create yourself

Format the card FAT32, put your images in a directory called `HD`, and
then **create `/profile.cfg` in the root of the card by hand**, with
any text editor and any non-zero-length content, so the drive can
remember its state.

**This drive can modify files, but it cannot create them.** Every
write it performs must be in a sector that is already allocated: a
host's write goes into the image's own cluster chain, and remembering
a menu selection rewrites `profile.cfg`'s existing data sector and
then corrects the size in its directory entry. There is no allocator,
no free-cluster search and no FAT writing anywhere in it, because
serving a fixed-size volume never needs one. Only creating a file
would need them. A text file you drop on the card
avoids all that complexity.

The images must be in a directory named `HD` as a practical matter;
the more stuff the descender has to enumerate, the longer it takes for
this device to boot. We have about 0.58s before the Apple ][ ProFile card
ROM gives up; unlike some other cards, it makes one STATUS request and then
bails if it doesn't respond.

## Building it

This builds with an open source toolchain: yosys, nextpnr-ecp5, and
prjtrellis, all of which are part of oss-cad-suite. The build also needs
`node`, which imports the selector's blobs. Source oss-cad-suite's
environment first (`source <wherever you unpacked it>/environment`) and then

    make            synthesize, place and route, pack
    make sim-all    the testbenches. Run these before touching hardware
    make prog       load into SRAM over USB, volatile
    make flash      write SPI flash, survives a power cycle
    make reload     re-load the last SRAM build with no rebuild
    make stat       resource count only: yosys alone, no place and route
    make clean

`make stat` needs nothing but yosys, which makes it the fastest way to see
whether a change still fits.

**SRAM and flash get different bitstreams.** The flash copy is
compressed and configured for fast-read at 38.8 MHz (62 MHz on r2). An
uncompressed stream takes about 2.25 s to load and loses the power-on
race against the //e. Compressed, the same load is about 70 ms.
Loading that same compressed flavor into SRAM over JTAG makes the
ECP5's config engine abort with `BSE Abort ERR` and leaves the device
blank. `make prog` therefore builds and loads the uncompressed
`_nextflash.bit`, and `make flash` the compressed one. Both work via
`fujprog`.

There is one more ECP5 wrinkle worth knowing: this is built as an
ECP5-25F, but I'm using an ECP5-12F to serve it. They're the same
silicon; the 12F has artificial limits that the open source toolchain
is happy to ignore.

Four separate diagnostic bitstreams build from the same sources, each
of which takes the SD engine or the bus for itself and so cannot
coexist with serving a host: `make verify-prog` (whole-volume
checksums), `make dump-prog DLBA=n` (one sector as hex), `make
exercise-prog` (a known pattern on the DB-25 with no host) and `make
stream-prog SBLOCK=n` (a real block onto the DB-25 with no host). Each
is described in its own section below.

### Building for the r2 board

The r2 board in `hw-r2/` is this same design on an LFE5U-12F in a
TQFP-144, with its own pinout in `constraints/profile-drive-r2.lpf`. r2 has
three pins the ULX3S build doesn't, so its top module is `src/top_r2.v`,
which wraps `top` and leaves every other build alone:

- **U3_OE_N** (pin 76) enables U3, the buffer for the host's control lines.
  R53 holds it off until the FPGA is configured, and the gateware ties it low.
  If it isn't driven low, the FPGA never sees /PCMD or /PSTRB.
- **XCVR_OE_N** (pin 74) is held off until the host asserts /PCMD the first
  time, because PD0 to PD7 have no pulls and float with no cable.
- **FPGA_REBOOT** (pin 67) goes to PROGRAMN through R48. The SD update pulls
  it low through an explicit open-drain pad when it's done, and otherwise it's
  released. Don't make it an output assigned `1'bz`: nextpnr turns that into a
  driven pin, and low there reboots the part over and over. It must never be
  driven high either.
- **The flash** (pins 46, 47 and 49, and the configuration clock through
  `USRMCLK`) is ours once the part is configured. The SD update uses it.

It builds next to the ULX3S flavor under its own name:

    make r2             both bitstreams
    make r2-prog        load SRAM over JTAG, volatile
    make r2-reload      the same with no rebuild
    make r2-flash-dual  write both flash images and the jump record over JTAG
    make r2-flash       rewrite just the primary image over JTAG
    make r2-update      build R2UPDATE.BIN, for an update from the SD card

JTAG is an FT4232H module on J4, channel A, under the module's UART
labels: TXD to TCK (J4 pin 4), RXD to TDI (pin 3), RTS to TDO (pin 2),
CTS to TMS (pin 5), GND to pin 6, nothing on pin 1. `R2CABLE=` overrides
the cable name. Power the board from USB-C, not from J4.

After any rebuild, read the catalog back (the dumper's `K`, or a window
read of `$FFFF80`) rather than trusting the boot alone. A build can meet
timing and pass every simulation and still serve a wrong catalog (see the
fat32 note under "Known bring-up items").

The first r2 build lost the power-on race on a
cold start. The //e's card ROM looks for the drive about 0.6 s after
autostart reaches the slot, and if /PBSY or PCHK is wrong it skips the slot
and never asks again. Two things changed, both only for r2:

- The FPGA reads its flash at 62 MHz (`FLASHFREQ`, ecppack's top rate)
  instead of 38.8. The 348 KB compressed load drops from about 72 ms to
  about 45. The W25Q128 is good to 133 MHz.
- The SD card initializes at 397 kHz (`SD_DIV_INIT` 62) instead of 100 kHz.
  That's just under the 400 kHz the SD spec allows for initialization, and
  it's fine now that the socket is on the board rather than on flying leads.
  The first read also happens at this rate, so it's faster too.

Reads after that stay at 8.33 MHz, which is the last rate measured to work on
the old harness. r2 hasn't been swept yet (`SWEEP_ENABLE`), so it may well go
faster.

The FAT32 directory recursion is still on. It costs boot time when the card
holds a lot of files, which is why the images belong in `HD`.

With both changes, build 10231827 (60.17 MHz against the 50 MHz target)
serves the //e from a cold start. I couldn't measure how much margin is
left. The //e's monitor takes longer to come up than the drive does, so
there's nothing to time on the screen.

## Updating from the SD card

r2 can rewrite its own flash from a file on the SD card and reboot into it.
`src/fw_update.v` does the work, through `src/spi_flash.v`. It's only built
into r2 (`UPDATE_ENABLE`), since the ULX3S has no line from the FPGA to
PROGRAMN.

### The flash layout

    0x000000   the primary image: 256 bytes of $FF, then the bitstream
    0x100000   the golden image, laid out the same way
    0xFFFF00   a jump record pointing at the golden image

The updater only ever writes below 0x100000. If the primary fails to
configure, because an update lost power partway through for instance, the
ECP5 follows the jump record to the golden copy at power-up (Lattice
FPGA-TN-02203). The golden copy carries the updater too, so it finds the file
again and finishes the job. `make r2-flash-dual` writes all three pieces over
JTAG, which a board needs once.

The layout is ecpmulti's, but we don't use ecpmulti. It re-serializes every
bitstream and drops `--compress`, `--freq` and `--spimode`, and r2 needs all
three to win the power-on race. `ecppack --bootaddr` sets multiboot instead,
and `tools/mkupdate.js` writes the jump record. Both images point their next
boot at 0. ecpmulti points the primary at the golden copy instead, and then
the reboot after an update would land on the golden copy, which would write
the same file again and reboot again, forever.

### The file

`make r2-update` builds `R2UPDATE.BIN`: a 512-byte header, then the primary
image padded to whole sectors. The header holds `PEFW`, the board (`R2`), the
payload's length, its CRC-32 and its `BUILD_ID`. Put it in the root of the
card.

### Which build is where

There's no separate version number. The `BUILD_ID` does the job: it's the
minutes since 2026-01-01 00:00 UTC with the top bit set, so it only goes up,
and a higher one is always a newer build. The build puts it in the gateware
and in the bitstream's header as a comment, `PEV:<BUILD_ID>`, ahead of
nextpnr's own. The header comes right after a slot's 256 bytes of $FF, so the
comment always starts 258 bytes into the slot.

At reset, before anything else, the updater reads both slots' headers. That
takes about 30 microseconds and doesn't touch the SD card. Whichever slot
carries the running `BUILD_ID` is the one we were loaded from. If neither
does, the gateware came in over JTAG. A slot can also read `EMPTY` (erased),
or `UNKNOWN` (no build ID in its header). Headers written by builds from
before this change say `PEV:<BUILD_ID>:1.0`, and the updater reads the build
ID out of those too. After an update writes the primary, it reads the
primary's header again, so the menu shows what's waiting for the reboot.

The select menu shows all of it, from the info block (`selector/SELECTOR.md`
has the layout):

    BUILD 80068B67 (PRIMARY)            row 2: what's running, and its slot
    PRIMARY 80068B67  GOLDEN 80068B38   bottom row: what each slot holds

The ULX3S shows only the first line, without a slot. The Teensy shows neither.

Whether the primary matches is all the slot test checks. If the primary's
header survives but the rest of it is damaged, the ECP5 loads the golden
copy instead, and if the two copies are the same build, the menu still says
`PRIMARY`. In that case it's the same gateware either way.

### What happens at power-up

Boot time comes first, so the updater does nothing until the drive is idle:
a full second with no host command, no block reads or writes, and the SD
card at rest. A booting //e reads with gaps of a few milliseconds, so the
updater never touches the card during a boot. Then it looks for
`R2UPDATE.BIN` in the root, which takes a few sector reads. It stops there
if:

- there's no file
- the file's `BUILD_ID` isn't newer than the one that's running. That's
  what keeps an update from repeating, so the file can stay on the card, and
  it means an older file is ignored. Going back to an older build takes
  JTAG (`make r2-flash-dual`).

Otherwise it makes two passes, in the background. The drive keeps serving
the whole time: the updater is the SD card's lowest-priority user, so a host
read waits at most for the one updater read already in flight (well under a
millisecond). The first pass reads the whole file and checks its CRC, and
writes nothing. The second erases each 64 KB block as it reaches it,
programs each sector, and reads it back. An update of the current 371 KB
image reads the card twice and writes about 6 blocks, which takes a few
seconds.

Then it waits for the drive to be idle again, and to stay idle for four
more seconds (five in all), before it pulls PROGRAMN. That reboot is the
only part a host can notice: the drive is gone while the FPGA configures and
mounts the card again, the same wait as at power-up. If the //e is in the middle of something
then, it gets an error. If the host never goes quiet for five seconds, the
board keeps running the old image, and the new one starts at the next
power-up.

The LEDs show it. During the first pass they fill from D1 to D8, blinking.
During the second they fill again, solid. All eight stay lit while it waits
to reboot. A failure blinks D8 and shows the error code on D1 to D4 (D1 is bit
0), until a host appears:

| code | meaning |
|-----:|---------|
| 1 | an SD read failed |
| 2 | the header isn't `PEFW`, or its length is bad |
| 3 | the file is for another board |
| 4 | the payload doesn't match its CRC |
| 5 | the file ends before its payload does |
| 6 | the flash didn't answer as a W25Q128 |
| 7 | an erase timed out |
| 8 | a page program timed out |
| 9 | a sector read back differently |

Errors 2 to 6 happen before anything is written, and so can error 1. Errors 7 to 9, and an error 1 during
the second pass, leave the primary half written. The running image is
fine until power is removed, and at the next power-up the board falls back
to the golden copy, which tries again.

The serial report carries the same status in the `FAT` line, words 4 to 6,
when the self-test's FAT scan isn't built in: `[25:24]` the pass (1 check, 2
write, 3 done), `[19:16]` the error, `[15:0]` the sectors done; then the
sectors in all; then the file's `BUILD_ID`.

`make sim-fw-update` runs it against a hand-built FAT32 card and a model
flash. A good file is written and read back, with its directory entry and
its clusters both scattered so the FAT has to be followed. A file that's
already running, corrupted, built for another board, missing, or that reads
back wrong is refused. Nothing at or above the golden image is ever written.
It doesn't start until the drive is idle, and host activity holds the reboot
off until the drive has been idle long enough.

**None of this has run on a board yet.** The simulation covers the updater's
own logic. It can't show that the ECP5 really falls back to the golden copy,
that `USRMCLK` drives the flash clock as expected, or that the reboot works.
Before trusting it, flash a board with `make r2-flash-dual`, then try an
update. Then erase the first block of the flash over JTAG and power-cycle the
board, to check it comes up from the golden copy.

One case can still loop. If a file passes every check but its image won't
configure, the board falls back to the golden copy, which writes the same
file again, and reboots into it. Each round takes a few seconds, and the
LEDs show it happening. Taking the card out stops it.

## Electrical: the bus is 5V, the FPGA is not

Two 74LVC245s serve as voltage level converters, since the FPGAs are
not 5v tolerant and the ProFile bus is 5V TTL.

## Architecture

    db25 <-> 74LVC245 pair <-> profile_bus (tri-state, parity)
                                    |
                              profile_target      the three-phase protocol FSM
                                    |
                        +-----------+-----------+
                        |                       |
                  block_router            status/geometry
                  (normal blocks,        (spare table for $FFFFFF,
                   $FFFFFE RAM buf,       4 status bytes, the
                   selector magic)        multiple-of-4 rule)
                        |
                +-------+-------+
                |               |
             fat32           selector      the image menu's drive half
        (MBR, BPB, the      ($FFFF00..
         directory walk,     $FFFFFD)
         cluster chains,
         profile.cfg)
                |
             sd_spi  ->  SD card  (SPI mode, CMD17/CMD24)

`reporter` streams the whole diagnostic state out the serial port (the
ULX3S's USB serial, or J5 on r2); it is not in the data path.

Module by module:

- **`profile_bus`** owns the pins: tri-state control, the `DIR`//`OE` pair for
  the translators, and odd parity. Parity is generated for **host-driven bytes
  too**, including the ack (`PROTOCOL.md` §9.4): the drive owns that line in
  both directions. Getting it wrong is what makes a parity latch appear to set
  itself.
- **`profile_target`** implements command / confirmation / data phases, the six
  command bytes, the `$55` ack and the `$AA` NAK, and the //e's two departures
  from the spec: a read is 4 status + 512 with the host then abandoning by
  re-asserting `/PCMD`, and a write is 512 + a 6-byte trailer
  (`PROTOCOL.md` §3 and §9.1, §9.3).
- **`block_router`** decodes the 24-bit block number: normal blocks to the SD
  reader, `$FFFFFE` to the RAM buffer, `$FFFFFF` to the synthesized spare table,
  and the `$FFFF00-$FFFFFD` selector window.
- **`sd_spi`** is a plain SPI block reader: init at 100 kHz (397 kHz on r2),
  run at the measured rates in the table below, CMD17 single-block read and CMD24 write. **Both
  card generations work.** SDHC addresses by block; a card that rejects CMD8 is
  treated as v1, ACMD41 goes out without HCS, CCS comes from the OCR, and CMD17
  arguments are scaled by 512. `make sim-v1` and `make sim-v1skew` are that
  path. It also recovers byte alignment when a card's replies arrive out of
  phase, which the real one does: `make sim-sweep` checks the recovery over all
  eight bit phases.

## Geometry, and what reverse engineering revealed

The claimed volume size in the spare table **must be a multiple of 4**
or the //e's ROM never hands off to the boot block (`PROTOCOL.md`
§9.2). The drive rounds down and caps the claim at `$FFFC`, which is
what the Teensy firmware does, and it will not be configured
otherwise. It is one comparator and it prevents an entire class of "it
reads fine but will not boot". Also see "Geometry: two block counts,
not one" below.

## Testing against the //e, not just the reader

`make sim` runs the protocol against a model of the READER's host
sequence, which is deliberately forgiving: microseconds per byte, a
settling delay after `PR/-W` rises, generous strobes. A drive that
fails there cannot possibly work. Passing it shows very little, and
PROTOCOL.md section 3 says so: the Apple /// and Lisa hosts insert
a delay before their first strobe, "which is why a slow drive works
with the reader but the same drive can still fail against the //e".

`make sim-iie` is the strict one. It models three things the reader model does
not:

1. **No settling delay.** The card raises `PR/-W` and clocks the first byte
   immediately, so byte 0 must already be on the bus. A drive that turns the bus
   around after seeing `PR/-W` loses it and the boot fails looking like bad data.
2. **A 355 ns strobe**, against 16 us in the reader model.
3. **The abandon.** 4 status + 512 data and then the card re-asserts `/PCMD`; it
   never takes the 20 tag bytes.

It also drives a write with the 6-byte trailer from section 9.3 and checks
`/PPARITY` on every strobed byte in both directions, which is the section 9.4
requirement that parity covers host-driven bytes too.

`make sim-iie-fast` reruns it with the pacing squeezed to a quarter of the //e's
as a margin measurement, and `make sim-iie-32mb` reruns it on a 65535-block
image so the decoupled geometry below is exercised through the host's own path.

## Non-data behaviors

**Status bytes.** Only two bits are the drive's to set, and both are in byte 2.
`$80` says the drive was reset since the last command, and is reported by the
FIRST command after power-up and then not again. `$40` says the operation
failed: a block number outside what we serve, a refused write, or an SD error
underneath.

**Block `$FFFFFE`, the drive's own RAM buffer.** Readable and writable, and
backed by its own storage rather than sharing the block buffer, so an ordinary
block read between a write and a read back does not disturb it.

**No media.** With no usable card the drive serves a self-verifying pattern
(`$A5`, the three block-number bytes, then a running sum) instead of whatever
the block buffer happens to hold, the same as the Teensy build with no image
open. This is what makes DB-25 bring-up possible with an empty SD socket.

**Write protect.** `WRITE_PROTECT` refuses every write. The socket's own
write-protect contact is NOT honored by default. The constraints file records
that these are bare switch contacts with no standardized polarity, and neither
has been metered on this board. Trusting one would either protect nothing or
refuse everything. `HONOR_WP` enables it once the contact has been metered.

**NAK aborts.** The //e's ROM sends `$AA` as a deliberate NAK and follows it
with a full retry from phase 1. A drive that pushes on into the data phase is
reading the retry's command-frame strobes as data-phase bytes and never
resynchronizes. So any ack that is not `$55` returns the drive to idle.

**Unknown opcodes are refused before the confirmation.** `$03` in particular is
Force Sparing: undocumented, destructive, and not a dry run. Answering it and
then failing it in status would still have entered a write data phase.

## Whole-volume verification

"A sector was read and it had $55 $AA in it" is not the same claim as "every
block this drive will serve matches the image". `src/verifier.v` and
`tools/verify_image.js` make the second claim.

The FPGA reads a range of the card and streams one line per checkpoint: six hex
digits of starting LBA, a colon, four hex digits of CRC16-CCITT over the bytes
in that checkpoint. The script computes the same numbers from an image file and
reports the first LBA that differs.

    make verify-prog VCOUNT=65536 VSTRIDE=1
    tools/verify_image.js ~/Desktop/card.img --port /dev/cu.usbserial-XXXXXXXX \
      --count 65536 --stride 1

The range is not negotiated over the wire, so the `VCOUNT`/`VSTRIDE` the
bitstream was built with and the `--count`/`--stride` the script is told have to
agree. `--input` re-checks a capture taken earlier with no board attached.

`VSTRIDE` is what makes a big card checkable. One line per sector is right for
the 65535 blocks a host can actually reach: about 790 KB of serial, a minute or
so. A 4 GB card is 8.4 million sectors. At one line each, the UART becomes the
limit, not the SD card. Fold many sectors into one CRC to sweep coarsely, and
narrow afterward.

**The verifier is deliberately a separate bitstream** (`make verify-bit`, not
`make`). The verifier holds the SD engine for the whole run, which is not
something a build meant to serve a host should ever do. It is compiled out of every other build.

The CRC is a contract between the bit-serial implementation in Verilog and the
table-driven one in JavaScript, so it is tested on both sides rather than only end
to end. `make sim-verify` decodes the verifier's own serial output a bit at a
time, recomputes every checksum independently, and prints fixed vectors that
`tools/verify_image.js --selftest` asserts against. If either implementation
drifts, one of the two suites fails immediately instead of the volume
comparison blaming sector 0.

## The filesystem layer

`src/fat32.v` is what lets the card be an ordinary FAT32 volume with image files
on it, the way the Teensy build works, instead of being dedicated to one raw
image. It reads the MBR, reads the BPB, finds a directory in the root
(`FS_DIR`, default `HD`), looks for `.IMG` and `.HDV` files in it and below it,
picks the one the configuration names, and walks that file's cluster chain.
`block_router` then maps a ProFile block through the chain. `image_blocks` comes
from the directory entry rather than a parameter, so the geometry rules apply to
the file's real size with nothing else changed.

**The chain is stored, not assumed contiguous.** Just copying images
to a blank SD card still creates fragmented files, so it's important
for this to follow actual chains.

The chain table is `FS_MAX_CLUSTERS` entries, 1024 by default, which is a full
32 MB volume at the 32 KB clusters this card uses.

### Choosing an image (at boot time)

I've talked about `profile.cfg` in the root already, and the directory
scan process based on `FS_DIR` (which default to `HD`, with
`FS_RECURSE` on). It will recurse up to three levels of subdirectories
looking for images. If you want a scan of the whole volume you would
set `FS_DIR` to all spaces. If you want even faster boots, you'd turn
off `FS_RECURSE`.

**Matching is done on long filenames** with full paths. **A pure 8.3
name is matched too.** An entry with no LFN chain in front of it, such
as `A2OSX.IMG`, is compacted as it streams: the on-disk `A2OSX   IMG`
becomes `A2OSX.IMG` in the name buffer, with the padding dropped and
the dot inserted at the right place. Everything downstream reads that
buffer, so the catalog, the configured-name comparison and the
persisted path all see the real filename whichever form it came from.
The comparison is case-folded on both sides.

`make sim-fat32-83` is that case: an image with no long name at all,
asked for in lower case, with autopick turned off so that resolving at
all is the name match and not a fallback.

**Which image wins: NONE, unless something chose it.** `../selector/SELECTOR.md`
is explicit that the drive "will not invent a default and boot it
without showing the menu", so that a stale or missing configuration can
never cause the wrong volume to be served. With images present but no
selection, `fat32.v` reports `need_select` and `E_NOSEL` and the drive
serves no media. With no `profile.cfg` at all it does the same, but reports
`E_NOCFG`. `make sim-fat32-noauto` tests that case.

`AUTOPICK` will automatically select the largest image it finds and is
used for bench testing: `make sim-fat32` and `tb_top_sd` use this,
but not any of the bitstreams for the device.

**Still missing:**

- **ProDOS-volume validation.** The Teensy reads block 2 of each image and
  skips anything that is not a ProDOS volume; the catalog here lists any file
  with an image extension, so a raw dump named `.img` appears in the menu.
- **`.tag` sidecars.** They're not used on the Apple //e and I haven't validated any of it on an Apple ///.

## The boot menu and selector

The boot menu obviously lets you select which image you want to boot
in to, but that behavior also exists from within the host OS. The
ProFile card doesn't have a lot of leeway for what we're doing, but
there is some; the selector (`src/selector.v` is the drive half of the
menu, while the 6502 parts live in `../selector`) can be invoked from
the host too.

In the boot process, the card ROM boots by reading block 0 into
`$0800` and jumping to `$0801`. This project recognizes that behavior
and substitutes a synthetic loader instead of the image's own boot
block. This is the sort of behavior that a ROM would normally handle
directly, if it could. I'm deliberately lying to the controller in
order to have this ProFile emulator supply a boot menu of available
images. It's one of a small number of hacks I've added to make the
ProFile card do more than it was designed to do.

These magic block numbers are used by the ProFile emulator to allow
signaling between the running machine and the emulator:

| block | meaning |
|---|---|
| `$FFFF00 + i` | SELECT image i. Returns `$00` accepted, `$FF` out of range |
| `$FFFF40 + p` | selector body, page p |
| `$FFFF80 + p` | catalog page p, eight 64-byte records |
| `$FFFFC0` | info block: counts, `$A5` signature, force-menu bit |

The synthetic loader menu appears in exactly three cases:

- **open-apple is held at boot**, which is the deliberate way in;
- **the drive sets the force bit**, which it does when there is no valid
  persisted selection (`need_select`: no `profile.cfg`, or a configured path
  that resolves to nothing);
- **a running program knocks**, by reading blocks `$FFFF41 $FFFF32 $FFFF45
  $FFFF44` back to back through the ordinary card ROM driver. That opens the
  menu from inside a running system. A selection made this way is ephemeral: it
  is served and booted but does not update `profile.cfg`.

**Block 0 is intercepted only during a BOOT.** The ProFile ROM
normally looks at the spare table (`$FFFFFF`) just before it reads
block 0 to boot; that's the trigger for the synthetic loader to be
substituted for the actual drive contents. Anything that tries to
read block 0 any other way gets the actual block 0 from the image
being served.

`make sim-selector` checks the blocks against independently read
copies of the blobs: that page 1 is the next 512 bytes rather than
page 0 again, that a page past the blob reads as zero, and that the
window's edges are right (`$FFFFFD` ours, `$FFFFFE` and `$FFFFFF` not,
`$00FF40` an ordinary data block).

### The catalog

`fat32.v` fills it while it scans: one 64-byte record per image,
holding the flags, 48 characters of name and the block count. Names
are captured through the long-name entries where there are any, and
folded to upper case.

## The serial report

Three lines, repeating every 1.5 s, then a hex dump of the block buffer:

    SD  wwwwwwww x8
    FAT wwwwwwww x8
    TXN wwwwwwww x8

Each line is a 256-bit bundle clocked out as eight 32-bit words. Their
bit-meanings are below.

### `SD`, the card and the SPI engine

| word | contents |
|---|---|
| 0 | `[24]` ready, `[16]` ok, `[11:8]` error code, `[4:0]` sd_spi state |
| 1 | `[29:24]` last command, `[23:16]` retries used, `[7:0]` self-test stage |
| 2 | **`BUILD_ID`**, minutes since 2026-01-01 00:00 UTC at synthesis, with the top bit set (`date -u -r $(( (0x<id> & 0x7FFFFFFF) * 60 + 1767225600 ))` turns it back into a date) |
| 3, 4 | the first eight bytes of `profile.cfg` exactly as read |
| 5 | boot chronometer: `[27:16]` ms to SD ready, `[11:0]` ms to resolver ready. `$FFF` means never |
| 6 | clusters in the resolved chain |
| 7 | `[31:24]` have-config and parsed path length, `[23:0]` chain entry 1 |

Word 2 identifies the running bitstream, so a test round can start by confirming
which build is actually on the board. Word 5's chronometer answers whether the
drive is up before the //e's ROM starts asking. It runs from reset release,
which on a flash boot is configuration done.

### `FAT`, the self-test's scan

Only populated when `SELFTEST_FAT` is built in; zero otherwise. Partition LBA,
FAT start, data start, FAT size, free clusters, first free cluster, target LBA,
bytes compared.

### `TXN`, the ProFile side and the filesystem

| word | contents |
|---|---|
| 0 | `[31:28]` strobe count, `[27:25]` command-byte index, `[23:0]` last block the host asked for |
| 1 | last opcode, then commands, completed reads, completed writes |
| 2 | NAKs, abandons, bad opcodes, out-of-range requests |
| 3 | claimed blocks, served blocks |
| 4 | `[25]` selector window live, `[24]` selector armed, `[23:0]` **the last block actually refused** |
| 5 | command byte 0 as seen at the strobe's LEADING edge, then `/PRES` level, geometry decoupled, status byte 2, and the no-media / PR-W / write-protect / busy flags |
| 6 | `[31]` fs ready, `[30]` fs ok, `[29]` **needs a selection**, `[28]` selector wanted, `[27:24]` fs error, `[23:16]` image count, `[15:0]` bad frame bytes |
| 7 | edge counts on the three host-driven lines, then their live levels |

`fs` error codes: 1 MBR, 2 BPB, 3 no directory, 4 no image, 5 chain broken,
6 SD, 7 chain too long, 8 nothing selected, 9 **no configuration**, 10
configured name not found.

Word 4's refused-block field is worth knowing about when the out-of-range count
saturates: the count alone cannot say which block was refused. Word 5 carries
command byte 0 as sampled at the strobe's leading edge, which is what tells you
whether a host is driving the bus early, late, or not at all for that byte.

## Timeouts are counted in real time, not byte times

Every timeout in the SD path is expressed in clock cycles, not in byte
times. A timeout counted in byte times changes its real duration with
the SD clock. The faster the link runs, the less patience the drive
has. That is backward, and it produces failures that are rare,
clustered, and invisible at slow clocks.

The most important timeout is the data-token timeout. The SD
specification allows a card up to 100 ms of read access time (Nac),
and `TOKEN_TIMEOUT_CYC` is 100 ms of clock cycles so that the card
gets what it is entitled to take. **If a timeout here is ever
expressed in bytes, it is a bug.**

## Retries

`RETRY_MAX` (default 3) re-issues a failed transfer before reporting
an error. A real drive retries, and the ProFile command frame carries
a retry count of its own. The retry deselects the card and leaves it
alone for a millisecond first, since whatever made the attempt fail
needs time to pass.

`make sim-retry` checks that a card that rejects two reads is absorbed
with no error reaching the host, and costs exactly two retries. `make sim-retry-exhaust`
checks that one that rejects five gives up cleanly after three. An
unbounded retry loop turns a card that has stopped answering into a
hang, which is worse than an error.

The verifier closes each run with a `#NN` line giving the retry count,
because a retry that worked leaves no trace in the per-sector output,
and "it passed" and "it passed first time" are different claims about
the hardware.

## Geometry: two block counts, not one

`src/geometry.v` owns the rule and `make sim-geom` tests it, including the four
measured geometries from PROTOCOL.md section 9.2.

The count we CLAIM in the spare table and the count we SERVE are
separate numbers. The claim must be a multiple of 4 or the //e's ROM
never hands off to the boot block, and it must fit the 16 bits the
interface card keeps, so it tops out at `$FFFC`. For any image of
65532 blocks or fewer the two are equal and none of this section matters.

They diverge for a full 32MB ProDOS volume: 65535 blocks is three too
many AND misaligned, so it can never be claimed.

Decoupling these two serves all 65535 blocks while continuing to claim
`$FFFC` existing blocks. For ProDOS that works because it takes the
volume's real size from the image, not from the drive or driver.

## Known bring-up items

- **Tri-state.** The eight data-bus bits go through an explicitly instantiated
  pad primitive, selected by the `FAMILY` parameter: `TRELLIS_IO` on the ECP5,
  `SB_IO` on the iCE40. The pad is read through the primitive's own input rather
  than by reading the `inout`, so the output enable is something the design
  states rather than something the tools infer. The failure mode if it were
  inferred wrongly is two drivers fighting on a 5V net with the //e's interface
  card on the far end. `make sim-iie` checks at the pad level that the FPGA's
  drivers let go on every host-driven byte.
- **Clock domains.** One domain. The ULX3S's 25 MHz oscillator goes through an
  `EHXPLLL` to 50 MHz and everything runs from that; reset is held until the PLL
  locks. The SD clock is a divider off it, not a second clock domain: 100 kHz
  for init (397 kHz on r2), then the measured rates in the table above.
  `CLK_HZ` is a parameter, and every real-time constant in the design is
  derived from it. Changing the system clock keeps every timeout at the same
  real duration.
- **`/PRES` is IGNORED.** `HONOR_PRES` is 0 in `top.v`. The //e's reset line is
  documented as idling asserted on some setups, and honoring it there pins the
  whole design in reset, which looks exactly like a board that does nothing.
  If the //e tries to reset the drive, we will not notice. In practice this never happens.
- **The fat32 scanner is the critical path.** On an earlier TQFP144 build
  it was 19.5 ns of a 20 ns period and the catalog was wrong on hardware
  while every bench passed. `lfn_base` in `fat32.v` is the fix. The margin is
  now about 17%. If a rebuild ever shows every catalog record with the
  same block count, suspect timing before logic.
- **The SELECT reply byte is not to be trusted.** `selector.v` answers `$00`
  for an index below `img_count` and `$FF` otherwise. On hardware (build
  10081647), accepted selects of indices 0 and 7 read back `$FF`, and the
  selects took effect and persisted anyway. Likely the rescan a select
  starts zeroes `img_count` while the reply is still being produced. This
  has not been chased down. Confirm a select from the spare table's block
  count instead.

## Status

Measured 2026-09-02, the full design with recursion on:

    TRELLIS_COMB   9730 / 24288   40%      (LUT4s and carry, after packing)
    TRELLIS_FF     3830 / 24288   15%
    DP16KD           43 /    56   76%      block RAM
    TRELLIS_IO       38 /   197   19%
    EHXPLLL           1 /     2   50%
    MULT18X18D        1 /    28    3%
    Max frequency 52.60 MHz  (PASS at 50 MHz)

**Block RAM is now the tight resource, not logic.** 76% of the DP16KDs
against 40% of the fabric. The cluster table, the block buffer, the
catalog and the two 6502 blobs are what fill it. If more is needed,
halving the catalog to 32 images is the fastest tradeoff.

**The timing margin is thin: about 5%.** Note also that the placer
reports a lower figure than the router's final one (47.79 against
52.60 MHz in this build), so a build judged on the intermediate number
looks like a failure when it is not. Do not read a seed sweep off the
placement figure.

On r2 (TQFP144) the same sources route at 54 to 60 MHz, depending on the
run: 60.17 MHz for build 10231827, and 54.89 MHz on 2026-10-24 after the
`block_router` fix. nextpnr's result moves from build to build. Before the
`lfn_base` change, an earlier TQFP144 build routed at 51.29 MHz.

**A 12F is enough.** A ULX3S sold as a 12F carries 25F silicon and
must be targeted as `--25k`, which is what the numbers above are
against. The part is half empty with every feature turned on.

### The simulation suite

`make sim-all` runs twenty-one benches. The rest are run by name when you want
them.

**The ProFile protocol, against a model host**

| bench | what it covers |
|---|---|
| `sim` | the whole transaction against the READER's host sequence: the forgiving one, microseconds per byte |
| `sim-iie` | the same against the //e: 355 ns strobes, no settling delay after `PR/-W` rises, the abandon after 512 bytes, the 6-byte write trailer, and `/PPARITY` on every strobed byte in both directions |
| `sim-iie-32mb` | a 65535-block image through the host's own path, so the decoupled geometry is exercised where it actually matters |
| `sim-iie-fast` | the //e bench with the byte pacing squeezed to a quarter. A margin measurement, not a requirement, so it is not in `sim-all` |
| `sim-geom` | the claimed and served block counts, the multiple-of-4 boot rule, and the four measured geometries from `PROTOCOL.md` §9.2 |
| `sim-exercise` | the no-host bus pattern, and that it yields to a host instantly |
| `sim-usb-prov` | the provisioning channel, and the new-selection zero-serve race |
| `sim-router-race` | that a read parked behind a busy resolver cannot inherit the previous transaction's `blk_ready` |

**The SD layer**

| bench | what it covers |
|---|---|
| `sim-sd` | init through a CMD17 read against a model card, no host at all |
| `sim-unit` | `sd_spi` on its own |
| `sim-skew` | a card whose replies are two bits out of byte alignment, which the real one does |
| `sim-sweep` | the alignment recovery at all eight bit phases, so it is phase-independent rather than a workaround |
| `sim-v1` | a card that rejects CMD8 and addresses by byte: ACMD41 without HCS, CCS from the OCR, CMD17 arguments scaled by 512 |
| `sim-v1skew` | the real card's combination, v1 behavior AND a skewed reply |
| `sim-retry` | a card that rejects two reads: absorbed with no error reaching the host, at a cost of exactly two retries |
| `sim-retry-exhaust` | a card that rejects five: gives up cleanly after three, because an unbounded retry turns a card that has stopped answering into a hang |

**The filesystem**

| bench | what it covers |
|---|---|
| `sim-fat32` | finding the image and resolving a chain the model deliberately fragments (4, 6, 5, 7). A resolver that assumed contiguity fails on block 1 |
| `sim-fat32-flat` | the same card and config with `FS_RECURSE=0` |
| `sim-fat32-83` | an image with no long name at all, asked for in lower case, with autopick OFF so that resolving at all is the name match |
| `sim-fat32-noauto` | images present and no `profile.cfg`: serve nothing and report `E_NOCFG`, rather than inventing a default |
| `sim-fat32-persist` | the write-back. Needs a selection that really changes, and has to wait: `ready` goes high before the persist runs |
| `sim-fat32-lru` | the LRU-2 chain cache: an ephemeral reselect between two images costs zero SD reads and serves each image's own chain |

**The menu and the verifier**

| bench | what it covers |
|---|---|
| `sim-selector` | the magic blocks: that page 1 is the next 512 bytes rather than page 0 again, that a page past the blob reads as zero, and the window's edges (`$FFFFFD` ours, `$FFFFFE` and `$FFFFFF` not, `$00FF40` an ordinary data block) |
| `sim-verify` | the verifier's serial output decoded a bit at a time, every checksum recomputed independently, and `tools/verify_image.js --selftest` asserted against the vectors it prints |
| `sim-verify-stride` | the same folding four sectors into each checksum, which is what makes a big card checkable |

Two things to know before you run them:

- **A failing bench fails make.** Each bench prints `PASS` or `FAIL`, and the
  Makefile checks for those lines, because `vvp` exits 0 either way. So
  `make sim-all` stops at the first failure; `make -k sim-all` runs the rest.
- **`sim-iie-late` is not in `sim-all` and is expected to fail.** It reproduces
  a known defect rather than guarding against a regression, and it does not
  terminate on its own; interrupt it.

### Other miscellania

**The design runs at 50 MHz.** Nothing in it needs more: the //e's shortest
strobe window is 355 ns, which is 17 clocks at 50 MHz, and a host-paced byte is
some 640 clocks. The ULX3S's oscillator is 25 MHz, so the PLL multiplies up to
reach it.

`src/sd_spi.v` implements initialization, CMD17 single-block reads and CMD24
single-block writes, all verified against a real card on the bench.

**Measured SD clock ceilings, on a hand-soldered harness.** Reads and writes
do not run at the same rate and the difference is not subtle:

| | 99.6 kHz | 391 kHz | 758 kHz | 1.47 MHz | 2.78 MHz | 5.0 MHz | 8.33 MHz | 12.5 MHz |
|---|---|---|---|---|---|---|---|---|
| read  | pass | pass | pass | pass | pass | pass | pass | fail |
| write | pass | pass | pass | fail | fail | fail | fail | fail |

Reads are verified by checksum against a known sector at every rate, writes by
writing a known-free block, reading it back and comparing all 512 bytes. The
asymmetry is the diagnosis. A read puts six command bytes on MOSI and a write
puts 512 back to back, so a write is the only thing that really exercises MOSI
integrity. This harness is flying leads through series resistors. Treat it
as a wiring limit rather than a protocol one. The settings are
`SD_DIV_FAST` and `SD_DIV_WRITE` in `src/top.v`; re-measure with `SWEEP_ENABLE`
before changing either.

This should go away with the manufactured PCBs. As of this writing r2 hasn't
been swept, so it still uses the harness's numbers.

### The first bench test: the SD card, with no host at all

With nothing plugged into the DB-25, nothing will drive the ProFile bus. The
bitstream is built for that case: **as soon as the card reports
ready it reads sector 0 by itself** and checks for the `$55 $AA` signature at
offsets 510 and 511, which any formatted card carries. The whole result is
readable off the LEDs, so the test needs the ULX3S, power, and an SD card wired
to the four SPI pins, and nothing else.

`make sim-sd` runs exactly this against a model card and expects LEDs = `$E0`.

While no host has ever asserted /PCMD, the LEDs show the **SD page**:

| led | shows |
|----:|-------|
| 7 | card initialized |
| 6 | self-test finished |
| 5 | before it finishes, a ~1.5 Hz heartbeat; after, **1 = sector 0 verified** |
| 4:0 | while initializing, the SD state number; once ready, the error code |

So a good run ends at **`$E0`**: top three lit, bottom five dark. Anything else,
read the bottom five:

| code | meaning |
|-----:|---------|
| 1 | `E_CMD0`, no response at all. Wiring, card not seated, or no power |
| 2 | `E_CMD8`, the v2 check pattern came back wrong |
| 3 | `E_ACMD41`, the card never left idle |
| 4 | `E_CMD58`, OCR unreadable |
| 5 | `E_NOTSDHC`, retired: byte-addressed cards are supported now |
| 6 | `E_CMD17`, the read was rejected |
| 7 | `E_TOKEN`, no `$FE` data token |
| 8 | `E_NOTREADY`, a transfer was asked for before the card was up |
| 9 | `E_WRITE`, CMD24 was rejected |
| 10 | `E_NOR1`, no R1 response byte could be recovered |
| 11 | `E_WDATA`, the card refused the write data |
| 12 | `E_WBUSY`, the card never released busy after a write |

If led[7] never lights and the bottom five sit at a fixed number, that number is
the state it stopped in. That is more useful than the error code: it says which
command never got an answer. led[5] blinking with nothing else lit means the part
is configured and running but the card is not answering.

The SD clock starts at 100 kHz (397 kHz on r2) to bring the card up and then
moves to the rates in the table further up: 8.33 MHz for reads, 391 kHz for
writes. Those were measured on the ULX3S harness, where the socket was
off-board on flying leads through 33R series resistors, which is the most
likely reason writes stop an order of magnitude below reads. r2 has its socket
on the board and still uses the same rates, because it hasn't been swept.

### The other bench test: the reader, not the //e

`make sim` already runs this transaction end to end. On hardware, drive it from
the reader's serial monitor and expect the get-status read of `$FFFFFF` to return
a spare table beginning `PROFILE      ` with device number `00 00 10`. The
simulation prints the same 16 bytes, so the two can be compared directly.

Once a host has asserted /PCMD for the first time the LEDs switch permanently to
the **bus page**, which shows live line state rather than protocol events,
because on a board that has never been powered the first questions are
electrical:

| led | shows |
|----:|-------|
| 0 | BSY asserted |
| 1 | we own the data bus (`xcvr_dir`) |
| 2 | /PCMD asserted by the host |
| 3 | **/PRES asserted by the host** |
| 4 | SD card detect, as wired |
| 5 | SD write protect, as wired |
| 6 | DB-25 1, /PH0 |
| 7 | DB-25 7, AUX7 (always lit on r2, which dropped AUX7 and pulls the pin up) |

led[3] is the one to look at if the board seems hung. `HONOR_PRES` in
`src/top.v` is 0, so /PRES does NOT reach the core reset today. This display is
what says whether turning it on would be safe on your //e: a host that idles the
line asserted would pin the target in reset. The display is deliberately not
gated by reset, so it still works when everything else has stopped. led[4] and
led[5] answer the other open question, which socket contacts are normally-open,
without a meter.
