# ProFile emulator

This is a replacement for an Apple ProFile hard drive. It plugs into the cable
from an Apple ][ ProFile interface card, answers the card the way a real
ProFile does, and serves blocks from disk images on a microSD card.

## How it got here

In 2021 I built a reader for my ProFile 10 around a Teensy 4.1, so I could get
the drive's contents into a disk image. That's
[profile-dumper](https://github.com/JorjBauer/profile-dumper), and the build
log is on [Hackaday](https://hackaday.io/project/179930-apple-profile-10mb-hard-drive-reader).

In 2026 I turned it around. The Teensy version in `teensy/` pretends to be the
drive instead of the host, and it works on my //e. But it's at the limit of
what the Teensy can do, and it took some clever optimization to get there.

So I moved it to an FPGA. I chose to make it FPGA-only, with no
microcontroller, to keep the cost down. That drove the complexity up. I started
on an Alchitry Cu (iCE40 HX), moved to the Alchitry Au (Artix-7) when the
iCE40's carry chains became the bottleneck, and then moved to a ULX3S-12F
(ECP5). If you find comments about the iCE40 or the Artix-7, that's why.

Then I built two boards of my own around the same ECP5 in a TQFP-144. r1
worked after some rework, but it had the wrong footprint for the flash chip,
along with other mistakes. r2 fixed them, and it's been serving my //e since
2026-10-23. r1 lives in the git history.

All of my testing is on a //e with a ProFile card whose ROM is 341-0299-B. I
haven't tried an Apple ///.

## What's here

- `fpga/`: the current design. Its `README.md` is the technical notebook:
  architecture, the serial report, the boot menu, and the tests.
  - `src/` and `sim/`: the Verilog and its testbenches.
  - `constraints/`: pinouts for the ULX3S (`ulx3s.lpf`) and r2
    (`profile-drive-r2.lpf`).
  - `tools/`: node scripts. One imports the selector's blobs into the build;
    the others check a card image against what the drive serves.
  - `carrier/ULX3S-ADAPTER.md`: wiring the interface board to a ULX3S.
  - `hw-r2/profile-drive/`: the r2 board. The KiCad project, the Gerber and
    stencil zips, a placement sheet and an assembly drawing.
  - `hw-r2/enclosure/`: a printed case shaped like a tiny ProFile.
- `teensy/`: the Teensy 4.1 version. It still works.
- `selector/`: a submodule with the 6502 boot menu that both versions serve,
  so you can pick an image from the //e.

The ProFile protocol, the DB-25 pinout and the interface card's behavior are
documented in profile-dumper's `PROTOCOL.md` and `HARDWARE.md`. The READMEs
here refer to them often.

## Getting the code

The selector is a submodule, so clone with it:

    git clone --recursive <this repo>

If you already cloned without it, run `git submodule update --init`.

## Where to start

### Preparing an SD card

Format it FAT32, put your images in a directory named `HD`, and create
`/profile.cfg` by hand with any non-empty content. The drive can rewrite
files, but it can't create them. The details are in `fpga/README.md` under
"Preparing the SD card".

### Building for a ULX3S

You need [oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build),
which has yosys, nextpnr-ecp5, ecppack, iverilog, openFPGALoader and fujprog.
You also need `node`, because the build uses it to import the selector's
blobs. Then, in `fpga/`:

    make sim-all    run the testbenches
    make            build the bitstream
    make prog       load it into SRAM, volatile
    make flash      write it to the SPI flash

`make sim-all` stops at the first bench that fails. `fpga/README.md` has the
full list of targets.

The ULX3S needs an interface board between it and the DB-25: two 74LVC245s
that shift the 5V bus down to 3.3V. Mine was hand-wired on perfboard, and its
design isn't in this repo. `fpga/carrier/ULX3S-ADAPTER.md` has the pin map
from that board to the ULX3S, and the wiring mistakes that damage parts.

### Ordering r2 boards

Everything is in `fpga/hw-r2/profile-drive/`.

- `profile-drive-r2-gerbers.zip` is the board: 4 layers, 1.6 mm, lead-free
  HASL, 1 oz outer copper, 100 x 55 mm.
- `profile-drive-r2-stencil-reduced.zip` is a top paste layer with the
  apertures on U1 and J1 shrunk by 0.04 mm. The full-size paste in the Gerber
  zip bridged their fine-pitch pins on my board. I ordered it in 4 mil
  stainless.
- Every part is on the top side.

If you want JLCPCB to assemble it, upload these two as well:

- `profile-drive-r2-bom-jlcpcb.csv`: the BOM, with the manufacturer part and
  the LCSC number for each line. Every LCSC number is the exact part in the
  schematic, except for the LEDs, which the schematic doesn't name. The BOM
  uses red ones (KENTO KT-0603R).
- `profile-drive-r2-cpl-jlcpcb.csv`: the pick-and-place file, in the same
  coordinates as the Gerbers. I haven't had JLCPCB assemble one, so check
  every part's rotation in their preview before you pay. The TQFP, the
  SOT-23s, the TSSOPs and the connectors are the usual suspects.

J4, J5, J6, J7 and SW1 are optional and left out of both files. They're for
programming, debugging and service, and you can solder them on later. A new
board's flash is blank, though, so something has to reach J4's pads once to
load it. I use a pogo-pin clip on J4. A 1x6 header soldered there works too.

As of 2026-10-24, three parts had no stock at JLCPCB: U1 (the FPGA), J1 (the
USB-C socket) and J2 (the DB-25). You'll probably have to send those in
yourself or solder them by hand. R31 (84.5k) had only 17 in stock.

The LCSC numbers are also in the schematic, as an `LCSC` field on each part,
and J4, J5, J6, J7 and SW1 are marked DNP in both the schematic and the board. If
you change the design, KiCad's own BOM and position exports pick both up, but
the two CSV files here won't follow on their own.

### The LEDs

The schematic gives each of the eight LEDs a 1k resistor (R35 to R42, in
order for D1 to D8) and doesn't name a color. That's sized for ordinary red or
yellow 0603 LEDs. The BOM uses red (KENTO KT-0603R, LCSC C2286). For yellow,
NationStar NCD0603Y2 (LCSC C89811) works the same way.

The LEDs and their resistors are optional. Nothing else is on those nets, so
leaving them off only loses the status display and the error codes.

If you use white, blue or true green LEDs, change their resistors to 330R.
Those colors need about 3 V to light, which leaves only a few tenths of a volt
across the resistor from a 3.3 V pin, so at 1k they're dim. On my board D1 to
D4 matter most, so D1 is white, D2 is blue, D3 and D4 are emerald, and R35 to
R38 are 330R.

### Building one by hand

`placement-r2.png` is the assembly sheet. It shows where each part
goes, marks pin 1 (and each LED's cathode) in red, and has a checklist BOM with
quantities and part numbers. `assembly-r2.pdf` puts the values on the
footprints. Use the reduced stencil.

I haven't published a bring-up checklist yet.

### Reflashing an r2 board over JTAG

In `fpga/`, `make r2` builds it, and `make r2-flash` writes it to the board's
flash (`make r2-prog` loads SRAM instead). The programmer is an FT4232H module
on J4, through a pogo-pin clip or a header soldered there. The wiring is in `fpga/README.md` under "Building for the r2 board".
Power the board from USB-C, not from J4.

There are no prebuilt bitstreams yet, so you need the same tools as the ULX3S
build.

### The Teensy version

`teensy/README.md` covers the wiring, the arduino-cli build and the serial
commands. The board was hand-wired, so its README is an as-built description,
not a schematic.

## What doesn't work yet

- The drive can't update its own firmware from the SD card. The r2 hardware
  can do it (the FPGA can drive its flash pins and pull its own PROGRAMN), but
  the gateware doesn't exist yet. For now it's JTAG.
- No prebuilt bitstreams.
- The ULX3S interface board's design isn't published.
- Nobody has tried it on an Apple ///.

## License

MIT. See `LICENSE`.
