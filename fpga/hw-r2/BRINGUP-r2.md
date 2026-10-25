# Building and bringing up an r2 board

This is how to go from a bare r2 board and a bag of parts to a drive that
serves your //e, and the problems to look out for along the way. All parts
are on the top side of the PCB.

## What you need

- the board, from `profile-drive/profile-drive-r2-gerbers.zip`
- the reduced stencil (unless you are feeling adventurous), `profile-drive/profile-drive-r2-stencil-reduced.zip`
- the parts, from `profile-drive/placement-r2.png` or the JLCPCB BOM
- Type 4 solder paste or finer (see "Paste and reflow")
- an FT4232H module for JTAG, and a pogo-pin clip for J4
- a multimeter, and a USB power meter if you have one

The reduced stencil has the pads for the FPGA reduced in size to
minimize solder paste slump that causes shorts across its very close
leads. You can reduce this further with a paste that minimizes slump -
but those are typically difficult to get in small quantity. I would
recommend a Type 4 leaded paste and expect to do some cleanup on the
legs at the end (see below)

## Placing parts

Print `profile-drive/placement-r2.png` and use its legend as a
checklist.  Pick a parts bag, find it in the legend, place every part
in that group, and check it off.

Place them in whatever order suits you. I went smallest to largest. Leave
C40 (the 100 uF electrolytic) for last, because it's tall and gets in the
way of everything around it (particularly problematic for left-handers).

The legend marks two kinds of parts you can leave off:

- **DNP:** J4, J5, J6, J7 and SW1. None of them are needed to run the drive.
  SW1 and J7 pull PROGRAMN low, which is for programming the flash by hand
  through J6: holding PROGRAMN low keeps the FPGA off the flash's SPI lines,
  and letting go makes it reload. You don't need that route. The whole
  process runs from J4 with a pogo-pin clip, and nothing has to be soldered
  there. The other headers are for debugging.
- **optional:** the LEDs (D1 to D8) and their resistors (R35 to R42).
  Nothing else is on those nets. Without them you lose the status display,
  which is the easiest way to see the board working before it's on a //e.
  The top-level README's LED section covers colors and resistor values.

Polarity and pin 1: the sheet marks pin 1 of every part in red. "North"
is the DB-25 and USB-C edge:

- **D1 to D8 (LEDs):** cathode toward the south edge, the microSD side. The
  silk has a thick bar on the cathode end. In diode mode of a digital test
  meter, the LED lights when the black lead is on the cathode.
- **D9 (SMF5.0A):** cathode, the striped end, toward the west. The silk bar
  is on that end. It's on +5V.
- **C40:** + toward the north edge.
- **X1:** pad 1 (OE, tied to +3V3) is lower left, 2 (GND) lower right,
  3 (CLK) upper right, 4 (VDD) upper left. Find the dot on X1 for pin 1.

## Paste and reflow

Use Type 4 paste or finer. U1 is a 0.5 mm pitch TQFP, and J1's signal pins
are just as fine. With Type 4 I still got solder bridges on U1's pins.

Type 5 might help. Its powder is 15 to 25 um, against 20 to 38 um for
Type 4, so it prints cleaner through small apertures. Chip Quik's Smooth
Flow NC191 (Sn63/Pb37) comes in a T5 grade. Most of the bridging is too much
paste, though, which is what the reduced stencil is for: it shrinks the
apertures on U1 and J1 by 0.04 mm. Use it either way. Or use the non-reduced
stencil if you think it's better for you.

I reflowed the board, sticking to the paste maker's recommended profile.
Don't let the paste sit hot longer than the profile calls for, because it
slumps, and slumped paste on U1's pins is where the bridge shorts come from.

## Inspection

Before anything touches power, look at every joint under magnification
if you can. Pay the most attention to U1 and J1, from a low angle
along each row of pins, because a thin web of solder under the lead
tips is hard to see from above.  Remove any bridge you find with flux
and solder braid, or drag the iron with flux along the row. If nothing
else seems to be working to clear a short, try applying enough solder
to the pins on a row of U1 that it bridges all of them together; then
apply flux and use solder braid to pull all of the solder back
off. This will help wick out solder bridges that might be under the
leads in a place you can't see them. (This will be important if you run
the resistance checks below and still can't find the error.)

## Resistance checks

With the board unpowered, measure each rail to GND at these points. North is
the DB-25 and USB-C edge.

| Rail | Red lead | Black lead (GND) |
|---|---|---|
| +5V | C40 north pad (the + end) | C40 south pad |
| +3V3 | C3 west pad | C3 east pad |
| +1V1 | C4 west pad | C4 east pad |
| +2V5 | FB1 south pad | C1 north pad |
| VCCAUX_F | FB1 north pad | C1 north pad |

The meter charges the rail's capacitors as it measures, so the reading
starts low and climbs. That's normal. What you're looking for is a short:
anything that settles under about 100 ohms is a fault, and a good rail settles
at kilo-ohms or more. On +3V3 and +1V1, the regulator feedback
dividers are the most a rail can read with nothing else on it (R33 + R34 is
334k, R31 + R32 is 184.5k). The FPGA and everything else on the rail pull
it lower than that.

If a rail reads short, find out which side of it the short is on
before you start lifting parts. For VCCAUX_F and +2V5, remove FB1 and
measure each side again:

- Short on the north side (VCCAUX_F): it's U1 or one of C1, C20, C21, C22
  and C23. Pin 132 is the first suspect.
- Short on the south side (+2V5): it's U7 or C9.

Mine read 3 ohms on the VCCAUX_F side with FB1 and C1
removed. Dragging solder along U1's pins and wicking it back off
cleared it (as I mentioned above). That was on U1 pin 132 (VCCAUX) to
pin 131 (GND) - the only VCCAUX pin with a GND neighbor. Pins 131/132
are on U1's south row, about directly above C16.

The other VCCAUX pins (17, 53 and 96) have no GND neighbor, but pin 17
sits right beside pin 16, which is +3V3. That's on U1's east row, next
to C25. A bridge there won't show up as a short to ground. It shows up
at power-up, as FB1's north pad reading 3.3 V instead of 2.5 V. If you
see that, unplug right away. On the other side of FB1, +2V5 only has U7
and C9 on it.

If +1V1 (C4's west pad) reads short, three of U1's +1V1 pins have a GND
neighbor, and one of them is in the same spot as the VCCAUX one: pin
130 sits between GND pins 129 and 131, two pins over from 132, above
C16. The others are pin 20 (GND at 21) on the east row, next to C20,
and pin 66 (GND at 65) on the north row, next to C14. You can rule out
the regulator by temporarily removing L1. U5 only reaches the rail through
it, so if the short stays on the rail side, it's U1 or one of its caps.

If +3V3 (C3's west pad) reads short, there are more places to look,
because nearly everything on the board runs from it. Seven of U1's nine
+3V3 pins have a GND neighbor:

- east row: pin 9 (GND at 8), next to C24, and pin 16 (GND at 15),
  next to C25
- north row: pin 43 (GND at 42), next to C27
- west row: pin 86, next to C29, and pin 100 (GND at 101), next to C30
- south row: pin 122 (GND at 123), next to X1, and pin 137 (GND at
  138), next to C23

I'd look at pin 86 first, because it has GND on both sides (85 and 87).
After U1, check the other fine-pitch parts on the rail: U2 and U3, the
microSD socket J3, and the flash U4.


## First power

Power the board from USB-C only, with nothing else connected: no DB-25
cable, no SD card, and nothing on the headers. Never power it from J4. (J4
pin 1 is the board's own +3V3, and the board makes its 1.1 V core supply
from the USB 5 V, so feeding 3.3 V in there leaves the board half powered.)

If you have a USB power meter, plug in through it. A board with a
blank flash has nothing to run and should draw little current. If it
draws hundreds of mA, or the current keeps climbing, unplug it and
feel for the part that's hot.

Then measure the rails at the same points as the resistance checks:

| Rail | Expect | Set by |
|---|---|---|
| +5V | 4.75 to 5.25 V | USB |
| +3V3 | 3.32 V | U6, R33 274k / R34 60k4 |
| +1V1 | 1.11 V | U5, R31 84.5k / R32 100k |
| +2V5 | 2.50 V | U7 |
| VCCAUX_F | 2.50 V | +2V5 through FB1 |

If any rail is more than about 5% off, unplug it and find out why before you
go further. After 30 seconds, U5, U6, U7 and U1 should be no more than
lukewarm.

## Programming the FPGA

Connect a FT4232H (or similar) to J4 through a pogo-pin clip (or
solder in headers and connect that way), with the board powered from
USB-C. Use channel A, under the module's UART labels:

| FT4232H | J4 |
|---|---|
| TXD | pin 4, TCK |
| RXD | pin 3, TDI |
| RTS | pin 2, TDO |
| CTS | pin 5, TMS |
| GND | pin 6 |
| 3V3 | nothing; pin 1 stays open |

Check the FPGA answers:

    openFPGALoader -c ft4232 --detect

It should report IDCODE `0x21111043`, the LFE5U-12F. If it doesn't:

- **All 1s (`0xffffffff`) or all 0s:** nothing is answering on TDO. Check
  the clip is seated on all six pads, GND is connected, and the board is
  powered from USB-C. Then check the wiring against the table, especially
  RXD and RTS: TDI and TDO swapped is the easy mistake.
- **No device found, or the cable won't open:** openFPGALoader uses channel
  A of the FT4232H. Make sure you wired that channel, and that nothing else
  (a serial terminal, say) has the port open.
- **Some other IDCODE:** a buffered JTAG adapter can need an enable line
  driven before it passes signals through, and it can return a garbled
  IDCODE until it gets one. A plain FT4232H or FT2232H module doesn't have
  that problem.

Then, in `fpga/`:

    make r2-prog        build and load it into SRAM, volatile
    make r2-flash-dual  write it to the SPI flash, twice

`make r2-prog` first lets you check the board before the flash is written.
`make r2-flash-dual` writes the gateware twice: the copy that runs, and a
golden copy the board falls back to if the first is ever damaged, plus the
record that points the FPGA at the golden one. After that, updates can come
from the SD card (the top-level README covers it).
Building needs the tools in the top-level README. `fpga/README.md` has the
details under "Building for the r2 board".

`make r2-flash-dual` should find the flash as a Winbond W25Q128 (JEDEC ID
`0xef4018`), and the FPGA reloads from it when the last write finishes. If you
fitted the LEDs:

- **No SD card:** D6 blinks at about 1.5 Hz. That's the heartbeat, and it
  means the FPGA loaded from flash and is running.
- **A FAT32 card in:** the LEDs settle at `$E0`: D6, D7 and D8 lit and D1 to
  D5 dark. Anything else, read D1 to D5 as the error code in
  `fpga/README.md`'s SD page table.

If you fitted J5 too, a 3.3 V serial adapter on it (pin 1 GND, pin 2 TX,
pin 3 RX, 115200 8N1) shows the drive's report, starting with its build ID.

### Programming the flash through J6 instead

You don't need this, since JTAG can write the flash. J6 is there in case
JTAG isn't working, or you'd rather use an SPI flash programmer. Its pins:

| J6 | signal |
|---|---|
| 1 | +3V3, for the programmer's reference only; don't power the board from it |
| 2 | GND |
| 3 | flash /CS |
| 4 | flash CLK (through R43) |
| 5 | flash MOSI (DI) |
| 6 | flash MISO (DO) |
| 7 | PROGRAMN |

The FPGA shares those SPI lines, so PROGRAMN (pin 7) has to be held low the
whole time the programmer is writing. Holding SW1 down does the same thing.
Letting go makes the FPGA reload from the new image. Power the board from
USB-C as usual.

Any 3.3 V programmer that flashrom supports will work. On r1 I used a D1
mini running a serprog sketch that held PROGRAMN low itself. Write
`profile_emu_r2.bit`, the compressed flash flavor that `make r2` builds, at
the start of the flash. flashrom wants an image the size of the chip or a
layout region, so pad the file out to a round size (1 MB is plenty) and
write just that region. The W25Q128 should identify as JEDEC ID `0xef4018`.

## Putting it in service

Prepare an SD card as the top-level README describes, put it in, and plug
the board into the ProFile card's cable. It takes its power from USB-C.

Then cold-start the //e. The card ROM looks for the drive once, about 0.6 s
after autostart reaches its slot, and skips the slot for good if the drive
isn't ready by then. The r2 build in the Makefile is tuned to come up in
time. If the //e boots past the drive, power-cycle it with the drive
already powered and see if that changes anything.

The case in `enclosure/` takes M3 x 5 screws for the board (the pilot
gives about 3.4 mm of thread) and something like M3 x 12 or #4-40 x
1/2" for the cover. Either a pan or socket head work for the long
screws; the internal screws should be pan head with a diameter of
6.4mm or less (a M3 x 5 thread-forming pan head is perfect).


## Substitutes that worked

I didn't use all the parts that I originally specified; here are some replacements I used. The 22uF cap really sould be a 25V for C2; at 10v, it's right at its tolerance for spikes coming in through the USB-C port during plug-in.

| BOM | used | notes |
|---|---|---|
| 1 uF 0603 (C10, C37) | KEMET C0603C105K8PACTU | |
| 22 uF 1206 25 V (C2, C3, C4) | KEMET C1206C226M8RACTU, 10 V | same footprint; 10 V derates more under DC bias, but the rails are 5 V and below |
| 74LVC245 | 74LVC245APW-Q100J | the automotive grade of the same part |
| 25 MHz oscillator | ECS-2520MV-250-CN-TR | same pinout, 'C' is a higher fidelity part |
| ferrite bead | Murata BLM18AG121SN1D | fine, as long as it's rated above about 300 mA |
