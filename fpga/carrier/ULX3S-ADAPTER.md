# Wiring the interface board to ULX3S J2

The ProFile interface board's J2 carries 25 FPGA-facing signals, and all of them
land on the ULX3S's J2 header. This is the pin map, plus the three things that
will damage parts or waste a build if you get them wrong.

Nothing else has to be connected: the clock, reset button, serial port and LEDs
are all on the ULX3S itself.

## Leave board pins 79 and 80 unconnected: they are 5V

They look like a fourth power pair at the end of the header and they are not:
pin 79 is IN5V and pin 80 is OUT5V, each sitting behind a Schottky (D51/D52).
Tying either to the 3V3 net puts 5V on both 74LVC245s and on the microSD card.
**This is the only wiring error here that destroys parts.**

## Rows

ULX3S J2 is board pins 41 to 80. **Top row is GN, the odd pins 41 to 79; bottom
row is GP, the even pins 42 to 80.** Read left to right from the side the cable
attaches, each row runs:

    Vcc  GND  x14 x15 x16 x17 x18 x19 x20  Vcc  GND  x21 x22 x23 x24 x25 x26 x27  GND  5V

where x is GN on the top row and GP on the bottom, and the last position is
IN5V on the top row and OUT5V on the bottom.

**Confirm the row binding before wiring anything.** emard's schematic numbers
J1/J2 for a female 90-degree header and warns that a male vertical one swaps the
rows. These are male right-angle, which is neither case, so the binding above is
what was observed on the board rather than what the drawing says. Use the
one-pin test: a bitstream that drives GP14 high and GN14 low, then probe
position 3 of each row. Getting it backward does not short the rail, because
+3V3 and GND land at the same positions on both rows. It puts every signal on
the wrong pin, which programs cleanly and does nothing.

## The pin map

Interface board J2 is a 2x18 header, pins 1 to 36, with the odd pins on one row
and the even pins on the other. Its pin names come from the interface board
schematic. Both tables are by pin number, so they hold whichever way round the
connectors physically face.

### GP row: the interface board's even pins

| IF J2 | net | ULX3S | board pin | ECP5 ball | run |
|---:|---|---|---:|---|---:|
| 4 | PD0 | GP27 | 76 | D13 | straight |
| 6 | PD1 | GP26 | 74 | B13 | straight |
| 8 | PD2 | GP25 | 72 | D14 | straight |
| 10 | PD3 | GP24 | 70 | C16 | straight |
| 12 | PD4 | GP23 | 68 | B17 | straight |
| 14 | PD5 | GP22 | 66 | B15 | straight |
| 16 | PD6 | GP21 | 64 | C18 | straight |
| 22 | /PCMD | GP20 | 58 | D18 | straight |
| 24 | /PSTRB | GP19 | 56 | F17 | straight |
| 26 | PR/W | GP18 | 54 | H18 | straight |
| 28 | /PRES | GP17 | 52 | L16 | straight |
| 30 | /PH0 | GP16 | 50 | N16 | straight |
| 32 | AUX7 | GP15 | 48 | N17 | straight |
| 34 | SD_CS | GP14 | 46 | U18 | straight |

Fourteen of fourteen, and every one of them straight across. The data byte
reads high to low along the connector, which is the facing orientation rather
than a mistake: `pd[0]` is still wired to IF J2.4.

### GN row: the interface board's odd pins, plus two that had nowhere else

| IF J2 | net | ULX3S | board pin | ECP5 ball | run |
|---:|---|---|---:|---|---:|
| 3 | XCVR_DIR | GN27 | 75 | E13 | straight |
| 5 | XCVR_/OE | GN26 | 73 | C13 | straight |
| 18 | PD7 | GN22 | 65 | C15 | 2 pos |
| 36 | SD_WP | GN21 | 63 | D17 | 10 pos |
| 21 | SD_CLK | GN20 | 57 | E17 | straight |
| 23 | /PBSY | GN19 | 55 | G18 | straight |
| 25 | SD_MOSI | GN18 | 53 | H17 | straight |
| 27 | /PPARITY | GN17 | 51 | L17 | straight |
| 29 | SD_MISO | GN16 | 49 | M17 | straight |
| 31 | PCHK | GN15 | 47 | P16 | straight |
| 35 | SD_CD | GN14 | 45 | U17 | 1 pos |
| - | spare | GN23 | 67 | C17 | - |
| - | spare | GN24 | 69 | D16 | - |
| - | spare | GN25 | 71 | E14 | - |

`PD7` and `SD_WP` are the two signals whose aligned position lands on a ULX3S
power pin, so they cannot be straight. `SD_WP` takes the long run of the two by
choice: it is a mechanical switch contact that never toggles, and `HONOR_WP` is
0 today, so it is the signal that cares least. `SD_CD` is one position over.

The three spares sit together in the middle of the row, which is where a test
header can reach them.

### Power and no-connects

| IF J2 | net | ULX3S board pins |
|---:|---|---|
| 2, 20 | +3V3 | 41, 42, 59, 60 (all four) |
| 1, 19 | GND | 43, 44, 61, 62, 77, 78 (all six) |
| 7, 9, 11, 13, 15, 17, 33 | not used on the interface board | no connection |
| - | - | **79, 80: DO NOT CONNECT, these are 5V** |

Tie all four +3V3 pins and all six grounds. J2's power pins are the `+3V3` net
directly, with no selection in front of them, so there is no VCCIO option to set
on this connector. (That is not true of J1, which is on the selectable
`2V5_3V3` rail; do not move to J1 without revisiting the level translation.)

Put 10 uF plus 100 nF on the 3V3 net at the interface-board end. The board
already has C3 at its own J2 pins, but the ULX3S's regulator is a longer run
away than that decoupling alone was sized for.

## Why is there an extra SD card socket?

This map uses its own SD socket; the ULX3S has one, but this prototype
is about testing the hardware that we'll be using for the final
PCB. Since that's not the ULX3S itself I decided to build around a
second, dedicated, SD card socket. The prototype here uses a full SD
carrier; the final board uses a microSD that doesn't have (for
example) card detected capability (I'm going with the cheapest parts I
can find, it seems a small sacrifice).
