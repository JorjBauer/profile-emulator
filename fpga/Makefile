# ProFile FPGA emulator: yosys / nextpnr-ecp5 / prjtrellis flow for the ULX3S.
#
#   make          synthesize, place and route, pack
#   make sim-all  simulate: SD card and ProFile protocol (do this first)
#   make prog     load over USB into SRAM (fujprog)
#   make flash    write SPI flash, so it survives a power cycle
#   make stat     resource report only, yosys alone, no place and route
#
# The r2 board in hw-r2/ builds from the same sources; see "the r2 board" below:
#
#   make r2           both r2 bitstreams, the flash copy and the SRAM copy
#   make r2-prog      r2: load SRAM over JTAG (FT4232H on J4), volatile
#   make r2-flash     r2: write the SPI flash over JTAG, through the FPGA
#
# The board is the ULX3S, wired to the interface board as described in
# carrier/ULX3S-ADAPTER.md. The full design, recursion and all, fits with room
# to spare; `make stat` reports the count.
#
# --25k, NOT --12k, even on a board sold as a 12F: the ULX3S 12F carries 25F
# silicon, so the usable fabric is the 25F's 24288 LUT4.
#
# The two vendor primitives are selected by the FAMILY parameter and there are
# only two of them: the PLL in src/top.v and the data-bus pads in
# src/profile_bus.v. `hierarchy -purge_lib` before synth is required: it drops
# the default-parameter elaboration of profile_bus. That elaboration still
# contains the SB_IO branch, which synth_ecp5 would otherwise refuse.

PROJ    := profile_emu_ulx3s
LPF     := constraints/ulx3s.lpf
ECP5DEV := --25k
ECP5PKG := CABGA381
# ---- the selector's 6502 blobs, imported from the submodule ----------------
# ../selector is a submodule and builds itself. Its committed generated artifact
# is selector_blobs.h; src/selector.v reads the two .hex files derived from it.
#
# This build NEVER runs the submodule's generator. If the header is stale,
# someone re-makes it over there: each module is responsible for its own output.
# All that happens here is that make notices the header is newer than our copies
# and re-derives them, so the FPGA and the Teensy cannot drift apart unnoticed.
#
# One recipe writes both files. GNU make 3.81 (which is what ships on macOS) has
# no grouped targets, so it runs the recipe once per out-of-date target; the
# import is idempotent, so that is harmless.
SELECTOR_H := ../selector/selector_blobs.h
# These are BUILD PRODUCTS: derived on demand, removed by `make clean`, and
# not checked in. Anything that elaborates src/selector.v lists them as a
# prerequisite, because selector.v $readmemh's them at elaboration time.
BLOBS      := src/blobs/sel_boot.hex src/blobs/sel_body.hex

$(BLOBS): $(SELECTOR_H) tools/import_blobs.js
	@echo "importing selector blobs from $(SELECTOR_H)"
	@mkdir -p src/blobs
	node tools/import_blobs.js $(SELECTOR_H) src/blobs

blobs: $(BLOBS)

SRC     := src/top.v src/profile_bus.v src/profile_target.v src/sd_spi.v \
           src/block_router.v src/spare_table.v src/geometry.v src/verifier.v \
           src/bus_exercise.v src/block_stream.v src/fat32.v src/selector.v \
           src/uart_tx.v src/uart_rx.v src/reporter.v src/bus_capture.v src/sd_arb.v src/usb_prov.v

# The parameters every ULX3S build carries.
#
# USE_RSTBTN=0: the ULX3S rst_n pad is FIRE1, which idles LOW (PULLMODE=DOWN),
# and top.v's reset term is USE_RSTBTN & ~rst_n, so honoring it holds the whole
# design in reset forever. POR still resets at configuration; FIRE1 is simply
# inert.
ECP5_PARAMS := -set FAMILY \"ECP5\" -set USE_PLL 1 -set USE_RSTBTN 0

# Kept for `make stat`, which needs no BUILD_ID; the real builds stamp one.
#
# `rename -enumerate` is what makes the count a property of the DESIGN. yosys
# names every internal cell by source file and line ($eq$src/x.v:99). Passes
# iterate cells in name order. ABC's packing depends on the order it is handed
# the netlist. So without this, the LUT4 figure moves by about 1% when a
# comment is added or a line is re-wrapped. Enumerated names depend only on
# creation order, which formatting cannot change. The shipping build is left
# alone: its netlist works and is not to be re-shuffled by a reporting fix.
ECP5_YS = read_verilog $(SRC); \
	  chparam -set FAMILY "ECP5" -set USE_PLL 1 top; \
	  hierarchy -top top -purge_lib; proc; rename -enumerate;

all: $(PROJ).bit

# ---- the build rules, once, for every bitstream flavor ----------------------
# $(1) is the project basename, $(2) any extra chparam settings on top of
# $(ECP5_PARAMS), $(3) the LPF and $(4) the package; if those two are left
# blank, then the ULX3S's $(LPF) and $(ECP5PKG) are used. $(5) is an extra
# source file and $(6) the top module it defines, for a board that wraps top
# (r2). If $(6) is blank, the top module is top. Every flavor gets four files:
#
#   <p>.json           synthesized netlist
#   <p>.config         placed and routed
#   <p>.bit            COMPRESSED, fast-read: the FLASH copy
#   <p>_nextflash.bit  uncompressed: the SRAM copy
#
# --idcode stamps the LFE5U-12F IDCODE (0x21111043) into the 25F-targeted
# bitstream. The board's chip answers JTAG as a 12F even though its fabric is
# 25F silicon, and fujprog/openFPGALoader refuse on the mismatch otherwise.
#
# FLASHFREQ is the MCLK the part reads its own flash at, in MHz. 38.8 is the
# tested default; a board can raise it with a target-specific value (r2 does).
FLASHFREQ ?= 38.8
#
# --compress + --freq 38.8 + fast-read are the INSTANT-ON flags for the FLASHED
# copy. At ecppack's 2.4 MHz default MCLK, an uncompressed 5.4 Mbit stream takes
# ~2.25 s to load from flash. That alone loses the power-on race against the
# //e. Compressed (675 KB -> 328 KB) at 38.8 MHz, the same load is ~70 ms.
# THESE FLAGS ARE FLASH-ONLY. A JTAG SRAM load of this compressed/fast-read
# flavor makes the ECP5 config engine abort with "BSE Abort ERR" and leaves the
# FPGA blank. SRAM loads use _nextflash.bit instead.
define ECP5_BUILD

$(1).json: $$(SRC) $(5) $$(BLOBS)
	@date -u +%m%d%H%M > .build_id
	@echo "BUILD_ID $$$$(cat .build_id) ($(1))"
	BID=$$$$((16#$$$$(cat .build_id))); \
	yosys -p "read_verilog $$(SRC) $(5); \
	  chparam $$(ECP5_PARAMS) $(2) -set BUILD_ID $$$$BID top; \
	  hierarchy -top $(if $(6),$(6),top) -purge_lib; \
	  synth_ecp5 -top $(if $(6),$(6),top) -json $$@"

$(1).config: $(1).json $(if $(3),$(3),$$(LPF))
	nextpnr-ecp5 $$(ECP5DEV) --package $(if $(4),$(4),$$(ECP5PKG)) --json $$< \
	  --lpf $(if $(3),$(3),$$(LPF)) --textcfg $$@ --freq 50

$(1).bit: $(1).config
	ecppack --idcode 0x21111043 --compress --freq $$(FLASHFREQ) --spimode fast-read $$< $$@

$(1)_nextflash.bit: $(1).config
	ecppack --idcode 0x21111043 --freq 38.8 $$< $$@

CLEANFILES += $(1).json $(1).config $(1).bit $(1)_nextflash.bit


endef

$(eval $(call ECP5_BUILD,$(PROJ),))

# Re-program SRAM (volatile), rebuilding if sources changed. Uses fujprog + the
# uncompressed bit; do NOT point openFPGALoader at the compressed $(PROJ).bit
# for SRAM: it aborts (see above). Needs the oss-cad-suite tools on PATH
# (`source ~/oss-cad-suite/environment`).
prog: $(PROJ)_nextflash.bit
	fujprog $<

# Reload SRAM from the already-built uncompressed bit with NO rebuild: the fast
# way back to the default image after a live swap.
reload:
	fujprog $(PROJ)_nextflash.bit

# Write the compressed instant-on bitstream to SPI flash (persists across power).
# openFPGALoader, NOT fujprog: this bitstream carries a --spimode fast-read
# command (0x79) in its header, and fujprog's parser cannot walk past it to
# reach the IDCODE record, so it aborts with "can't find IDCODE, invalid
# bitstream" before touching the board. openFPGALoader understands that header.
# fujprog still handles the SRAM path (prog/reload), whose bitstream has no
# spimode command.
flash: $(PROJ).bit
	openFPGALoader -b ulx3s -f $<

# Resource count with no place and route, so the fit can be judged before
# committing to a full build.
stat: $(BLOBS)
	@yosys -p '$(ECP5_YS) synth_ecp5 -top top; stat' > /tmp/ecp5-stat.log 2>&1 || \
	  { tail -20 /tmp/ecp5-stat.log; exit 1; }
	@awk '/=== design hierarchy ===/{p=1} p' /tmp/ecp5-stat.log | \
	  grep -E '^ +[0-9]+ +(LUT4|CCU2C|TRELLIS_FF|DP16KD|TRELLIS_IO|EHXPLLL|PFUMX|L6MUX21)' | tail -8

# Older names for the same targets.
ecp5:      all
ecp5-prog: prog
ecp5-reload: reload
ecp5-flash: flash
ecp5-stat: stat

# Simulation. Run this before touching hardware: it is the only check that
# catches a wiring-level mistake before two drivers fight on a 5V net.
SIMBIN  := /tmp/profile_emu_tb

# TRELLIS_IO is a Lattice cell: yosys has its own, iverilog needs ours. It goes
# to the simulator ONLY, never into $(SRC), or synthesis would see two
# definitions of the cell.
#
# The benches build FAMILY="ECP5", so the simulated pads are the pads the
# shipping bitstream uses.
SIMSRC  := $(SRC) sim/trellis_io.v

# sim      the ProFile protocol against a model of the READER's host sequence,
#          which is the forgiving one (microseconds per byte)
# sim-iie  the same protocol against a model of the APPLE //e, which is the
#          strict one: 355 ns strobes and no settling delay after PR/-W rises
# sim-sd   SD init + a CMD17 read, against a model SDHC card, no host at all
# sim-geom the claimed/served block counts and the multiple-of-4 boot rule
# sim-all  all of them
sim: sim/tb_top.v $(SIMSRC) $(BLOBS)
	iverilog -g2012 -o $(SIMBIN) -s tb_top sim/tb_top.v $(SIMSRC)
	vvp $(SIMBIN)

# The //e, which is the strict host: a 355 ns strobe and no settling delay
# after PR/-W rises. This is the test that catches a design that works on the
# bench but fails to boot on real hardware.
sim-iie: sim/tb_top_iie.v sim/sd_model.v $(SIMSRC) $(BLOBS)
	iverilog -g2012 -o $(SIMBIN)_iie -s tb_top_iie sim/tb_top_iie.v sim/sd_model.v $(SIMSRC)
	vvp $(SIMBIN)_iie

# The same bench with the byte pacing squeezed to a quarter of the //e's. Not a
# requirement, a margin measurement. If this passes there is real headroom. If
# it fails, the design meets the nominal figure with no margin to spare.
sim-iie-fast: sim/tb_top_iie.v sim/sd_model.v $(SIMSRC) $(BLOBS)
	iverilog -g2012 -Ptb_top_iie.BYTE_NS=2250 -Ptb_top_iie.STROBE_NS=200 \
	  -o $(SIMBIN)_iief -s tb_top_iie sim/tb_top_iie.v sim/sd_model.v $(SIMSRC)
	vvp $(SIMBIN)_iief

# The decoupled-geometry case through the HOST's path: a 65535-block image,
# which can only ever claim $FFFC, reading a block above that claim. This is the
# full 32MB ProDOS volume (Total Recall and friends) and it's the only geometry
# that cannot be fixed by resizing the image.
sim-iie-32mb: sim/tb_top_iie.v sim/sd_model.v $(SIMSRC) $(BLOBS)
	iverilog -g2012 -Ptb_top_iie.IMG_BLOCKS=65535 \
	  -o $(SIMBIN)_iie32 -s tb_top_iie sim/tb_top_iie.v sim/sd_model.v $(SIMSRC)
	vvp $(SIMBIN)_iie32

# KNOWN FAILING, deliberately, and NOT in sim-all: a reproduction of the
# late-driven byte 0, not a regression guard. LATE_NS=150 models the //e's card
# strobing the first command byte before it has driven the bus, so the fall
# reads $FF. The design answers that with the delayed sample in
# src/profile_bus.v; this bench does not currently pass and does not terminate
# on its own. Keep it runnable so a change to the byte-0 timing can be checked
# against it; do not put it in sim-all.
sim-iie-late: sim/tb_top_iie.v sim/sd_model.v $(SIMSRC) $(BLOBS)
	iverilog -g2012 -Ptb_top_iie.LATE_NS=150 \
	  -o $(SIMBIN)_iiel -s tb_top_iie sim/tb_top_iie.v sim/sd_model.v $(SIMSRC)
	vvp $(SIMBIN)_iiel

sim-sd: sim/tb_top_sd.v sim/sd_model.v $(SIMSRC) $(BLOBS)
	iverilog -g2012 -o $(SIMBIN)_sd -s tb_top_sd sim/tb_top_sd.v sim/sd_model.v $(SIMSRC)
	vvp $(SIMBIN)_sd

# The boot-path geometry rule and the decoupled-geometry band, on their own.
# Pure combinational logic, so this one runs in milliseconds; there is no excuse
# for not running it.
sim-geom: sim/tb_geometry.v src/geometry.v
	iverilog -g2012 -o $(SIMBIN)_geom -s tb_geometry sim/tb_geometry.v src/geometry.v
	vvp $(SIMBIN)_geom

# The verifier, with its serial output decoded a bit at a time and every
# checksum recomputed independently. The CRC is a contract between this Verilog
# and tools/verify_image.js, so it is tested on both sides of the wire:
# `tools/verify_image.js --selftest` checks the script against the vectors this
# bench prints.
sim-verify: sim/tb_verifier.v src/verifier.v src/uart_tx.v
	iverilog -g2012 -o $(SIMBIN)_ver -s tb_verifier \
	  sim/tb_verifier.v src/verifier.v src/uart_tx.v
	vvp $(SIMBIN)_ver
	@node tools/verify_image.js --selftest

# The same, folding four sectors into each checksum. Stride is what makes a 4 GB
# card checkable at all, so it must be tested too.
sim-verify-stride: sim/tb_verifier.v src/verifier.v src/uart_tx.v
	iverilog -g2012 -Ptb_verifier.STRIDE=4 -Ptb_verifier.COUNT=8 \
	  -o $(SIMBIN)_vers -s tb_verifier sim/tb_verifier.v src/verifier.v src/uart_tx.v
	vvp $(SIMBIN)_vers

# A card that rejects the first two reads. The retry path has to recover so the
# reads still succeed. The retry count must be exactly two.
sim-retry: sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	iverilog -g2012 -Ptb_sd.FAILR=2 -o $(SIMBIN)_rty -s tb_sd \
	  sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	vvp $(SIMBIN)_rty

# And a card that rejects more reads than the budget allows, so the give-up path
# is tested too: an unbounded retry turns a failed card into a hang.
sim-retry-exhaust: sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	iverilog -g2012 -Ptb_sd.FAILR=5 -o $(SIMBIN)_rtx -s tb_sd \
	  sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	vvp $(SIMBIN)_rtx

# The no-host bus pattern, and the rule that it yields to a host instantly.
# The parked-read race: a read parked behind a busy resolver must not inherit
# the previous transaction's blk_ready.
sim-usb-prov:
	iverilog -g2012 -o /tmp/tb_usb_prov.vvp -s tb_usb_prov sim/tb_usb_prov.v src/usb_prov.v src/uart_tx.v
	vvp /tmp/tb_usb_prov.vvp

sim-router-race: sim/tb_router_race.v src/block_router.v src/spare_table.v
	iverilog -g2012 -o $(SIMBIN)_race -s tb_router_race \
	  sim/tb_router_race.v src/block_router.v src/spare_table.v
	vvp $(SIMBIN)_race

sim-exercise: sim/tb_exercise.v src/bus_exercise.v
	iverilog -g2012 -o $(SIMBIN)_exr -s tb_exercise sim/tb_exercise.v src/bus_exercise.v
	vvp $(SIMBIN)_exr

# The filesystem resolver: find an image in a subdirectory and resolve its
# cluster chain, which the model deliberately fragments.
sim-fat32: sim/tb_fat32.v sim/sd_model.v src/fat32.v src/sd_spi.v
	iverilog -g2012 -o $(SIMBIN)_f32 -s tb_fat32 \
	  sim/tb_fat32.v sim/sd_model.v src/fat32.v src/sd_spi.v
	vvp $(SIMBIN)_f32

# The shipping selection policy: with images present but none configured, serve
# NOTHING. SELECTOR.md is explicit that the drive must not invent a default and
# boot it without asking. Picking the largest image would do exactly that.
sim-fat32-noauto: sim/tb_fat32.v sim/sd_model.v src/fat32.v src/sd_spi.v
	iverilog -g2012 -Ptb_fat32.AUTO=0 -Ptb_fat32.NOCFG=1 -o $(SIMBIN)_f32n -s tb_fat32 \
	  sim/tb_fat32.v sim/sd_model.v src/fat32.v src/sd_spi.v
	vvp $(SIMBIN)_f32n

# A configured image whose name is a bare 8.3 name, with AUTOPICK OFF so that
# being resolved at all IS the name match. The card's entry is the on-disk
# "A2OSX   IMG" with no long-name chain in front of it, and profile.cfg asks for
# "/HD/a2osx.img" in lower case, so this passes only if the resolver compacts
# the 8.3 form into "A2OSX.IMG" and folds case on both sides.
sim-fat32-83: sim/tb_fat32.v sim/sd_model.v src/fat32.v src/sd_spi.v
	iverilog -g2012 -Ptb_fat32.NOLFN=1 -Ptb_fat32.AUTO=0 \
	  -o $(SIMBIN)_f8 -s tb_fat32 \
	  sim/tb_fat32.v sim/sd_model.v src/fat32.v src/sd_spi.v
	vvp $(SIMBIN)_f8

# The write-back, which is the ONLY bench that exercises it. It needs a
# selection that really changes: profile.cfg names the second image and the
# bench selects the first, because fat32 short-circuits a reselect of the index
# already being served ("nothing to do: already serving it") and no persist is
# attempted for one. It also has to WAIT: ready goes high before the persist
# runs, by design, so that blocks keep serving while it waits for the bus.
sim-fat32-persist: sim/tb_fat32.v sim/sd_model.v src/fat32.v src/sd_spi.v
	iverilog -g2012 -Ptb_fat32.PERSIST=1 -o $(SIMBIN)_fp -s tb_fat32 \
	  sim/tb_fat32.v sim/sd_model.v src/fat32.v src/sd_spi.v
	vvp $(SIMBIN)_fp

# The LRU-2 chain cache: two images, ephemeral reselect between them must be
# instant (zero SD reads) and serve each image's own fragmented chain.
sim-fat32-lru: sim/tb_fat32_lru.v sim/sd_model.v src/fat32.v src/sd_spi.v
	iverilog -g2012 -o $(SIMBIN)_f32lru -s tb_fat32_lru \
	  sim/tb_fat32_lru.v sim/sd_model.v src/fat32.v src/sd_spi.v
	vvp $(SIMBIN)_f32lru

# The resolver with recursion compiled out. FS_RECURSE=0 is what the exercise
# bitstream builds with, and it stays tested: same card, same config, same
# answer, because the model's image is directly in /HD. If this ever diverges,
# the flat resolver is no longer a subset of the recursive one.
sim-fat32-flat: sim/tb_fat32.v sim/sd_model.v src/fat32.v src/sd_spi.v
	iverilog -g2012 -Ptb_fat32.RECURSE=0 -o $(SIMBIN)_f32f -s tb_fat32 \
	  sim/tb_fat32.v sim/sd_model.v src/fat32.v src/sd_spi.v
	vvp $(SIMBIN)_f32f

# The boot menu's magic blocks: the info block, the body pages, the synthetic
# block 0 and SELECT. The 6502 side is not simulated; this checks that the drive
# answers what that side asks for.
sim-selector: sim/tb_selector.v src/selector.v $(BLOBS)
	iverilog -g2012 -o $(SIMBIN)_sel -s tb_selector sim/tb_selector.v src/selector.v
	vvp $(SIMBIN)_sel

sim-unit: sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	iverilog -g2012 -o $(SIMBIN)_unit -s tb_sd sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	vvp $(SIMBIN)_unit

# The same unit test against a card whose replies are two bits out of byte
# alignment, which is what the real one does. Without the alignment recovery in
# sd_spi.v this fails; with it, it must pass identically to the aligned case.
sim-skew: sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	iverilog -g2012 -Ptb_sd.SKEW=2 -o $(SIMBIN)_skew -s tb_sd \
	  sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	vvp $(SIMBIN)_skew

# Every bit phase, not just the one the real card happened to show. The
# alignment recovery has to be phase-independent or it is just a workaround.
sim-sweep: sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	@for k in 0 1 2 3 4 5 6 7; do \
	  iverilog -g2012 -Ptb_sd.SKEW=$$k -o $(SIMBIN)_sw -s tb_sd \
	    sim/tb_sd.v sim/sd_model.v src/sd_spi.v || exit 1; \
	  printf 'skew %s: ' $$k; \
	  vvp $(SIMBIN)_sw | grep -E '^(PASS|FAIL)' || exit 1; \
	done

# A card that rejects CMD8 and addresses by byte, which is what the real one
# looks like. Exercises the v1 fallback: ACMD41 without HCS, CCS from the OCR,
# and CMD17 arguments scaled by 512.
sim-v1: sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	iverilog -g2012 -Ptb_sd.V1=1 -o $(SIMBIN)_v1 -s tb_sd \
	  sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	vvp $(SIMBIN)_v1

# The real card's combination: v1 behavior AND a skewed reply.
sim-v1skew: sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	iverilog -g2012 -Ptb_sd.V1=1 -Ptb_sd.SKEW=2 -o $(SIMBIN)_v1s -s tb_sd \
	  sim/tb_sd.v sim/sd_model.v src/sd_spi.v
	vvp $(SIMBIN)_v1s

sim-all: sim sim-iie sim-iie-32mb sim-sd sim-geom sim-verify sim-verify-stride sim-retry sim-retry-exhaust sim-exercise sim-fat32 sim-fat32-flat sim-fat32-noauto sim-fat32-83 sim-fat32-persist sim-selector sim-unit sim-skew sim-sweep sim-v1 sim-v1skew

# ---- the r2 board (hw-r2/) -------------------------------------------------
# The same design again, wrapped in src/top_r2.v for the three pins r2 added
# (U3's /OE, the reboot line, and holding U2 off until a host appears), with
# its own pinout in constraints/profile-drive-r2.lpf. It gets its own project
# name so that an r2 build and a ULX3S build can sit side by side without
# clobbering each other.
#
# JTAG is an FT4232H module on J4, channel A, which the module labels as a
# UART: TXD to TCK (J4 pin 4), RXD to TDI (pin 3), RTS to TDO (pin 2), CTS to
# TMS (pin 5), GND to pin 6, nothing on pin 1. The board is powered from
# USB-C, never from J4.
R2PROJ   := profile_emu_r2
R2LPF    := constraints/profile-drive-r2.lpf
R2PKG    := TQFP144
R2CABLE  ?= ft4232

# r2 races the //e at power-on. The card ROM checks for the drive about 0.6 s
# after autostart reaches the slot, and it does not ask twice. Two settings
# make the r2 build come up faster.
#
# First, the part reads its flash at 62 MHz, ecppack's top rate (the W25Q128
# does 133). That takes the 348 KB load from about 72 ms to about 45.
#
# Second, the card comes up at 397 kHz (SD_DIV_INIT 62) instead of 100 kHz.
# That is just under the spec's 400 kHz for initialization. The faster rate is
# safe because the socket is on the board now, not on flying leads. The first
# read also stays at this initialization rate.
# Reads after it stay at 8.33 MHz, the last rate measured to return correct
# data. Re-measure with SWEEP_ENABLE before raising that.
R2PARAMS := -set SD_DIV_INIT 62
$(R2PROJ).bit: FLASHFREQ := 62.0

ifneq ($(PROJ),$(R2PROJ))
$(eval $(call ECP5_BUILD,$(R2PROJ),$(R2PARAMS),$(R2LPF),$(R2PKG),src/top_r2.v,top_r2))
endif

r2: $(R2PROJ).bit $(R2PROJ)_nextflash.bit

r2-prog: $(R2PROJ)_nextflash.bit
	openFPGALoader -c $(R2CABLE) $<

r2-reload:
	openFPGALoader -c $(R2CABLE) $(R2PROJ)_nextflash.bit

r2-flash: $(R2PROJ).bit
	openFPGALoader -c $(R2CABLE) -f $<

# ---- whole-volume verification bitstream ------------------------------------
# A SEPARATE bitstream, because the verifier holds the SD engine for the whole
# run and no build that serves a host should do that. Override the range on the
# command line; it must match what tools/verify_image.js is told:
#
#   make verify-prog VCOUNT=65536 VSTRIDE=1        # the 32MB a host can reach
#   make verify-prog VCOUNT=8388608 VSTRIDE=1024   # a whole 4GB card, coarsely
#
# then
#
#   tools/verify_image.js ~/Desktop/4gb-dump.img --port /dev/cu.usbserial-... \
#     --count 65536 --stride 1
VPROJ   := profile_emu_verify
VSTART  ?= 0
VCOUNT  ?= 65536
VSTRIDE ?= 1

$(eval $(call ECP5_BUILD,$(VPROJ),-set VERIFY_ENABLE 1 -set VERIFY_START $(VSTART) -set VERIFY_COUNT $(VCOUNT) -set VERIFY_STRIDE $(VSTRIDE)))

verify-bit: $(VPROJ).bit
verify-prog: $(VPROJ)_nextflash.bit
	fujprog $<

# ---- dump one sector as hex -------------------------------------------------
# Reads DLBA at the end of the self-test so the reporter's hex dump shows it.
# Used to answer "what actually differs" when the volume comparison flags a
# sector, which a checksum can never say.
#
#   make dump-prog DLBA=8193
DPROJ := profile_emu_dump
DLBA  ?= 0

$(eval $(call ECP5_BUILD,$(DPROJ),-set DUMP_ENABLE 1 -set DUMP_LBA $(DLBA)))

dump-prog: $(DPROJ)_nextflash.bit
	fujprog $<

# ---- bus exerciser ----------------------------------------------------------
# Drives a known pattern onto the DB-25 with no host, so the sniffer's live
# monitor can check the drive-owned half of the bus. See src/bus_exercise.v.
#
#   make exercise-prog
EPROJ := profile_emu_exercise
EDWELL ?= 20        # MICROseconds; see src/bus_exercise.v

# FS_RECURSE 0: this build never touches the filesystem, so the directory walk
# is unused logic. sim-fat32-flat keeps the flat resolver tested.
$(eval $(call ECP5_BUILD,$(EPROJ),-set EXERCISE_ENABLE 1 -set EXERCISE_DWELL_US $(EDWELL) -set FS_RECURSE 0))

exercise-prog: $(EPROJ)_nextflash.bit
	fujprog $<

# ---- block streamer ---------------------------------------------------------
# Clocks a real block from the card onto the DB-25 with no host, so the sniffer
# can check the whole read path against the card image. See
# src/block_stream.v and tools/check_stream.js.
#
#   make stream-prog SBLOCK=0
SPROJ  := profile_emu_stream
SBLOCK ?= 0
SDWELL ?= 20

$(eval $(call ECP5_BUILD,$(SPROJ),-set STREAM_ENABLE 1 -set STREAM_BLOCK $(SBLOCK) -set STREAM_DWELL_US $(SDWELL)))

stream-prog: $(SPROJ)_nextflash.bit
	fujprog $<

clean:
	rm -f $(CLEANFILES) $(BLOBS) .build_id

.PHONY: all blobs prog reload flash stat ecp5 ecp5-prog ecp5-reload ecp5-flash ecp5-stat \
        r2 r2-prog r2-reload r2-flash \
        sim sim-iie sim-iie-late sim-iie-fast sim-iie-32mb sim-sd sim-geom sim-verify \
        sim-verify-stride sim-retry sim-retry-exhaust sim-usb-prov sim-router-race \
        sim-exercise sim-fat32 sim-fat32-flat sim-fat32-noauto sim-fat32-83 sim-fat32-persist sim-fat32-lru \
        sim-selector sim-unit sim-skew sim-sweep sim-v1 sim-v1skew sim-all \
        verify-bit verify-prog dump-prog exercise-prog stream-prog clean
