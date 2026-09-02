# Makefile for the ProFile emulator. It drives arduino-cli.
#
# Common targets:
#   make deps      Install/verify the Teensy board package
#   make build     Compile the sketch (default target)
#   make upload    Compile and flash to a connected Teensy
#   make monitor   Open the USB serial monitor at 115200 baud
#   make clean     Remove build artifacts
#
# Override any variable on the command line, e.g.:
#   make upload PORT=/dev/cu.usbmodem12345
#   make build FQBN=teensy:avr:teensy40

# --- Configuration ---------------------------------------------------------
SKETCH       := emulator.ino
SKETCH_NAME  := $(basename $(SKETCH))
FQBN         ?= teensy:avr:teensy41
BAUD         ?= 115200
# Absolute path: the Teensy Loader runs as a separate GUI process (cwd "/"), so a
# relative build path makes it fail with "unable to read your compiled sketch".
BUILD_DIR    ?= $(CURDIR)/build
ARDUINO_CLI  ?= arduino-cli

# arduino-cli insists a sketch's main .ino be named after its directory, and this
# directory is "teensy" while the sketch is "emulator.ino". Rather than rename
# either one, stage a sketch dir named after the sketch and fill it with symlinks
# back to the real sources, then compile that.
STAGE_ROOT   := $(CURDIR)/.stage
STAGE_DIR    := $(STAGE_ROOT)/$(SKETCH_NAME)
SOURCES      := $(wildcard *.ino *.h *.hpp *.c *.cpp)
# Subdirectories carried along too: selector/ holds the 6502 blobs the sketch
# includes, and lives here as a submodule.
SUBDIRS      := $(patsubst %/,%,$(filter-out .stage/ build/,$(wildcard */)))

# Teensy boards are distributed through PJRC's third-party package index.
TEENSY_URL   := https://www.pjrc.com/teensy/package_teensy_index.json
URL_FLAG     := --additional-urls $(TEENSY_URL)

# Serial port, used by `monitor` only. Auto-detect the first usbmodem.
PORT         ?= $(shell $(ARDUINO_CLI) board list 2>/dev/null | awk '/serial/ && /usbmodem/ {print $$1; exit}')

# Teensy's HID upload port (protocol "teensy", e.g. usb:130000), used by `upload`.
# This is NOT the serial port. With several Teensies attached, auto-detect grabs
# the FIRST, so pass it explicitly to pick the right board (see the upload target).
TEENSY_PORT  ?= $(shell $(ARDUINO_CLI) board list 2>/dev/null | awk '$$2=="teensy"{print $$1; exit}')

# --- Targets ---------------------------------------------------------------
.PHONY: all build compile upload flash monitor deps stage clean list help

all: build

## Install the Teensy platform (idempotent).
deps:
	$(ARDUINO_CLI) core update-index $(URL_FLAG)
	$(ARDUINO_CLI) core install teensy:avr $(URL_FLAG)

## Stage the sketch under a directory arduino-cli will accept.
stage:
	@mkdir -p $(STAGE_DIR)
	@for f in $(SOURCES); do \
	    case "$$f" in \
	      $(SKETCH)) dest=$(STAGE_DIR)/$(SKETCH_NAME).ino ;; \
	      *)         dest=$(STAGE_DIR)/$$f ;; \
	    esac; \
	    rm -f "$$dest"; \
	    ln -s $(CURDIR)/$$f "$$dest"; \
	  done
	@for d in $(SUBDIRS); do \
	    rm -f $(STAGE_DIR)/$$d; \
	    ln -s $(CURDIR)/$$d $(STAGE_DIR)/$$d; \
	  done

## Compile the sketch.
build compile: stage
	$(ARDUINO_CLI) compile --fqbn $(FQBN) --build-path $(BUILD_DIR) $(STAGE_DIR)

## Compile and upload to the board.
## Uses the Teensy HID port (protocol "teensy"), NOT a serial /dev/tty port.
## Handing it a serial port makes the loader fail ("Unable to configure Teensy
## Loader to Auto mode!"). With more than one Teensy attached, name the target:
##   make upload TEENSY_PORT=usb:130000     (list them: arduino-cli board list)
## or just unplug the other boards so only the target remains.
upload flash: stage
	@test -n "$(TEENSY_PORT)" || { echo "No teensy port found; is the board plugged in? (arduino-cli board list)"; exit 1; }
	$(ARDUINO_CLI) compile --fqbn $(FQBN) --build-path $(BUILD_DIR) --port $(TEENSY_PORT) --upload $(STAGE_DIR)

## Open the serial monitor.
monitor:
	@test -n "$(PORT)" || { echo "No port found; set PORT=/dev/cu.usbmodemXXXX"; exit 1; }
	$(ARDUINO_CLI) monitor --port $(PORT) --config baudrate=$(BAUD)

## Show detected boards / ports.
list:
	$(ARDUINO_CLI) board list

## Remove build artifacts.
clean:
	rm -rf $(BUILD_DIR) $(STAGE_ROOT)

help:
	@grep -E '^##' $(MAKEFILE_LIST) | sed 's/## //'
