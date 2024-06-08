TOP=..

-include  $(TOP)/Make.local
include $(TOP)/Make.rules

# Compile options
#
# PAGE_BYTES_528	 : Used if SPI FLASH conforms to 528 byte page size.
#					   Default is 512 byte page.
#					

VERILOG_DEFINES = -D Yosys -D PAGE_BYTES_528

.PHONY: all clean burn world timings

FILES= \
	lib/wtm_resetSyncDelay.v \
	lib/wtm_resetSync.v \
	lib/wtm_sigSync.v \
	pll/pll_cpu.v \
	src/counter.v \
	src/wtm_busInterface.v \
	src/wtm_flashReader_v1_LessLogic.v \
	src/wtm_flashToMemory.v \
	src/top.v \

all:: top.bin
	cat top.bin firmware/RCZ180_Nouveau.rom > flash.bin

top.json: $(FILES)
	$(COMPILE.v) $(VERILOG_DEFINES) -p "synth_ice40 -top top -json $@" $(FILES) | tee $(basename $@)_yosys.rpt
timing: top.asc
	icetime -tmd $(DEVICE) $^

counter_tb.vvp: counter_tb.v counter.v
	iverilog -o $@ $^

plot: counter_tb.vcd
	gtkwave $^

prog: flash.bin
	$(FLASH_PROG) $^

# extra dependancies
top.asc: $(PINMAP)

