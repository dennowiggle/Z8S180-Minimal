# A Makefile suitable for simulation only

TOP=../../../..

-include  $(TOP)/Make.local
include $(TOP)/Make.rules

# Defines
#
# PAGE_BYTES_528		- SPI FLASH can have 512 byte pages or 528 byte pages. 
#						  Set the flag if 528 page mode.
# TEST_BENCH            - Brings out extra signals.
# TB_DEBUG				- Extra prints for code debug.
#
DEFINES = -DPAGE_BYTES_528 -DTEST_BENCH -DTB_DEBUG

FILES= \
	../../../lib/wtm_resetSyncDelay.v       \
	../../../src/wtm_flashToMemory.v \
	spi_tx_bfm.v						    \
	../../../lib/wtm_sigSync.v 		        \
	../../../src/wtm_flashReader_v2_MisoInputSync.v 	
#	../../../src/wtm_flashReader_v1_LessLogic.v 	


.PHONY: all clean world run plot

all:: run

flashReaderToMem_tb.vvp : flashReaderToMem_tb.v $(FILES)
	iverilog $(DEFINES) -o $@ $^

# Run a simulation. But don't plot it.
run: flashReaderToMem_tb.vvp
	vvp flashReaderToMem_tb.vvp
	vcd2fst $(basename $^).vcd $(basename $^).fst 
	rm $(basename $^).vcd

plot: flashReaderToMem_tb.fst
	gtkwave $^


world:: clean all
