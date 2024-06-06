TOP=..

-include  $(TOP)/Make.local
include $(TOP)/Make.rules

# Compile options
#
# SD_CARD_FREQ_25MHZ : 	Default SD card SPI clock is 12.5MHz. Use for 25MHz SD card clock.
#
# EN_VRAM_READ       : 	VDP vram reads by the CPU are disabled by default and the vram 
# 							memory will be clocked at 50MHz. 
#						Warning : This option enables reads by the CPU but image may not  
#							meet timing and work properly. In one 50MHz clock cycle two 
# 							reads are performed at 100MHz my muxing the reads.
#
# UART CLEAR_VALID_OPTION : UART setting.
#                      		The internal uart_cpu_clear_valid signal relies on
# 					   		the iorq_n read signal going high. With this compiler option
#					   		set the IO Address is also checked and the signal is delayed
#					   		by two clock cycles. Use if uart read stability issue.
#  
# PAGE_BYTES_528	 : Used if SPI FLASH conforms to 528 byte page size.
#					   Default is 512 byte page.
#					

# VERILOG_DEFINES = -D Yosys -D SD_CARD_FREQ_25MHZ -D EN_CPU_READ_VRAM
# VERILOG_DEFINES = -D Yosys -D SD_CARD_FREQ_25MHZ -DCLEAR_VALID_OPTION
VERILOG_DEFINES = -D Yosys -D SD_CARD_FREQ_25MHZ -D PAGE_BYTES_528 -D EN_CPU_READ_VRAM

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

prog: top.bin
	$(FLASH_PROG) $^

# extra dependancies
top.asc: $(PINMAP)

