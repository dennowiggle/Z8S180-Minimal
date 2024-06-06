# FPGA code for 2057-ICE40HX4K to boot 2067-Z8S180-Rev2 CPU board.

## Functionality

* Controls CPU inputs based on CPU outputs.
* Provides clock and reset to the CPU.
* Provides Memory Control Signals.
   - SRAM memory is on the CPU board. The FPGA sets the SRAM R/W/OE signals dependant on the CPU R/W/MREQ and address signals.
* Incorporates ROM boot code starting at address 0x00000 and ROM is currently 8K in size.
* The ROM can be disabled by the CPU by writing to an FPGA register bit allowing the CPU to access the 0-0x2000 SRAM address space previuosly occupied by the ROM.
* Provides an SD card interface for the CPU. Data access is via two 512 byte buffers on the FPGA that are in the CPU address space.


## FPGA Registers

FPGA registers are accessible using IO address 0x50 (address) and 0x51 (data). Registers are :
* 0x00 = (R)  Version register.
* 0x01 = (RW) Test Register.
* 0x02 = (RW) LED Control Register.
* 0x03 - (RW) Enable / Disable Register.
  - Bit 0 - LED control bit so CPU can control the 8x LED's.
  - Bit 1 - ROM disable bit so CPU can use the full SRAM address space.
  - Bit 2 - Reset enable bit to reset the CPU.
  - Bit 3 - SD Card enable bit.
  - Bit 4 - SD Card auto-init enable bit.
* 0x04 = (R)  Status Register.
  - Bit 0 - CPU Warm boot status (1 = CPU was reset, not FPGA).
* 0x05 = (RW) Interrupt Register (clears bits on a write if write bit = 1).
  - Bit 0 - SD Detect Interrupt.
  - Bit 1 - SD Error Interrupt.
* 0x06 = (RW) INT0 Interrupt Enable Register (0x00 on reset).
  - Bit 0 - SD Detect Interrupt Enable.
  - Bit 1 - SD Error Interrupt Enable.
* 0x10 - (R)  SD card status register.
  - Bit 0 - Busy
  - Bit 1 - Error
  - Bit 2-3 - Card Type 0=No card, 1=V1, 2=V2, 3=SDHC
  - Bit 4-7 
      - 0 = No error.
      - 1 = R1 error.
      - 2 = CRC error or Write timeout,.
      - 3 = Data Response Token error.
      - 4 = Data Error Token error
      - 5 = Write Protect error.
      - 6 = Unusable SD Card.
      - 7 = No card, i.e. no response to CMD0.
      - 8 = Read timeout.
* 0x11 - (W)  SD card command register
    - Bit 0 - Run card initialisation sequence (auto clear).
    - Bit 1 - Read a block from the sd card to the read buffer (auto clear).
    - Bit 2 - Write a block from the write buffer to the SD card (auto clear).
* 0x12 - (R)  SD card state machine state register (for decoding errors).
* 0x13 - (RW) SD Card block address byte 0.
* 0x14 - (RW) SD Card block address byte 1.
* 0x15 - (RW) SD Card block address byte 2.
* 0x16 - (RW) SD Card block address byte 3.

