// ****************************************************************************
// * 
// *    Code by Denno Wiggle aka WTM.
// *
// *    Copyright (C) 2023 Denno Wiggle
// *
// *    Z80 Bus Interface Module
// *    -- Copies the ROM from SPI FLASH to SRAM memory.
// *       Max size is 65536 bytes.
// *
// *    -- Provides FPGA registers accessible using IO address 0x50 (address) 
// *       and 0x51 (data). Registers are :
// *
// *       * 0x00 = (R)  Version register.
// *
// *       * 0x02 = (RW) LED Control Register.
// *
// *       * 0x03 - (RW) Enable / Disable Register.
// *           Bit 0 - LED control bit so CPU can control the 8x LED's.
// *           Bit 2 - Reset enable bit to reset the CPU.
// *
// *       * 0x04 = (R)  Status Register.
// *           Bit 0 - CPU Warm boot status (1 = CPU was reset, not FPGA).
// *
// *       * 0x05 = (RW) Interrupt Register (clears bits on a write if write bit = 1).
// *
// *       * 0x06 = (RW) INT0 Interrupt Enable Register (0x00 on reset).
// *
// * 
// * 
// ****************************************************************************
// * 
// *    This library is free software; you can redistribute it and/or
// *    modify it under the terms of the GNU Lesser General Public
// *    License as published by the Free Software Foundation; either
// *    version 2.1 of the License, or (at your option) any later version.
// *
// *    This library is distributed in the hope that it will be useful,
// *    but WITHOUT ANY WARRANTY; without even the implied warranty of
// *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// *    Lesser General Public License for more details.
// *
// *    You should have received a copy of the GNU Lesser General Public
// *    License along with this library; if not, write to the Free Software
// *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301
// *    USA
// *
// ****************************************************************************
// Don't allow inferred nets as that can cause debug issues
`default_nettype none

// Define wether to use 16-bit I/O addresses or not (8-bit)
// `define USE_16BIT_IO_ADRESS

module wtm_busInterface #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 20,
    parameter [3:0] MAJOR_VERSION = 'h0,
    parameter [3:0] MINOR_VERSION = 'h0
)(
    // Reset is held low for 3ms after sending an 18.36MHz clock to the CPU.
    // Scope confirms that the CPU PHI clock (the clock that drives this module) is
    // output from the CPU for 3ms before reset is de-asserted. So we can use a 
    // synchronous reset in the logic of this module.

    // CPU clock
    input  wire clock,
    // CPU reset
    input  wire reset_n,

    // CPU address and data bus
    input  wire [ADDR_WIDTH-1:0] cpu_address,
    output reg  [DATA_WIDTH-1:0] cpu_data_out,
    input  wire [DATA_WIDTH-1:0] cpu_data_in,
    output reg                   cpu_data_oe,

    // When asserted the CPU requests a memory space transaction
    input  wire         cpu_mreq_n,
    // When asserted the CPU requests an IO space transaction
    input  wire         cpu_iorq_n,
    // CPU bus control signals
    input  wire         cpu_m1_n,
    input  wire         cpu_wr_n,
    input  wire         cpu_rd_n,

    // Wait tells the CPU that we need more time.
    output wire         cpu_wait_n,

    // SRAM control pins. We have to drive these based on the CPU RW signals and
    // the address present on the bus
    output wire         mem_ce_n,
    output wire         mem_we_n,
    output wire         mem_oe_n,

    // Signals used for writing from SPI FLASH to SRAM memory
    input wire [19:0]   boot_address,
    input wire  [7:0]   boot_data,
    input wire          boot_ce_n,
    input wire          boot_we_n,
    // The FLASH to RAM write sequence is complete if this is asserted
    input wire          rom_load_done,

    // Signals that attach to FPGA registers.
    output reg  [7:0]   led_register,
    output reg          led_cpu_en,
    input  wire         status_cpu_wboot,

    // An IRQ vector signal when an interrupt condition is detected.
    // Each bit is a vector signal, i.e. lasts one clock pulse.
    input  wire [7:0]   irq_vector,
    
    // if any bit of the irq register is non-zero an interrupt is generated on 
    // cpu_int0_n if the corresponding bits in the irq enable register are set.
    output wire         cpu_int0_n,

    // Signal to reset the CPU via a register write which is 
    // cleared during external FPGA reset but not with local
    // reset signal reset_n
    output reg          reset_cpu_logic,

    // 8 bit debug port for board bringup and test.
    // This makes it easy to attach a scope and monitor internal FPGA signals as well
    // as duplicate CPU signals that have no easy scope attachment point.
    // WTM Todo : consider removing when no longer needed.
    output wire  [7:0]  debug_port

);


    // Check for an edge change on the sd_detect_n signal caused by insertion or removal
    // of the SD card.


    // FPGA Registers internal to this module
    reg  [7:0] fpga_reg_addr;
    reg  [7:0] version_register;
    reg  [7:0] irq_register;
    reg  [7:0] irq_en_register;
    

    // I/O address, data, and control signals internal to module.
    // On the Z8S180 I/O requests use 16 bits of the addreess bus
    wire [15:0] cpu_iorq_addr;  
    reg         cpu_iorq_r;
    reg         cpu_iorq_w;
    reg  [7:0]  fpga_data_out;
    reg         fpga_cs_r;


    // Process to detect a Z80 I/O request
    always @(*)
    begin
        if (reset_n == 1'b0) begin
            cpu_iorq_r <= 1'b0;
            cpu_iorq_w <= 1'b0;
        end
        else begin
            cpu_iorq_r <=  ~cpu_iorq_n & cpu_m1_n &  ~cpu_rd_n;
            cpu_iorq_w <=  ~cpu_iorq_n & cpu_m1_n &  ~cpu_wr_n;
        end
    end
    

    //
    // Provide access for the CPU to read the internal FPGA registers.
    //
    // Uncomment/Comment one of the following two lines if you want a clocked architecture or not.
    // always @*
    // The above reports combinatorial loops in nextpnr. Errors unless you use the --ignore-loops or --force option for nextpnr.
    // As we receive the CPU Phi clock and drive logic of it, this process can be synchronous
    always @(posedge clock)
    begin
        if (reset_n == 1'b0) 
        begin
            fpga_cs_r               <= 1'b0;
            fpga_data_out           <= 8'h00;
        end else 
        begin
            // assign a default value that can be over-ridden
            fpga_cs_r               <= 1'b0;
            fpga_data_out           <= fpga_data_out;

            if(cpu_iorq_r == 1'b1) 
            begin

`ifdef USE_16BIT_IO_ADRESS
                case(cpu_iorq_addr[15:0])
`else
                case(cpu_iorq_addr[7:0])
`endif

                // Register address register
                16'h0050 : 
                    begin
                        fpga_data_out <= fpga_reg_addr;
                        fpga_cs_r     <= 1'b1;
                    end
                // Data registers
                16'h0051 : 
                    begin
                        case(fpga_reg_addr)
                        // 00 = Version register
                        8'h00 : 
                            begin
                                fpga_data_out <= version_register;
                                fpga_cs_r     <= 1'b1;
                            end
                        // 02 = LED Register
                        8'h02 : 
                            begin
                                fpga_data_out <= ~led_register;
                                fpga_cs_r     <= 1'b1;
                            end
                        // 03 = Enable / Disable Register
                        8'h03 : 
                            begin
                                fpga_data_out <= {5'b00000, reset_cpu_logic, 1'b0, led_cpu_en};
                                fpga_cs_r     <= 1'b1;
                            end
                        // 04 = Status Register
                        8'h04 : 
                            begin
                                fpga_data_out <= {7'b0000000, status_cpu_wboot};
                                fpga_cs_r     <= 1'b1;
                            end
                        // 05 - Interrupt register
                        8'h05 : 
                            begin
                                fpga_data_out <= irq_register;
                                fpga_cs_r     <= 1'b1;
                            end
                        // 06 - Interrupt Enable register for interrupt output to INT0
                        8'h06 : 
                            begin
                                fpga_data_out <= irq_en_register;
                                fpga_cs_r     <= 1'b1;
                            end
                        // Catch all - output 0xFF
                        default : 
                            begin
                                fpga_data_out <= 8'hFF;
                                fpga_cs_r     <= 1'b1;
                            end
                        endcase
                    end

                default : 
                    begin
                    end

                endcase
                    
            end
        end
    end

    // Provide access for the Z80 to write some of the internal registers.
    // This uses a clocked process so that outputs can also be clocked elsewhere
    // in the code.
    always @(posedge clock) 
    begin
        if(reset_n == 1'b0) 
        begin
            fpga_reg_addr           <= 8'h00;
            version_register        <= {MAJOR_VERSION, MINOR_VERSION};
            led_register            <= 8'h0;
            led_cpu_en              <= 1'b0;
            reset_cpu_logic         <= 1'b0;
            irq_register            <= 8'h0;
            irq_en_register         <= 8'h00;   // No IRQ signals enabled at reset

        end else 
        begin
            // assign a default value that can be over-ridden
            fpga_reg_addr           <= fpga_reg_addr;
            version_register        <= version_register;
            led_register            <= led_register;
            led_cpu_en              <= led_cpu_en;
            reset_cpu_logic         <= reset_cpu_logic;
            irq_register            <= irq_register | irq_vector;
            irq_en_register         <= irq_en_register;

            if(cpu_iorq_w == 1'b1) 
            begin

`ifdef USE_16BIT_IO_ADRESS
                case(cpu_iorq_addr[15:0])
`else
                case(cpu_iorq_addr[7:0])
`endif

                // Register address register
                16'h0050 : 
                    begin
                        fpga_reg_addr               <= cpu_data_in;
                    end
                // Data registers
                16'h0051 : 
                    begin
                        case(fpga_reg_addr)
                        // 01 - Unused (was test register)
                        8'h01 : 
                            begin
                                ;
                            end
                        // 02 - LED control register
                        8'h02 : 
                            begin
                                led_register        <= ~cpu_data_in;
                            end
                        // 03 - Disable register
                        8'h03 : 
                            begin
                                led_cpu_en          <= cpu_data_in[0];
                                reset_cpu_logic     <= cpu_data_in[2];
                            end
                        // 05 - Interrupt register
                        8'h05 : 
                            begin
                                irq_register        <= irq_register & ~cpu_data_in;
                            end
                        // 06 - Interrupt Enable register for interrupt output to INT0
                        8'h06 : 
                            begin
                                irq_en_register     <= cpu_data_in;
                            end
                        // Catch all register addresses
                        default : 
                            begin
                            end
                        endcase
                    end
                    
                // Catch all I/O addresses
                default : 
                    begin
                    end
                endcase
            end 
        end
    end

    // Output an interrupt signal if there is active interrupt
    assign cpu_int0_n   = ( (irq_register & irq_en_register) == 8'h00) ? 1'b1 : 1'b0;

    // Place holder for the wait signal. Not currently implemented.
    // For SD Card we might want to assert WAIT until data is ready.
    // For fast CPU's (think eZ80) we may need to implement WAIT states
    // for register access.
    assign cpu_wait_n   = 1'b1;

    // On the Z8S180 I/O requests use 16 bits of the addreess bus
    assign cpu_iorq_addr = cpu_address[15:0];

    // Put ROM into SRAM memory at the start and then give the CPU control of 
    // the memory bus.
    // Gate the request by the rom_load_done setting and boot rom WE_N and CE_N.
    // Put FPGA registers on the bus when requested.
    wire   rom_req;
    assign rom_req = ~(boot_ce_n && boot_we_n || rom_load_done);

    // Process to control the CPU data bus output signal based on the internal request signals. 
    always @(*)
    begin
        // Default values
        cpu_data_out <= 8'hFF;
        cpu_data_oe <= 1'b0;

        case({2'b0, rom_req})
        // The ROM is being requested
        3'b001 : 
            begin
                cpu_data_out <= boot_data;
                cpu_data_oe  <= 1'b1;
            end
        // Catch all other cases
        default : 
            begin
                cpu_data_out <= fpga_data_out;
                cpu_data_oe  <= fpga_cs_r;
            end
        endcase
    end 

    // Define the memory control signals for access to the external SRAM memory.
    assign mem_we_n = rom_load_done ? cpu_wr_n : boot_we_n;
    assign mem_oe_n = cpu_rd_n;
    assign mem_ce_n = rom_load_done ? (cpu_wr_n && cpu_rd_n) : boot_ce_n;

    // 8 bit debug port for board bringup and test.
    assign debug_port = {1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0};

    
endmodule