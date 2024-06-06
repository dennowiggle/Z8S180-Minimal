// ****************************************************************************
// * 
// *    Code by Denno Wiggle aka WTM. 
// *    
// *    Module will read data from the FPGA configuartion SPI flash memory,
// *    and write the contents to external SRAM memory.
// *
// *    Copyright (C) 2023 Denno Wiggle
// *
// *    Module to read the FPGA configuration SPI FLASH and write to SRAM memory.
// *    -- Read data from the SPI flash memory.
// *    -- Write the data to CPU SRAM memory.
// *    -- Used to load ROM SW into RAM for CPU start-up.
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

// Don't allow inferred nets as that can cause debug issues.
`default_nettype none

// Defines a SPI FLASH compatible with 528 byte page mode.
// Define is set in Maekfile if needed.
// `define PAGE_BYTES_528

module wtm_flashToMemory #(
    parameter FLASH_LOGIC_CLOCK_HZ      = 25000000,
    parameter FLASH_SPI_CLOCK_HZ        =   100000,
    parameter FLASH_SW_START_ADDRESS    =   135100,
    parameter FLASH_SW_NUM_BYTES        =    65536,
    parameter MEM_START_ADDRESS         =        0
)(
    // Clock and reset
    input  wire         clock,
    input  wire         reset_n,

    // SRAM Memory address, data, and control pins. 
    output reg  [19:0]  mem_address,
    output reg   [7:0]  mem_data,
    output reg          mem_ce_n,
    output reg          mem_we_n,

    // The FLASH to RAM write sequence is complete
    output wire         rom_load_done,

    // SPI signals
    output wire         spi_clk,
    output wire         spi_cs,
    output wire         spi_mosi,
    input  wire         spi_miso

`ifdef TEST_BENCH   // Signals brought out for test purposes
    ,
    // Control signals to read the flash
    output reg          flash_read_en,
    output wire         flash_read_active,

    // Byte holding the data read from the FLASH.
    // The data valid signal has a duration of one clock cycle.
    output wire  [7:0]  flash_tData,
    output wire         flash_tValid

`endif

);

// If using a SPI FLASH compatible with 528 byte page mode.
// then the address is calculated differently than with 512 byte mode.
`ifdef PAGE_BYTES_528

    localparam PAGE_ADDRESS = FLASH_SW_START_ADDRESS / 528;
    localparam BYTE_ADDRESS = FLASH_SW_START_ADDRESS - PAGE_ADDRESS * 528;
    // Page address is defined with 12 bits of address[21:10]
    // Byte address within page is defined with 10 bits of address[9:0]
    localparam FLASH_START_ADDRESS = (PAGE_ADDRESS << 10) + BYTE_ADDRESS;

`else 
    localparam FLASH_START_ADDRESS = FLASH_SW_START_ADDRESS;
`endif 

`ifndef TEST_BENCH

    // Control signals to read the flash
    reg          flash_read_en;
    wire         flash_read_active;

    // Byte holding the data read from the FLASH.
    // The data valid signal has a duration of one clock cycle.
    wire  [7:0]  flash_tData;
    wire         flash_tValid;

`endif

    // Used along with the 'flash_read_active` signal for detecting a state change.
    reg         flash_read_active_prev;
    // Signal indicates if reading of the FLASH has finished.
    reg         read_finished;


    // Module that reads the FLASH data
    wtm_flashReader #(
        .CLOCK_FREQ_HZ      (FLASH_LOGIC_CLOCK_HZ),
        .SPI_CLOCK_HZ       (FLASH_SPI_CLOCK_HZ),

        // Flash data parameters
        .START_ADDRESS      (FLASH_START_ADDRESS),
        .NUM_READ_BYTES     (FLASH_SW_NUM_BYTES)
    ) FlashReader_inst  (
        // Clock and reset
        .clock              (clock),
        .reset_n            (reset_n),

        // SPI signals
        .spi_clk            (spi_clk),
        .spi_cs             (spi_cs),
        .spi_mosi           (spi_mosi),
        .spi_miso           (spi_miso),

        // Controls the start and returns the status of reading the FLASH.
        .flash_read_en      (flash_read_en),
        .flash_read_active  (flash_read_active),

        // Byte holding the data read from the FLASH.
        // The data valid signal has a duration of one clock cycle.
        .flash_tValid        (flash_tValid),
        .flash_tData         (flash_tData)
    );


    // Copy FLASH data to SRAM memory
    always @(posedge clock)
    begin : FLASH_TO_MEM

        if (reset_n == 1'b0)
        begin
            mem_address     <= MEM_START_ADDRESS;
            mem_ce_n        <= 1'b1;
            mem_we_n        <= 1'b1;
        end else
        begin
            if (flash_tValid == 1'b1)
            begin
                mem_data    <= flash_tData;
                mem_ce_n    <= 1'b0;
                mem_we_n    <= 1'b0;
            end else
            begin
                mem_ce_n    <= 1'b1;
                mem_we_n    <= 1'b1;

                // mem_we_n is asserted for one clock only so we can update the address safely
                if (mem_we_n == 1'b0)
                    mem_address <= mem_address + 1;
            end
        end
    end


    // Process block to enable reading of the SPI flash and assert the finished 
    // signal upon completion. Completion is triggered by detection of a 
    // level change on the flash_read_active signal of 1 to 0.
    always @(posedge clock)
    begin : FLASH_READ_CONTROL

        if (reset_n == 1'b0)
        begin
            flash_read_en <= 1'b0;
            flash_read_active_prev <= 1'b0;
            read_finished <= 1'b0;
        end else
        begin
            flash_read_active_prev <= flash_read_active;

            if (read_finished == 1'b0)
                if (flash_read_active_prev == 1'b1 && flash_read_active == 1'b0)
                begin
                    read_finished <= 1'b1;
                    flash_read_en <= 1'b0;
                end else
                begin
                    flash_read_en <= 1'b1;
                end
        end
    end

    assign rom_load_done = read_finished;


endmodule
