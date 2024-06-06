// ****************************************************************************
// * 
// *    Code by Denno Wiggle aka WTM.
// *
// *    Copyright (C) 2024 Denno Wiggle
// *
// *    Test bench to test reading the SPI FLASH, transmitting the data
// *    over UART, and copying the data to memory.
// *    -- This is to simulate loading a copy of the system software from 
// *       FLASH (ROM) into memory for a CPU to boot.
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
`timescale 1ns/100ps 

// Don't allow inferred nets as that can cause debug issues
`default_nettype none

// Uncommenting this gives verbose printing to help debug issues with design.
// `define TB_DEBUG

module flashReaderToMem_tb ();

    localparam  LOGIC_CLOCK_FREQ_HZ     = 25000000;
    localparam  CLOCK_PERIOD_NS         = 1000000000 / LOGIC_CLOCK_FREQ_HZ;

    localparam  FLASH_SPI_CLOCK_HZ      = 12500000;

    localparam  FLASH_SW_START_ADDRESS  = 135100;  // 0x020fbc
    localparam  FLASH_SW_NUM_BYTES      = 16;

    localparam  MEM_SIZE_BYTES          = 65536;
    localparam  MEM_START_ADDRESS       = 0;

    // Clock and reset
    reg         clock               = 1'b1;
    reg         reset_n             = 1'b0;

    // Simulated SRAM memory. 
    // Data read from FLASH is stored in SRAM.
    wire [19:0] mem_address;
    wire  [7:0] mem_data;
    wire        mem_ce_n;
    wire        mem_we_n;
    reg   [7:0] sram_memory [MEM_SIZE_BYTES - 1:0];

    // SPI Signals
    wire        spi_cs;
    wire        spi_clk;
    wire        spi_mosi;
    wire        spi_miso;

    // Flash reader data signals.
    wire        flash_read_en;
    wire        flash_read_active;
    wire  [7:0] flash_read_tData;
    wire        flash_read_tValid;

    // ROM load done flag.
    // When data read from Flash (ROM) has completed writing to 
    // simulated memory, the done flag is asserted.
    wire        rom_load_done;

    // The data valid flag is asserted when the test bench wants to send
    // data over the SPI BFM.
    reg [7:0]   bfm_spi_tx_tData;
    reg         bfm_spi_tx_tValid   = 1'b0;

    // The is a copy of 'flash_read_tData', the data read from the FLASH
    // that is checked against the data that the test bench sent over the SPI BFM.
    reg   [7:0] read_data;

    // This used to check the simulated memory, against the data that the test 
    // bench sent over the SPI BFM.
    reg   [7:0] ram_check_byte;

    // Internal test bench process control signals.
    reg         start_reader_test   = 1'b0;
    reg         spi_tx_done         = 1'b0;
    reg         ram_read_done       = 1'b0;

    // Having the same seed values for tx and rx bytes
    // allows us to transmit a known sequence and then later
    // check the received sequence against the same random bytes
    // without have to store the bytes in the test bench or file.
    reg [31:0]  spi_send_seed       = 32'h13427465;
    reg [31:0]  ram_check_seed      = 32'h13427465;

    // Keep track of the number of bytes that are sent and received
    // and which RAM address to read next.
    integer spi_send_count;
    integer ram_read_address;

    // Generate the system clock
    always
    begin
        # (CLOCK_PERIOD_NS / 2)
        clock <= ~clock;
    end

    // Device Under Test - the top module is a unit that can be 
    // tested on real hardware. 
    // At startup load the SRAM memory with the SW contents of the SPI FLASH.
    // This is to perform a ROM load to RAM before the CPU is enabled.
    wtm_flashToMemory #(
        .FLASH_LOGIC_CLOCK_HZ   (LOGIC_CLOCK_FREQ_HZ),
        .FLASH_SPI_CLOCK_HZ     (FLASH_SPI_CLOCK_HZ),
        .FLASH_SW_START_ADDRESS (FLASH_SW_START_ADDRESS),
        .FLASH_SW_NUM_BYTES     (FLASH_SW_NUM_BYTES),
        .MEM_START_ADDRESS      (0)
    ) DUT (
        // Clock and reset
        .clock                  (clock),
        .reset_n                (reset_n),

        // SRAM Memory address, data, and control ports. 
        .mem_address            (mem_address),
        .mem_data               (mem_data),
        .mem_ce_n               (mem_ce_n),
        .mem_we_n               (mem_we_n),

        // The FLASH to RAM write sequence is complete
        .rom_load_done          (rom_load_done),

        // SPI signals
        .spi_clk                (spi_clk),
        .spi_cs                 (spi_cs),
        .spi_mosi               (spi_mosi),
        .spi_miso               (spi_miso),

        // Control signals to read the flash
        .flash_read_en          (flash_read_en),
        .flash_read_active      (flash_read_active),

        // Byte holding the data read from the FLASH.
        // The data valid signal has a duration of one clock cycle.
        .flash_tData            (flash_read_tData),
        .flash_tValid           (flash_read_tValid)
    );

    // Bus Functional Model of the SPI TX that is used to send data to the 
    // DUT. The timing of this module uses FLASH part datasheet timing, and 
    // has internal checks for signal timing requirements. 
    spi_tx_bfm bfm_spi_flash(
        .spi_tx_tData           (bfm_spi_tx_tData),
        .spi_tx_tValid          (bfm_spi_tx_tValid),
        .spi_cs                 (spi_cs),
        .spi_clk                (spi_clk),
        .spi_miso               (spi_miso)
    );


   //
    // SRAM memory timing
    //
    // Error flag that will be asserted if there is a timing violation.
    reg error_flag = 1'b0;

    specify 
        // IS61WV10248EDBLL-10 timing parameters taken from the datasheet.
        specparam t_writeCycle              =  10;  // t_wc  - write cycle time. 
        specparam t_ce_toWriteEnd           =   8;  // t_sce - chip enable to write end.
        specparam t_we_pulseWidth           =  10;  // t_pwe - write enable pulse width.
        specparam t_dataSetup_ToWriteEnd    =   6;  // t_sd  - data setup to write end.
        specparam t_dataHold_FromWriteEnd   =   0;  // t_hd  - data hold to write end.
        specparam t_addSetup                =   8;  // t_sa  - address setup.
        specparam t_addSetup_ToWriteEnd     =   8;  // t_aw  - address setup to write end.
        specparam t_addHold_FromWriteEnd    =   0;  // t_ha  - address hold to write end.

        // CE timing check
        $setup(negedge mem_ce_n, posedge mem_we_n, t_ce_toWriteEnd, error_flag);

        // WE timing check
        $width(negedge mem_we_n, t_we_pulseWidth, 0.5, error_flag);

        // Data timing checks.
        $setup(mem_data, posedge mem_we_n, t_dataSetup_ToWriteEnd, error_flag);
        $hold(posedge mem_we_n, mem_data, t_dataHold_FromWriteEnd, error_flag);

        // Address timing checks.
        $setup(mem_address, negedge mem_we_n, t_addSetup, error_flag);
        $setup(mem_address, negedge mem_ce_n, t_addSetup, error_flag);
        $setup(mem_address, posedge mem_we_n, t_addSetup_ToWriteEnd, error_flag);
        $hold(posedge mem_we_n, mem_address, t_addHold_FromWriteEnd, error_flag);
    endspecify

    // Stop the simulation if there is a timing error.
    always @(posedge error_flag)
    begin
        $display("\n%t Memory timing error. Stopping simulation.\n", $time);
        $finish;
    end

    //
    // Simple model of SRAM memory. This is for waveform viewing purposes only.
    //
    always @(*)
    begin : SRAM_MEMORY_WRITE
        if (mem_ce_n == 1'b0 && mem_we_n == 1'b0)
        begin
            sram_memory[mem_address] <= mem_data;
        end
    end

    //
    // Process block to send the test transmit data over the SPI TX BFM.
    //
    always
    begin : SPI_TRANSMIT_DATA

        spi_send_count          = 0;

        // Wait until the test start signal has been sent.
        // Wait if the data has already been transmitted.
        while ((start_reader_test == 1'b0) || spi_tx_done)
            @(posedge clock);

        // Wait until data reading inside the DUT becomes active.
        while (flash_read_active == 1'b0)
            @(posedge clock);

        // Loop and send the specified number of data bytes to the BFM.
        // Check the data read back by the module matches.
        repeat (FLASH_SW_NUM_BYTES) 
        begin
            spi_send_count      = spi_send_count + 1;

            // Prepare the data and assert the valid signal for one clock cycle.
            bfm_spi_tx_tData    = {$random(spi_send_seed)} %255;
            bfm_spi_tx_tValid   = 1'b1;

`ifdef TB_DEBUG
            $display("%t       - SPI sending byte no.    %2d = %b", $time, spi_send_count, bfm_spi_tx_tData);
`endif

            @(negedge clock);
            @(posedge clock);
            bfm_spi_tx_tValid          = 1'b0;
            @(negedge clock);

            // Wait for valid data to read back from the module.
            while (flash_read_tValid == 1'b0)
                @(posedge clock);

            read_data = flash_read_tData;

`ifdef TB_DEBUG
            $display("%t          - SPI sent byte no.    %2d = %b", 
                    $time, spi_send_count, read_data);
`endif

            if (read_data !== bfm_spi_tx_tData)
            begin
                $display("%t Error - FlashReader received byte no. %0d = %b doesn't match sent byte %b.\n", 
                        $time, spi_send_count, read_data, bfm_spi_tx_tData);
                @(posedge clock);
                $finish;
            end

        end
        $display("%t     SPI sent %0d bytes.", $time, spi_send_count);

        // Set the TX complete flag
        spi_tx_done             = 1'b1;
        @(posedge clock);
    end

    //
    // Process block to check the data in simulated RAM once RAM writes have completed. 
    //
    always
    begin : CHECK_RAM_DATA

        ram_read_address        = MEM_START_ADDRESS;

        // Wait until the test start signal has been sent and the load of ROM to RAM is complete.
        // Wait if all the data has been read and checked.
        while ((start_reader_test == 1'b0) || ~rom_load_done || ram_read_done)
            @(posedge clock);

        // Loop throught the memory and check the specified number of data bytes.
        repeat (FLASH_SW_NUM_BYTES) 
        begin
            ram_check_byte      = {$random(ram_check_seed)} %255;

            @(posedge clock);

`ifdef TB_DEBUG
            $display("%t       - RAM memory byte no. %0d = %b, sent byte %b.", 
                    $time, ram_read_address, sram_memory[ram_read_address], ram_check_byte);
`endif
            if (sram_memory[ram_read_address] !== ram_check_byte)
            begin
                $display("%t Error - RAM memory byte no. %0d = %02x doesn't match sent byte %02x.\n", 
                        $time, ram_read_address, sram_memory[ram_read_address], ram_check_byte);
                @(posedge clock);
                $finish;
            end

            ram_read_address    = ram_read_address + 1;

        end
        $display("%t     Read %0d RAM bytes OK.", $time, ram_read_address - MEM_START_ADDRESS);

        // Set the RAM read complete flag
        ram_read_done        = 1'b1;
        @(posedge clock);
    end

    //
    // The main test bench test flow.
    //
    initial 
    begin
        // Set the test bench flags to zero.
        start_reader_test       = 1'b0;
        spi_tx_done             = 1'b0;

        $dumpfile("flashReaderToMem_tb.vcd");
        $dumpvars;

        // Set the time format to ns.
        $timeformat(-9, 0, " ns :", 14);

        $display("\n################################################");
        $display("###  Flash Reader + Mem + Uart Test Bench   ####");
        $display("################################################");

        // Hold reset low for ten clock periods
        reset_n            = 1'b0;
        # (CLOCK_PERIOD_NS * 10);

        $display("\n%t Releasing reset.\n", $time);
        reset_n            = 1'b1;
        # (CLOCK_PERIOD_NS * 10);

        $display("%t Starting TX and RX processes.", $time);
        start_reader_test       = 1'b1;

        $display("%t Waiting for %0d bytes to be sent and %0d bytes to be received.", $time, FLASH_SW_NUM_BYTES, FLASH_SW_NUM_BYTES);
        while ( ~(spi_tx_done && ram_read_done) )
            @(posedge clock);

        $display("\n%t All bytes received.", $time);
        $display("%t Test : OK\n", $time);
        
        // Add some extra clocks so signals at the end can be clearly found.
        repeat (100)
            @(posedge clock); 
         
        $finish;
    end

endmodule