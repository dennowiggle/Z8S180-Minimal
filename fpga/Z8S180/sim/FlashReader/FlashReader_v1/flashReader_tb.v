// ****************************************************************************
// * 
// *    Code by Denno Wiggle aka WTM.
// *
// *    Copyright (C) 2024 Denno Wiggle
// *
// *    Test bench to test reading the SPI FLASH.
// *    -- 
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

module flashReader_tb ();

    localparam  CLOCK_FREQ_HZ           = 25000000;
    localparam  CLOCK_PERIOD_NS         = 1000000000 / CLOCK_FREQ_HZ;

    localparam  FLASH_SPI_CLOCK_HZ      = 12500000;

    localparam  FLASH_SW_START_ADDRESS  = 135100;  // 0x020fbc
    localparam  FLASH_SW_NUM_BYTES      = 16;

    // Clock and reset
    reg         clock               = 1'b1;
    reg         fpga_reset_n        = 1'b0;
    wire        reset_n;

    // SPI Signals
    wire        spi_cs_n;
    wire        spi_clk;
    wire        spi_mosi;
    wire        spi_miso;

    // Flash reader data signals.
    reg         flash_read_en;
    wire        flash_read_active;
    wire  [7:0] flash_read_tData;
    wire        flash_read_tValid;

    // The data valid flag is asserted when the test bench wants to send
    // data over the SPI BFM.
    reg [7:0]   bfm_spi_tx_tData;
    reg         bfm_spi_tx_tValid   = 1'b0;

    // The is a copy of 'flash_read_tData', the data read from the FLASH 
    // that is checked against the data that the test bench sent over the SPI BFM.
    reg   [7:0] read_data;

    // Internal test bench process control signals.
    reg         start_reader_test   = 1'b0;
    reg         reader_txrx_done    = 1'b0;

    // Having the same seed values for tx and rx bytes
    // allows us to transmit a known sequence and then later
    // check the received sequence against the same random bytes
    // without have to store the bytes in the test bench or file.
    reg [31:0]  spi_send_seed           = 32'h13427465;

    // Keep track of the number of bytes that are sent and received.
    integer spi_send_count;

    // Generate the system clock
    always
    begin
        # (CLOCK_PERIOD_NS / 2)
        clock <= ~clock;
    end

    // The module under test only has one reset.
    assign reset_n = fpga_reset_n;

    // Device under test
    wtm_flashReader #(
        .CLOCK_FREQ_HZ      (CLOCK_FREQ_HZ),
        .SPI_CLOCK_HZ       (FLASH_SPI_CLOCK_HZ),

        // Flash data parameters
        .START_ADDRESS      (FLASH_SW_START_ADDRESS),
        .NUM_READ_BYTES     (FLASH_SW_NUM_BYTES)
    ) DUT (
        // Clock and reset
        .clock              (clock),
        .reset_n            (reset_n),

        // SPI signals
        .spi_clk            (spi_clk),
        .spi_cs_n           (spi_cs_n),
        .spi_mosi           (spi_mosi),
        .spi_miso           (spi_miso),

        // Control and data signals involved with reading the flash
        .flash_read_en      (flash_read_en),
        .flash_read_active  (flash_read_active),

        .flash_tValid       (flash_read_tValid),
        .flash_tData        (flash_read_tData)
    );

    // Bus Functional Model of the SPI TX that is used to send data to the 
    // DUT. The timing of this module uses FLASH part datasheet timing, and 
    // has internal checks for signal timing requirements. 
    spi_tx_bfm bfm_spi_flash(
        .spi_tx_tData       (bfm_spi_tx_tData),
        .spi_tx_tValid      (bfm_spi_tx_tValid),
        .spi_cs_n           (spi_cs_n),
        .spi_clk            (spi_clk),
        .spi_miso           (spi_miso)
    );

    //
    // Process block to send the transmit data from the SPI BFM
    // and check that the module under test reads the data correctly.
    //
    always
    begin : TX_RX_DATA

        spi_send_count              = 0;

        // Wait until the test start signal has been sent.
        // Wait if the data has already been transmitted.
        while ((start_reader_test == 1'b0) || reader_txrx_done)
            @(posedge clock);

        // Send the signal to start reading from the FLASH.
        flash_read_en              = 1'b1;

        // Wait until data reading becomes active.
        while (flash_read_active == 1'b0)
            @(posedge clock);

        // Loop and send the specified number of data bytes to the BFM.
        // Check the data read back by the module matches.
        repeat (FLASH_SW_NUM_BYTES) 
        begin
            spi_send_count = spi_send_count + 1;

            // Prepare the data and assert the valid signal for one clock cycle.
            bfm_spi_tx_tData           = {$random(spi_send_seed)} %255;
            bfm_spi_tx_tValid          = 1'b1;

`ifdef TB_DEBUG
            $display("%t       - SPI sending byte no.    %2d = %b", $time, spi_send_count, bfm_spi_tx_tData);
`endif

            @(negedge clock);
            @(posedge clock);
            bfm_spi_tx_tValid   = 1'b0;
            @(negedge clock);

            // Wait for valid data to read back from the module.
            while (flash_read_tValid == 1'b0)
                @(posedge clock);

            read_data = flash_read_tData;

            // Wait a clock to see if flash_read_active has gone low
            @(posedge clock);

            if (flash_read_active == 1'b0)
                flash_read_en      = 1'b0;

`ifdef TB_DEBUG
            $display("%t       - Receiving byte no.  %2d = %b", $time, spi_send_count, read_data);
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

        // Set the TX & RX complete flag
        reader_txrx_done        = 1'b1;
        @(posedge clock);
        @(posedge clock);
    end


    //
    // The main test bench test flow.
    //
    initial 
    begin
        // Set the test bench flags to zero.
        start_reader_test     = 1'b0;
        reader_txrx_done      = 1'b0;

        $dumpfile("flashReader_tb.vcd");
        $dumpvars;

        // Set the time format to ns.
        $timeformat(-9, 0, " ns :", 14);

        $display("\n################################################");
        $display("########   Flash Reader Test Bench   ###########");
        $display("################################################");

        // Hold reset low for ten clock periods
        fpga_reset_n            = 1'b0;
        # (CLOCK_PERIOD_NS * 10);

        $display("\n%t Releasing reset.\n", $time);
        fpga_reset_n            = 1'b1;
        # (CLOCK_PERIOD_NS * 10);

        $display("%t Waiting for the DUT to release the reset.", $time);
        while (reset_n == 1'b0)
            @(posedge clock);

        $display("%t Starting TX and RX processes.", $time);
        start_reader_test       = 1'b1;

        $display("%t Waiting for %0d bytes to be sent and %0d bytes to be received.", $time, FLASH_SW_NUM_BYTES, FLASH_SW_NUM_BYTES);
        while (~reader_txrx_done)
            @(posedge clock);

        $display("\n%t All bytes received.", $time);
        $display("%t Test : OK\n", $time);
        
        // Add some extra clocks so signals at the end can be clearly found.
        repeat (100)
            @(posedge clock); 
         
        $finish;
    end

endmodule