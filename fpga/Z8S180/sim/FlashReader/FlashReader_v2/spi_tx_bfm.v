// ****************************************************************************
// * 
// *    Code by Denno Wiggle aka WTM.
// *
// *    Copyright (C) 2024 Denno Wiggle
// *
// *    SPI TX Bus Functional Model (BFM). 
// *    -- Takes a byte and sends it in SPI Mode 3 format.
// *    -- Signal timing and SPI timing checks specified for a AT45DB161D 
// *       SPI flash memory chip.
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

module spi_tx_bfm (
    // When tValid is asserted for one clock cycle acept that data to 
    // transmit on the SPI interface.
    input  wire  [7:0]  spi_tx_tData,
    input  wire         spi_tx_tValid,

    // The SPI Interface to test
    input  wire         spi_cs,
    input  wire         spi_clk,
    output wire         spi_miso
  ); 

    // Internal version of the byte to send.
    reg   [7:0] send_byte;
    // Bit position in the byte to send.
    integer     bit_num;

    // Error flag that will be asserted if there is a timing violation.
    reg         error_flag = 1'b0;

    // Internal version of the SPI MISO signal
    reg         spi_miso_i;
    assign      spi_miso = spi_miso_i;


    // Timing checks
    // https://www.hdlworks.com/hdl_corner/verilog_ref/items/SystemTimingChecks.htm
    // $setup(data_event, reference_event, limit [ , notifier ] );
    // $hold(reference_event, data_event, limit [ , notifier ] );
    // $setuphold(reference_event, data_event, setup_limit, hold_limit [ , notifier ] 
    //     [, tstamp_cond ] [ , tcheck_cond ] [ , delayed_ref ] [ , delayed_data ] );
    // $period(reference_event, limit [ , notifier ] );
    // $width(reference_event, limit [ , treshold [ , notifier ]] );
    // $skew(reference_event, data_event, limit [ , notifier ] );
    // $recovery(reference_event, data_event, limit [ , notifier ] );
    // $nochange(reference_event, data_event, start_edge_offset, end_edge_offset [ , notifier ] ); 

    specify 
        // AT45DB161D - SPI Mode 3 timing.
        specparam t_setup_in    =  2.0;  // MISO to SPI_CLK valid setup time.
        specparam t_hold_in     =  3.0;  // SPI_CLK to MISO valid hold time.
        specparam t_cs_setup    =  5.0;  // SPI_CS setup time until SPI_CLK falling edge.
        specparam t_cs_hold     =  5.0;  // SPI_CS hold time after SPI_CLK rising edge.
        specparam t_cs_minWidth = 20.0;  // Minimum SPI_CS pulse width.
        // specparam t_cs_minWidth = 3000.0;  // Minimum SPI_CS pulse width. 
        specparam t_hold_out    =  0.0;  // Output hold time after falling edge of clock.
        specparam t_valid       =  6.0;  // Negative edge of SPI_CLK to Output valid time.
        specparam t_dis         = 35.0;  // Delay from CS high until data tri-state

        // MISO timing checks.
        $setup(spi_miso, posedge spi_clk, t_setup_in, error_flag);
        $hold(posedge spi_clk, spi_miso, t_hold_in, error_flag);

        // CS timing checks.
        $setup(negedge spi_cs, negedge spi_clk, t_cs_setup, error_flag);
        $hold(posedge spi_clk, posedge spi_cs, t_cs_hold, error_flag);
        $width(negedge spi_cs, t_cs_minWidth, 0, error_flag);
    endspecify

    // Stop the simulation if there is a timing error.
    always @(posedge error_flag)
    begin
        $display("\n%t SPI timing error. Stopping simulation.\n", $time);
        $finish;
    end


    // When CS goes high the MISO slave output port will go high impedance.
    always @(posedge spi_cs)
    begin
        # t_dis;
        spi_miso_i = 1'bZ;
    end

    // Process block to send data over SPI.
    always
    begin : BFM_SPI_FLASH_TX

        // Wait until the rising edge of the valid signal which indicates
        // the data byte is ready to send.
        @(posedge spi_tx_tValid);

        send_byte  = spi_tx_tData;
        bit_num    = 7;

        repeat (8)
        begin
            @(negedge spi_clk);
            spi_miso_i <= 1'bX;

            # t_valid;

            spi_miso_i = send_byte[bit_num];
            bit_num = bit_num - 1;

            # (t_valid - t_hold_out);

            @(posedge spi_clk);
            # t_hold_in;
        end
        spi_miso_i <= 1'bX;
    end


    initial begin
        // Set time format to ns.
        $timeformat(-9, 0, " ns :", 14);
    end



endmodule

