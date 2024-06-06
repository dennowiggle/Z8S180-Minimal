// ****************************************************************************
// * 
// *    Code by Denno Wiggle aka WTM.
// *
// *    Copyright (C) 2024 Denno Wiggle
// *
// *    UART Bus Functional Model (BFM). 
// *    -- Converts parallel data to and from serial UART format with no parity
// *       and 1 stop bit.
// *    -- TX logic is passed a data word by the upper level module and the  
// *       BFM sends serial data on the TX output port.
// *    -- RX logic receives serial data on the RX port, shifts the data in, 
// *       and hands the data word to the upper level module. 
// *    -- The stop bit is checked for validity.
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

module uart_bfm #(
    parameter CLOCK_FREQ_HZ     = 25000000,
    parameter BAUD_RATE         =   115200,
    parameter NUM_BITS          =        8
  )(
    // Data to send on UART Tx
    input  wire  [NUM_BITS - 1:0]   tx_data,        // Incoming data to send of Uart Tx.
    input  wire                     tx_data_valid,  // Incoming data byte is valid when 1.
    output reg                      tx_data_ready,  // Ready to receive new data when 1.

    // Receive data from UART Rx
    output reg  [NUM_BITS - 1:0]    rx_data,        // Outgoing data.
    output reg                      rx_data_valid,  // Outgoing data is valid when 1.

    // UART interface
    output reg                      tx,
    input  wire                     rx
  );

    reg   [NUM_BITS - 1:0]          rx_data_in;
    reg   [NUM_BITS - 1:0]          tx_data_in;
    
    localparam CLOCK_PERIOD_NS  = 1000000000 / CLOCK_FREQ_HZ;
    localparam BIT_DURATION_NS  = 1000000000 / BAUD_RATE;

    // The delay seed is used to generate a random delay up to one bit duration 
    // before sending TX data on the UART line.
    reg [31:0]  delay_seed         = 32'h13427465;

    // Transmit logic.
    always
    begin : TRANSMIT_BITS

        // Set the initial state of the TX data ready signal to ready to accept data.
        // Initialise the TX signal to 1.
        tx_data_ready       = 1'b1;
        tx                  = 1'b1;
        
        // Wait for a data valid signal from the upper level module.
        while (tx_data_valid == 1'b0)
            # CLOCK_PERIOD_NS;

        // Data is valid so accept the incoming data and de-assert the ready for 
        // data signal.
        tx_data_in          = tx_data;
        tx_data_ready       = 1'b0;

        // Add in a random delay before sending the data oon the TX UART line..
        # ({$random(delay_seed)} %BIT_DURATION_NS)

        // Send the start bit which is a transition from high to low for one 
        // bit duration.
        tx                  = 1'b0;
        # BIT_DURATION_NS

        // Shift out each of the bits using a shift register and place on the 
        // serial TX bit port.
        repeat (NUM_BITS)
        begin
            tx              = tx_data_in[0];
            tx_data_in      = {1'b0, tx_data_in[NUM_BITS - 1:1]};
            # BIT_DURATION_NS;
        end

        // Send the stop bit.
        tx                  = 1'b1;
        # BIT_DURATION_NS;
    end

    // Receive logic
    always
    begin : RECEIVE_BITS

        // Set the initial state of data valid signal to not valid
        rx_data_valid           = 1'b0;

        // A falling edge on RX indicates a start bit. Wait until
        // we receive a start bit.
        @(negedge rx);

        // Wait for the duration of the start bit and half of the 
        // first data bit. We want to sample in the middle of the bit period. 
        # (BIT_DURATION_NS * 1.5)

        // Shift in the bits using a shift register.
        repeat(NUM_BITS)
        begin
            rx_data_in          = {rx, rx_data_in[NUM_BITS-1:1]};
            # BIT_DURATION_NS;
        end

        // The stop bit should be a high level signal.
        if (rx == 1'b0)
        begin
            $display("%t Error - The stop bit should be 1.\n", $time);
            # BIT_DURATION_NS;
            $finish;
        end

        // Send the data to the upper level module and assert the data 
        // valid signal for one clock period.
        rx_data                 = rx_data_in;
        rx_data_valid           = 1'b1;
        # CLOCK_PERIOD_NS
        rx_data_valid           = 1'b0;

    end

endmodule