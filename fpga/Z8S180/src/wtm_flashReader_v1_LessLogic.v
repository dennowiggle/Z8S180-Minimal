// ****************************************************************************
// * 
// *    Code by Denno Wiggle aka WTM. 
// *    
// *    Module will read data from the FPGA configuartion SPI flash memory.
// *
// *    Copyright (C) 2023 Denno Wiggle
// *
// *    Read data from the SPI flash memory.
// *    -- v1 has SPI clock, MISO, & MOSI in one process block.
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

module wtm_flashReader #(
    // System clock frequency and SPI clock frequency.
    parameter           CLOCK_FREQ_HZ   = 25000000, 
    parameter           SPI_CLOCK_HZ    =   100000,

    // Flash data parameters
    parameter           START_ADDRESS   = 135100, // 0x020fbc
    parameter           NUM_READ_BYTES  =  65336
)(
    // Clock and reset
    input  wire         clock,
    input  wire         reset_n,

    // SPI signals
    output wire         spi_clk,
    output reg          spi_cs_n,
    output reg          spi_mosi,
    input  wire         spi_miso,

    // Controls the start and returns the status of reading the FLASH.
    input  wire         flash_read_en,
    output reg          flash_read_active,

    // Byte holding the data read from the FLASH.
    // The data valid signal has a duration of one clock cycle.
    output reg   [7:0]  flash_tData,
    output reg          flash_tValid
);


    // SPI clock timing.
    // SPI clock half period must be a minimum of one system clock,
    localparam CLOCKS_PER_SPI_CLK_HALF   = ( (CLOCK_FREQ_HZ / SPI_CLOCK_HZ) / 2 ) > 0 ? 
                ((CLOCK_FREQ_HZ / SPI_CLOCK_HZ) / 2 ) : 1;
    localparam HALF_PERIOD_COUNTER_MAX   = CLOCKS_PER_SPI_CLK_HALF - 1;
    localparam HALF_PERIOD_COUNTER_WIDTH = $clog2(HALF_PERIOD_COUNTER_MAX + 1);
    reg  [HALF_PERIOD_COUNTER_WIDTH - 1:0] half_period_counter;

    // Counter parameters
    localparam SPI_BIT_COUNTER_MAX       = 8;
    localparam SPI_BIT_COUNTER_WIDTH     = $clog2(SPI_BIT_COUNTER_MAX + 1);
    reg  [SPI_BIT_COUNTER_WIDTH - 1:0]   spi_bit_counter;

    // Holds the count of the number of bytes that have been read.
    localparam BYTE_COUNTER_WIDTH = $clog2(NUM_READ_BYTES + 1);
    reg  [BYTE_COUNTER_WIDTH - 1:0] byte_counter;

    // FLASH commands that are used.
    localparam READ_COMMAND   = 8'h03;
    localparam WAKEUP_COMMAND = 8'hAB;

    // Command count sets the command byte number to transmit.
    reg  [3:0]  command_count;
    // 24-bit FLASH address where we want to read data.
    reg [23:0]  flash_address;

    // Used to signal a 35us timer to wait for completion of certain FLASH commands.
    reg         flash_wait_timer;
    wire        flash_wait_complete;

    // Flash read state machine
    reg [2:0] flash_read_state;
    localparam [2:0]
        FLASH_IDLE          = 0,
        FLASH_WAKE_UP       = 1,
        FLASH_WAIT          = 2,
        FLASH_READ_CMD      = 3,
        FLASH_READ_DATA     = 4,
        FLASH_STOP          = 5;

    // SPI state machine
    reg   [1:0] spi_state;  
    localparam [1:0]
        SPI_IDLE        = 1,
        SPI_PROCESSING  = 2;
    
    // Used to sync MISO to the clock domain and shift in MISO bits into a byte.
    reg  [7:0]  miso_shift_reg;
    // Holds the TX data to be sent out on the SPI transmit MOSI port.
    // The data valid signal has a duration of one clock cycle.
    reg  [7:0]  spi_tx_tData;
    reg         spi_tx_tValid;

    // Internal version of the SPI clock.
    reg         spi_clk_i = 1'b1;


    assign spi_clk = spi_clk_i;

    // 35us timer used to wait after certain flash commands like reset and wake up.
    wtm_resetSyncDelay #(35, CLOCK_FREQ_HZ)
    flash_wait_timer_inst(
        .clock(clock),
        .rst_n(flash_wait_timer),
        .rst_out_n(flash_wait_complete)
    );

    // 
    // SPI state machine specific for reading the FLASH data bytes.
    //
    always @(posedge clock) 
    begin : SPI_STATE_MACHINE
        if (reset_n == 1'b0) 
        begin
            spi_state           <= SPI_IDLE;

            spi_clk_i           <= 1'b1;
            spi_mosi            <= 1'b1;

            half_period_counter <= 0;
            spi_bit_counter     <= 0;

            miso_shift_reg      <= 8'hFF;
            flash_tData         <= 8'hFF;
            flash_tValid        <= 1'b0;

        end else 
        begin
            // Default value for pulsed data valid signal.
            flash_tValid         <= 1'b0;       

            case(spi_state)

            SPI_IDLE : // Move to the processing state if upper module has
                       // enabled reading and we are not in a wait state. 
            begin
                spi_mosi                    <= 1'b1;

                if (spi_tx_tValid == 1'b1) // Data valid signal is asserted as we have valid data to proceed
                    spi_state           <= SPI_PROCESSING;
            end

            SPI_PROCESSING : 
            begin
                spi_mosi <= spi_mosi;

                // Data changes on SPI clock edges which are controlled by the counter.
                if (half_period_counter == HALF_PERIOD_COUNTER_MAX) 
                begin
                    half_period_counter     <= 0;
                    spi_clk_i               <= ~spi_clk_i;

                    // if spi_clk_i = 1 we are reading data on the falling edge on SPI clock.
                    // if spi_clk_i = 0 we are reading data on the rising  edge on SPI clock.
                    if (spi_clk_i == 1'b0) 
                    begin
                        // If this is a data bit we sample it
                        if (spi_bit_counter < SPI_BIT_COUNTER_MAX) 
                        begin
                            miso_shift_reg  <= {miso_shift_reg[6:0], spi_miso};
                            spi_bit_counter <= spi_bit_counter + 1;
                        end
                    end else 
                    begin
                        // Data is output on the falling edge of SPI clock.
                        if (spi_bit_counter < SPI_BIT_COUNTER_MAX) 
                            spi_mosi        <= spi_tx_tData[7- spi_bit_counter];

                        // If finished reading the byte make the read data available if we are in
                        // the read stage, and go back to idle state.
                        if (spi_bit_counter == SPI_BIT_COUNTER_MAX) 
                        begin
                            if (flash_read_active)
                            begin
                                flash_tData  <= miso_shift_reg;
                                flash_tValid <= 1'b1;
                            end
                            // Set signals back to idle levels
                            spi_clk_i       <= 1'b1;
                            spi_bit_counter <= 0;
                            spi_mosi        <= 1'b1;
                            spi_state       <= SPI_IDLE;
                        end
                    end
                end else 
                begin
                    // Update the counter while we wait for the next cock edge.
                    half_period_counter     <= half_period_counter + 1;
                end
            end

            default : 
                spi_state <= SPI_IDLE;

            endcase
        end
    end


    //
    // FLASH reading
    //

    // The command number specifies the byte to be transmitted on the SPI MOSI port.
    // If adding or removing commands remember to change LAST_COMMAND_BYTE and make 
    // sure `command_count` register has enough bits to support all the commands.
    localparam LAST_COMMAND_BYTE = 4;
    always @(*)
    begin : SELECT_TX_BYTE
        case(command_count)
        0: spi_tx_tData = WAKEUP_COMMAND;
        1: spi_tx_tData = READ_COMMAND;
        2: spi_tx_tData = flash_address[23:16];
        3: spi_tx_tData = flash_address[15:8];
        4: spi_tx_tData = flash_address[7:0];
        default: spi_tx_tData = 8'hFF;
        endcase
    end

    //
    // State machine to control read/write of bytes to/from the FLASH.
    // Signal spi_tx_tValid triggers the SPI state machine to perform
    // the actual SPI read/write.
    // 1. Wait for activate command from upper level module.
    // 2. Wake the FLASH chip from power down mode and wait 35us.
    // 3. Put the FLASH in read mode and assign start address.
    // 4. Read the bytes coninuously and signal we are reading using the
    //    flash_read_active flag.
    // 5. De-assert the flash_read_active signal to show completion.
    //
    always @(posedge clock)
    begin : READ_FLASH_STATE_MACHINE

        if (reset_n == 1'b0)
        begin
            spi_cs_n                    <= 1'b1;
            flash_address               <= START_ADDRESS;
            flash_read_active           <= 1'b0;
            flash_wait_timer            <= 1'b0;
            command_count               <= 0;
            byte_counter                <= 0;
            flash_read_state            <= FLASH_IDLE;
        end else
        begin
            // Default value for pulsed SPI TX data valid signal.
            spi_tx_tValid               <= 1'b0;

            case (flash_read_state)

            FLASH_IDLE : // Waiting on upper level module for enable
            begin
                if (flash_read_en == 1'b1)
                begin
                    spi_cs_n            <= 1'b1;
                    command_count       <= 0;
                    flash_read_active   <= 1'b0;
                    flash_wait_timer    <= 1'b0;
                    flash_read_state    <= FLASH_WAKE_UP;
                end
            end

            FLASH_WAKE_UP : // Wake the FLASH from power down mode.
            begin   
                if (spi_state == SPI_IDLE && spi_tx_tValid == 1'b0 )
                begin   
                    spi_cs_n            <= 1'b0;
                    spi_tx_tValid       <= 1'b1;
                    flash_read_state    <= FLASH_WAIT;
                end
            end

            FLASH_WAIT : // Wait the specified 35us to wake up
            begin
                if (spi_state == SPI_IDLE && spi_tx_tValid == 1'b0 && flash_wait_timer == 1'b0)
                begin
                    spi_cs_n            <= 1'b1;
                    flash_wait_timer    <= 1'b1;
                end

                if (flash_wait_complete == 1'b1)
                begin
                    spi_cs_n            <= 1'b0;
                    flash_wait_timer    <= 1'b0;
                    flash_read_state    <= FLASH_READ_CMD;
                end
            end

            FLASH_READ_CMD : // Send the 4byte READ command + Address to the FLASH.
            begin
                if (spi_state == SPI_IDLE && spi_tx_tValid == 1'b0)
                    if (command_count == LAST_COMMAND_BYTE)
                    begin
                        flash_read_state    <= FLASH_READ_DATA;
                        // Advance to the last MOSI data byte = 0xFF for reading.
                        command_count       <= command_count + 1;
                    end else
                    begin
                        spi_tx_tValid       <= 1'b1;
                        command_count       <= command_count + 1;
                    end
            end

            FLASH_READ_DATA : // Read the specified number of bytes
            begin
                if (spi_state == SPI_IDLE && spi_tx_tValid == 1'b0)
                    if (byte_counter == NUM_READ_BYTES)
                    begin
                        flash_read_active   <= 1'b0;
                        flash_read_state    <= FLASH_STOP;
                    end else
                    begin
                        flash_read_active   <= 1'b1;
                        spi_tx_tValid       <= 1'b1;
                        byte_counter        <= byte_counter + 1;
                    end
            end

            FLASH_STOP : // Stay here until the read enable signal is lowered.
            begin
                spi_cs_n                    <= 1'b1;

                if (flash_read_en == 1'b0)
                    flash_read_state        <= FLASH_IDLE;
            end

            default :
            begin
                flash_read_state        <= FLASH_IDLE;
                command_count           <= 0;
            end

            endcase
        
        end
    end



endmodule
