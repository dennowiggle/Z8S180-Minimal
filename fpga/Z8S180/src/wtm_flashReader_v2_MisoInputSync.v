// ****************************************************************************
// * 
// *    Code by Denno Wiggle aka WTM. 
// *    
// *    Module will read data from the FPGA configuartion SPI flash memory.
// *
// *    Copyright (C) 2023 Denno Wiggle
// *
// *    Read data from the SPI flash memory.
// *    -- v2 synchronizes MISO to the clock domain.
// *    -- The SPI clock, MISO and MOSI have separate processes.
// *    -- v2 uses approxinately 50 more ice40 logic cells than v1.
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
    output reg          spi_clk,
    output reg          spi_cs,
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
    localparam SPI_NUM_DATA_BITS        = 8;
    localparam SPI_BIT_COUNTER_MAX      = SPI_NUM_DATA_BITS - 1;
    localparam SPI_BIT_COUNTER_WIDTH    = $clog2(SPI_BIT_COUNTER_MAX + 1);
    localparam SPI_CLK_COUNTER_WIDTH    = $clog2(SPI_NUM_DATA_BITS*2 + 1);

    // Counters used in SPI clock, MISO, and MOSI processes.
    reg  [SPI_BIT_COUNTER_WIDTH - 1:0]  spi_tx_bit_counter;
    reg  [SPI_BIT_COUNTER_WIDTH - 1:0]  spi_rx_bit_counter;
    reg  [SPI_CLK_COUNTER_WIDTH - 1:0]  spi_clockedge_count; // Twice the number of bits

    // Command count sets the command byte number to transmit.
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
    reg   [2:0] flash_read_state;
    parameter [2:0]
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
    wire        spi_miso_sync;
    reg  [7:0]  miso_shift_reg;
    // Holds the TX data to be sent out on the SPI transmit MOSI port.
    // The data valid signal has a duration of one clock cycle.
    reg  [7:0]  spi_tx_tData;
    reg  [7:0]  spi_tx_byte;
    reg         spi_tx_tValid;

    // Signals relating to the SPI clock
    reg         spi_clk_i       = 1'b1; // Internal version of the SPI clock.
    reg         spi_rclk_tick;          // High for one clock pulse if rising edge of SPI clock.
    reg         spi_fclk_tick;          // High for one clock pulse if falling edge of SPI clock.
    wire        spi_rclk_tick_sync;     // Delayed version that matches MISO timing. 

    // Signals that the MISO shift register has valid data and can data can be latched.
    reg         latch_rx_data;


    // Synchronize the incoming MISO serial data to the system clock.
    wtm_sigSync spi_miso_sigSync(
        .clock      (clock),
        .rst_n      (reset_n),
        .sig_in     (spi_miso),
        .sig_out    (spi_miso_sync)
    );

    // Delay the rising clock tick by the same as MISO for reading MISO.
    wtm_sigSync spi_fclk_sigSync(
        .clock      (clock),
        .rst_n      (reset_n),
        .sig_in     (spi_rclk_tick),
        .sig_out    (spi_rclk_tick_sync)
    );

    // 35us timer used to wait after certain flash commands like reset and wake up.
    wtm_resetSyncDelay #(35, CLOCK_FREQ_HZ)
    flash_wait_timer_inst(
        .clock(clock),
        .rst_n(flash_wait_timer),
        .rst_out_n(flash_wait_complete)
    );


    //
    // Generate the SPI clock
    //
    always @(posedge clock) 
    begin : SPI_CLOCK_GEN
        if (reset_n == 1'b0) 
        begin
            spi_clk_i                   <= 1'b1;
            half_period_counter         <= 0;
            spi_clockedge_count         <= 0;
            spi_rclk_tick               <= 0;
            spi_fclk_tick               <= 0;
        end else
        begin
            spi_rclk_tick               <= 0;
            spi_fclk_tick               <= 0;
            // Output clock has to be in sync with MOSI,
            // so delay by one system clock.
            spi_clk                     <= spi_clk_i;

            // Check if valid data to start a SPI cycle
            if (spi_tx_tValid == 1'b1)
                spi_clockedge_count     <= SPI_NUM_DATA_BITS * 2;

            if (spi_clockedge_count > 0)
            begin
                if (half_period_counter == HALF_PERIOD_COUNTER_MAX) 
                begin
                    half_period_counter <= 0;
                    spi_clockedge_count <= spi_clockedge_count - 1;

                    // Create a tick for the clock edge
                    if (spi_clk_i == 1'b0)
                        spi_rclk_tick   <= 1'b1;
                    else
                        spi_fclk_tick   <= 1'b1;

                    spi_clk_i           <= ~spi_clk_i;
                end else
                begin
                    half_period_counter <= half_period_counter + 1;
                end
            end
        end
    end


    //
    // Send the MOSI data.
    //
    always @(posedge clock) 
    begin : SPI_MOSI_GEN
        if (reset_n == 1'b0) 
        begin
            spi_mosi                    <= 1'b1;
            spi_tx_bit_counter          <= 0;
        end else
        begin
            // Check if valid data to start a SPI cycle
            if (spi_tx_tValid == 1'b1)
            begin
                spi_tx_bit_counter      <= SPI_BIT_COUNTER_MAX;
                spi_tx_byte             <= spi_tx_tData;
            end

            if (spi_fclk_tick) 
            begin
                spi_mosi                <= spi_tx_byte[spi_tx_bit_counter];
                spi_tx_bit_counter      <= spi_tx_bit_counter - 1;
            end
        end
    end


    //
    // Receive MISO data.
    //
    always @(posedge clock) 
    begin : SPI_MISO_RECV
        if (reset_n == 1'b0) 
        begin
            spi_rx_bit_counter          <= 0;
            latch_rx_data               <= 1'b0;
            flash_tValid                <= 1'b0;
            spi_state                   <= SPI_IDLE;
        end else
        begin
            // Default value for pulsed data signals.
            flash_tValid                <= 1'b0;     
            latch_rx_data               <= 1'b0;

            // Check if valid data to start a SPI cycle
            if (spi_tx_tValid == 1'b1)
            begin
                spi_rx_bit_counter      <= SPI_BIT_COUNTER_MAX;
                spi_state               <= SPI_PROCESSING;
            end

            // If a rising clock shift in MOSI data
            if (spi_rclk_tick_sync == 1'b1) 
            begin
                miso_shift_reg          <= {miso_shift_reg[6:0], spi_miso_sync};
                if (spi_rx_bit_counter == 0) 
                    latch_rx_data       <= 1'b1;
                else
                    spi_rx_bit_counter  <= spi_rx_bit_counter - 1;
            end

            if (latch_rx_data)
            begin
                if (flash_read_active)
                begin
                    flash_tData         <= miso_shift_reg;
                    flash_tValid        <= 1'b1;
                end
                spi_state               <= SPI_IDLE;
            end
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
            spi_cs                      <= 1'b1;
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
                    spi_cs              <= 1'b1;
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
                    spi_cs              <= 1'b0;
                    spi_tx_tValid       <= 1'b1;
                    flash_read_state    <= FLASH_WAIT;
                end
            end

            FLASH_WAIT : // Wait the specified 35us to wake up
            begin
                if (spi_state == SPI_IDLE && spi_tx_tValid == 1'b0 && flash_wait_timer == 1'b0)
                begin
                    spi_cs              <= 1'b1;
                    flash_wait_timer    <= 1'b1;
                end

                if (flash_wait_complete == 1'b1)
                begin
                    spi_cs              <= 1'b0;
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
                spi_cs                      <= 1'b1;

                if (flash_read_en == 1'b0)
                    flash_read_state        <= FLASH_IDLE;
            end

            default :
            begin
                flash_read_state        <= FLASH_IDLE;
            end

            endcase
        
        end
    end



endmodule
