// ****************************************************************************
// * 
// *    Code by Denno Wiggle aka WTM for the "2057-ICE40HX4K-TQ144-breakout" 
// *    board connected to 2067-Z8S180 CPU board. Both boards designed by
// *    John Winans.
// *
// *    Copyright (C) 2023 Denno Wiggle
// *
// *    -- Controls CPU inputs based on CPU outputs.
// *    -- Provides clock and reset to the CPU.
// *    -- Provides Memory Control Signals.
// *       - SRAM memory is on the CPU board. The FPGA sets the SRAM R/W/OE 
// *         signals dependant on the CPU R/W/MREQ and address signals.
// *       - Muxes memory control signals for CPU vs ROM load access
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


module top #(
    parameter CPU_CLOCK_FREQ_HZ         = 18360000,
    parameter LOGIC_CLOCK_FREQ_HZ       = 25000000,
    parameter FLASH_SPI_CLOCK_HZ        = 12500000,
    parameter FLASH_SW_START_ADDRESS    =   135100,
    parameter FLASH_SW_NUM_BYTES        = 512*1024,
)(
    // The PLL on this FPGA is using clock_25 input pin 52 on the 2057-ICE40HX4K board.
    // This means pin 49 (cpu_dreq1_n) can only be an output
    // See Lattice FPGA technical note "FPGA-TN-02052-1.4" section 5 "Hardware Design Considerations"
    input  wire         clock_25_in,

    // Local hardware reset signal pin - connected to a reset switch
    //
    // There is no reset supervisor IC or RC delay on power-up in hardware.
    // WTM - Added a cap/diode rework on hardware reset together with a 3.3V rail
    //       bleed resistor. This takes care of delaying reset during power-up
    //       to ensure reset block code is actually run. (iceCube2 does not allow
    //       forced initial conditions except for memory blocks).
    //  
    // In the code all reset signals derived from the hardware reset signal 
    // incorporate a minimum of a 1ms delay. In additon the CPU reset is gated 
    // on the PLL lock signal plus an additional 1ms delay.
    input  wire         fpga_reset_n,

    // Clock that we provide to the CPU from the PLL output.
    // Z8S180 will be 18.36MHz for friendly baud rates
    output wire         clock_cpu_out,
    // Clock received back from the CPU. We use this for clocking.
    // See Z8S180 Product Specification for the bus timing relationship to the PHI clock.
    input  wire         cpu_clk_phi,
    // Output to control reset to the CPU
    output wire         cpu_reset_n,

    // CPU address and data bus
    inout  wire [19:0]  cpu_address,
    inout  wire  [7:0]  cpu_data,
    // When MREQ is asserted the CPU requests a memory space transaction (e.g. SRAM/ROM)
    // When asserted CPU requests a memory space transaction 
    input wire          cpu_mreq_n,
    // When IORQ is asserted CPU requests an IO space transaction
    input wire          cpu_iorq_n,
    // When IORQ and M1 are asserted, an interrupt acknowledgement is taking place
    // and not an I/O memory space transaction.
    input wire          cpu_m1_n,
    // CPU read and write signals are used in conjuction with cpu_mreq_n or cpu_iorq_n 
    input wire          cpu_wr_n,
    input wire          cpu_rd_n,
    // Wait tells the CPU that we need more time.
    output wire         cpu_wait_n,
    // Bus request and acknowledgement signals if we need to request the address and data bus.
    output wire         cpu_busreq_n,
    input  wire         cpu_busack_n,

    // Interrupt pins
    output wire         cpu_nmi_n,
    output wire         cpu_int0_n,
    output wire         cpu_int1_n,
    output wire         cpu_int2_n,

    // DMA request signal
    output wire         cpu_dreq1_n,
    // End of block transfer DMA operation
    input  wire         cpu_tend1_n,

    // Enable Clock (Output). This pin functions as a synchronous,
    // machine-cycle clock output during bus transactions.
    //  - The Z8S180 Product Spec figure 20 shows timing
    //    with reference to the CPU PHI clock output. That is what
    //    we will use in the code. We will ignore the E signal.
    //  - In the Z8081x User Manual Figure 12 & 13 show the same thing.
    input  wire         cpu_e,

    // Status (Output). This signal is used with the M1_n and
    // Halt_n outputs to decode the status of the CPU machine cycle
    input  wire         cpu_st,
    // Refresh is not used. We are lucky enough not to need DRAM.
    // With SRAM we don't need refresh so code will run faster.
    input  wire         cpu_refresh_n,
    // Halt lets us know the CPU has stopped e.g. sleep mode.
    input  wire         cpu_halt_n,

    // SRAM Memory control pins. The FPGA controls these based on address, 
    // data, and other inputs.
    output wire         mem_ce_n,
    output wire         mem_we_n,
    output wire         mem_oe_n,

    // SNES Controller signals
    output wire         snes_clock,
    output wire         snes_latch,
    input  wire         snes_data1,
    input  wire         snes_data0,

    // UART Signals
    output wire         uart0_tx,
    input  wire         uart0_rx,
    output wire         uart1_tx,
    input  wire         uart1_rx,

    // SD Card signals
    output wire         sd_cs_n,
    output wire         sd_clk,
    output wire         sd_mosi,
    input  wire         sd_miso,
    input  wire         sd_det_n,

    // SPI signals attached to FPHA and configuration FLASH
    output wire         spi_clk,
    output wire         spi_cs,
    output wire         spi_mosi,
    input  wire         spi_miso,

    // WTM : Debug Only Port - for wtm_busInterface.v
    // Good place to hook up your probes if you have them.
    output wire  [7:0]  debug_port,

    // WTM : Debug Port - to check signals in top.v 
    output wire  [2:0]  debug_test,
    
    // FPGA board LED's
    output wire  [7:0]  led
);


    // Synchronised reset signals for specific clock domains
    wire        reset_25_n;            // referenced to clock_25
    wire        reset_cpu_src_n;       // referenced to clock_cpu_src (clock sent out to CPU)
    wire        reset_cpu_n;           // referenced to clock_cpu (clock received by CPU)
    
    // Clocks
    // Input clock from global buffer
    wire        clock_25;
    // Clock from CPU (output of global buffer whose input is the CPU PHI clock)
    wire        clock_cpu;
    // Clock source to CPU
    wire        clock_cpu_src;
    // Clock from second PLL at 50MHz
    // wire        clock_50;
    // PLL Lock signal used for gating reset on logic that uses the PLL clock.
    wire        clock_pll_lock;
    // PLL Lock signal used for gating reset on logic that uses the second PLL clock.
    // wire clock_50_lock;

    // CPU Data bus signals
    wire [7:0]  cpu_data_in;
    wire [7:0]  cpu_data_out;
    wire        cpu_data_oe;

    // CPU Address bus input signal
    wire [19:0] cpu_address_in;

    // Signals used for writing from SPI FLASH to SRAM memory
    wire [19:0] boot_address;
    wire  [7:0] boot_data;
    wire        boot_ce_n;
    wire        boot_we_n;

    // The FLASH to RAM write sequence is complete if this is asserted
    wire        rom_load_done;

    // Warm boot status flag register.
    // 0 = FPGA and CPU cold boot
    // 1 = CPU has been reset by the FPGA after the FPGA has been commanded to do so.
    reg         status_cpu_wboot;

    // The input clock should use a global buffer
    SB_GB global_buf_clock_25 (
        .USER_SIGNAL_TO_GLOBAL_BUFFER (clock_25_in),
        .GLOBAL_BUFFER_OUTPUT (clock_25) 
    );

    // The clock received back from the CPU uses a global buffer
    SB_GB global_buf_phi (
        .USER_SIGNAL_TO_GLOBAL_BUFFER (cpu_clk_phi),
        .GLOBAL_BUFFER_OUTPUT (clock_cpu) 
    );

    // Define the Data bus as a tri-state bidirectional bus with pull-ups
    SB_IO #(
        .PIN_TYPE       (6'b 1010_01),      // output = tri-state, input = 1
        .PULLUP         (1'b 1)             // enable the pullup = 1
    ) iobuf_data[7:0] (
        .PACKAGE_PIN    (cpu_data),
        .OUTPUT_ENABLE  (cpu_data_oe),
        .D_OUT_0        (cpu_data_out),
        .D_IN_0         (cpu_data_in)
    );

    // Define the Address bus as a tri-state bidirectional bus with pull-ups.
    // At startup port is an output to write ROM to RAM before the CPU
    // is enabled. After the ROM load is complete the address is a read 
    // only port.
    SB_IO #(
        .PIN_TYPE       (6'b 1010_01),      // output = tri-state, input = 1
        .PULLUP         (1'b 1)             // enable the pullup = 1
    ) iobuf_address[19:0] (
        .PACKAGE_PIN    (cpu_address),
        .OUTPUT_ENABLE  (~rom_load_done),
        .D_OUT_0        (boot_address),
        .D_IN_0         (cpu_address_in)
    );


    // Assign debug pin to debug top level verilog internal signals
    assign debug_test[0] = rom_load_done;
    assign debug_test[1] = spi_cs;
    assign debug_test[2] = reset_cpu_n;

    // The uart 0 and 1 ports are not currently used
    assign uart0_tx = 1'b1;
    assign uart1_tx = 1'b1;


    // PLL input frequency  = 25MHz
    // PLL target frequency = 18.432MHz
    // PLL actual frequency = 18.36MHz (-0.4%)
    pll_cpu pll_cpu_inst(
        .REFERENCECLK    (clock_25_in),
        .PLLOUTCORE      (),
        .PLLOUTGLOBAL    (clock_cpu_src),
        .RESET           (fpga_reset_n),
        .LOCK            (clock_pll_lock)
    );


    // Reset signal generation.

    // Synchronize the FPGA logic reset to the FPGA clock input and reset imput pins
    // and add a 1ms delay
    wtm_resetSyncDelay #(1000, LOGIC_CLOCK_FREQ_HZ)
    sync_delay_reset_to_clock_25(
        .clock          (clock_25),
        .rst_n          (fpga_reset_n),
        .rst_out_n      (reset_25_n)
    );

    // Synchronize reset_cpu_src_n to the 18.36MHz PLL clock, the PLL lock signal for 
    // that clock, and the FPGA logic reset signal.
    wtm_resetSync sync_resetn_to_clock_cpu_src(
        .clock          (clock_cpu_src),
        .rst_n          (reset_25_n && clock_pll_lock),
        .rst_out_n      (reset_cpu_src_n)
    );

    // If the FPGA or PLL has received a hard reset then consider this a cold boot
    //     and set signal "status_cpu_wboot" flag to 0.
    // If the CPU commands the FPGA to reset itself then set signal 
    //     "status_cpu_wboot" to 1. 
    wire reset_cpu_logic;
    always @(posedge clock_cpu or negedge reset_cpu_src_n)
    begin
        if (reset_cpu_src_n == 1'b0)
        begin
            status_cpu_wboot <= 1'b0;
        end else
        begin
            if (reset_cpu_logic == 1'b1)
            begin
                status_cpu_wboot <= 1'b1;
            end else 
            begin
                status_cpu_wboot <= status_cpu_wboot;
            end
        end
    end

    // Synchronize reset_cpu_n signal to the PHI clock received from the CPU
    // and add a 1ms delay for warm reset or 2ms for cold reset. 
    // * The CPU will send the PHI clock (clock_cpu) back to us even when the
    //   the CPU is held in reset (confirmed on oscilloscope).
    // * This means any logic using the received clock can use use the 
    //   synchronous reset signal reset_cpu_n.
    // * At startup on the Z180 CPU we receive a clock that is half the frequency 
    //   of the one we sent. 
    // * The total CPU reset time is the 1ms here and the 1ms  
    //   generated by 'reset_cpu_src_n' for a total of 2ms. 
    //
    // The code provides a mechanism for the CPU to reset itself by writing to 
    // a control register bit to assert the reset_cpu signal.
    // * Upon assertion of reset_cpu_n the control bit will reset back to 0.
    // 
    wtm_resetSyncDelay #(1000, CPU_CLOCK_FREQ_HZ/2)
    sync_delay_reset_to_clock_cpu(
        .clock(clock_cpu),
        .rst_n(reset_cpu_src_n & ~reset_cpu_logic & rom_load_done),
        .rst_out_n(reset_cpu_n)
    );

    // These signals are not used right now but the CPU needs them to be high
    assign cpu_busreq_n = 1'b1;
    assign cpu_nmi_n    = 1'b1;
    assign cpu_int1_n   = 1'b1;
    assign cpu_int2_n   = 1'b1;
    assign cpu_dreq1_n  = 1'b1;

    // Send the CPU reset signal to the CPU
    assign cpu_reset_n  =  reset_cpu_n;

    // Send the PLL derived 18.36MHz clock to the CPU if we have PLL lock.
    assign clock_cpu_out = (clock_pll_lock) ? clock_cpu_src : 1'b1;

    // The LED's can be driven by an FPGA register setting 
    // or by an FPGA counter (John Winan's Blinky test code).
    wire [7:0] led_register;
    wire led_cpu_en;

    // Control the LED's either by the counter or by the FPGA resister
    // which is under CPU control.
    assign led = (led_cpu_en == 1'b0) ? led_counter : led_register;

    // Internal signals that will generate an interrupt
    wire [7:0] irq_vector;

    // This is where we assign the IRQ signals used by the IRQ registers and 
    // if selected the Interrupt pin
    assign irq_vector = {6'b000000, 1'b0, 1'b0};

    // Link in the module that interfaces to the CPU address and data bus
    // and provides FPGA registers for the CPU to access.
    wtm_busInterface #(
        .DATA_WIDTH             (8),
        .ADDR_WIDTH             (20)
    )
    z80_busInterface(
        // CPU clock and reset
        .clock                  (clock_cpu),
        .reset_n                (reset_cpu_n),

        // CPU address and data bus
        .cpu_address            (cpu_address_in),
        .cpu_data_out           (cpu_data_out),
        .cpu_data_in            (cpu_data_in),
        .cpu_data_oe            (cpu_data_oe),

        // CPU bus control signals
        .cpu_mreq_n             (cpu_mreq_n),
        .cpu_iorq_n             (cpu_iorq_n),
        .cpu_m1_n               (cpu_m1_n),
        .cpu_wr_n               (cpu_wr_n),
        .cpu_rd_n               (cpu_rd_n),

        // Wait tells the CPU that we need more time.
        .cpu_wait_n             (cpu_wait_n),

        // SRAM control pins.
        .mem_ce_n               (mem_ce_n),
        .mem_we_n               (mem_we_n),
        .mem_oe_n               (mem_oe_n),

        // Signals used for writing from SPI FLASH to SRAM memory
        .boot_address           (boot_address),
        .boot_data              (boot_data),
        .boot_ce_n              (boot_ce_n),
        .boot_we_n              (boot_we_n),

        // The FLASH to RAM write sequence is complete if this is asserted
        .rom_load_done          (rom_load_done),

        // Signals that attach to FPGA registers.
        .led_register           (led_register),
        .led_cpu_en             (led_cpu_en),
        .status_cpu_wboot       (status_cpu_wboot),

        // Interrupt signals
        .irq_vector             (irq_vector),
        .cpu_int0_n             (cpu_int0_n),
        
        // Signal to reset the CPU via a register write
        .reset_cpu_logic        (reset_cpu_logic),

        // 8 bit debug port for board bringup and test.
        .debug_port             (debug_port)
    );

    // Clock and reeset
    wire        clock_sd;
    wire        reset_sd_n;

    // At startup load the SRAM memory with the SW contents of the SPI FLASH.
    // This is to perform a ROM load to RAM before the CPU is enabled.
    wtm_flashToMemory #(
        .FLASH_LOGIC_CLOCK_HZ   (LOGIC_CLOCK_FREQ_HZ),
        .FLASH_SPI_CLOCK_HZ     (FLASH_SPI_CLOCK_HZ),
        .FLASH_SW_START_ADDRESS (FLASH_SW_START_ADDRESS),
        .FLASH_SW_NUM_BYTES     (FLASH_SW_NUM_BYTES),
        .MEM_START_ADDRESS      (0)
    ) flashToMemory (
        // Clock and reset
        .clock                  (clock_25),
        .reset_n                (reset_25_n & ~reset_cpu_logic),

        // SRAM Memory address, data, and control ports. 
        .mem_address            (boot_address),
        .mem_data               (boot_data),
        .mem_ce_n               (boot_ce_n),
        .mem_we_n               (boot_we_n),

        // The FLASH to RAM write sequence is complete
        .rom_load_done          (rom_load_done),

        // SPI signals
        .spi_clk                (spi_clk),
        .spi_cs                 (spi_cs),
        .spi_mosi               (spi_mosi),
        .spi_miso               (spi_miso)
    );

    // John's Basement Counter Code to flash the LED's and keep us entertained,
    // unless the CPU takes control of the LED's via a register bit setting.
    wire [7:0] led_counter;
    counter c (
        .clk        (clock_cpu_src),
        .reset_n    (reset_cpu_src_n),
        .led        (led_counter)
    );
    
endmodule
