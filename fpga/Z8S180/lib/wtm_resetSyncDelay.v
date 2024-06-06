// ****************************************************************************
// * 
// *    Code by Denno Wiggle aka WTM.
// *
// *    Copyright (C) 2023 Denno Wiggle
// *
// *    -- Syncrhonize a reset signal to the clock domain and add a delay 
// *       specified by the user as a parameter.
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

module wtm_resetSyncDelay #(
    parameter delay_in_us = 1250,
    parameter clk_freq_hz = 10000000
)
(
    input  wire     clock,
    input  wire     rst_n,
    output reg      rst_out_n
);

    // We must have a minium of 2 clock cyles to synchronize
    // WTM todo : Need to implement a check for min 2.
    //            We could just add 2 clocks?
    integer counter_max = delay_in_us * (clk_freq_hz / 1000000);
    integer counter = 0;

    always @(negedge rst_n, posedge clock) 
    begin
        if (rst_n == 1'b0) begin
            rst_out_n <= 1'b0;
            counter   <= 0;
        end else 
        begin
            if (counter < counter_max)
            begin
                counter <= counter + 1;
            end
            else begin
                rst_out_n <= 1'b1;
            end
        end
    end


endmodule
