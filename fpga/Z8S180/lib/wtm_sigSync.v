// ****************************************************************************
// * 
// *    Code by Denno Wiggle aka WTM.
// *
// *    Copyright (C) 2023 Denno Wiggle
// *
// *    -- Generate a synchronized signal to a new clock domain
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

module wtm_sigSync #(
    // Use a two flip-flop synchronizer
    parameter WIDTH = 2
)(
    input  wire clock,
    input  wire rst_n,
    input  wire sig_in,
    output wire sig_out
);

    // Use an N flip-flop synchronizer
    reg [1:WIDTH] syncReg;

    always @(negedge rst_n, posedge clock) 
    begin
        if (rst_n == 1'b0) begin
            syncReg <= {(WIDTH){1'b0}};
        end else begin
            syncReg <= {sig_in,syncReg[1:WIDTH - 1]};
        end
    end

    assign sig_out = syncReg[WIDTH];

endmodule
