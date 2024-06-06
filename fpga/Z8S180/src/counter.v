// Code from https://github.com/johnwinans/Verilog-Examples/

// Don't allow inferred nets as that can cause debug issues
`default_nettype none

module counter (
    input wire clk,
    input wire reset_n,
    output wire [7:0] led
    );

    reg [31:0] counter = 32'b0;

    //assign led = ~counter[28:21];
    //assign led = counter[27:20];
    assign led = ~counter[30:23];

    always @ (posedge clk) 
    begin
        if (reset_n == 0)
            counter <= 0;
        else
            counter <= counter + 1;
    end

endmodule
