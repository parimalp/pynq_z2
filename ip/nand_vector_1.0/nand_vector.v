`timescale 1ns / 1ps
/////////////////////////////////////////////////////////////////
// Module Name: nand_vector
/////////////////////////////////////////////////////////////////
module nand_vector #(parameter SIZE=4, DELAY=3)(
    input wire [SIZE-1:0] a,
    input wire [SIZE-1:0] b,
    output wire [SIZE-1:0] y
    );

   nand #DELAY (y,a,b);

endmodule
