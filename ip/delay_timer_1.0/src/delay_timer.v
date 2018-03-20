`timescale 1ns / 1ps

module delay_timer #(parameter C_COUNTER_WIDTH = 32)(
    input clk,
    input reset_n,
    input start,
    input stop,
    input reset_done,
    input [C_COUNTER_WIDTH-1:0] delay_cnt,
    output reg done
    );
    
    reg [C_COUNTER_WIDTH-1:0] count = {C_COUNTER_WIDTH{1'b0}}; 
    reg cnt_enb;
    
    wire start_cnt, stop_cnt, cnt_done, cntr_reset_n, reset_done_int;
 
//    assign done = ((count <= duty_cnt) && (cnt_enb==1)) ? 1 : 0;
    
    xpm_cdc_async_rst #(    
       //Common module parameters
       .DEST_SYNC_FF    (2), // integer; range: 2-10
       .RST_ACTIVE_HIGH (0)  // integer; 0=active low reset, 1=active high reset     
    ) xpm_cdc_async_rst_inst (    
       .src_arst  (reset_n),
       .dest_clk  (clk),
       .dest_arst (cntr_reset_n)     
    );
 
    pulse_gen sync_start(.async_in(start), .sync_clk(clk), .pulsed_out(start_cnt));
    pulse_gen sync_stop(.async_in(stop), .sync_clk(clk), .pulsed_out(stop_cnt));
    pulse_gen sync_reset_done(.async_in(reset_done), .sync_clk(clk), .pulsed_out(reset_done_int));
    
    assign cnt_done = (count == delay_cnt);
 
    always @(posedge clk)
       if ((!cntr_reset_n) || (stop_cnt) || (reset_done_int))
          cnt_enb <= 1'b0;
       else if (start_cnt) 
          cnt_enb <= 1'b1;
 
    always @(posedge clk)
         if ((!cntr_reset_n) || (stop_cnt) || (reset_done_int))
            done <= 1'b0;
         else if (cnt_done) 
            done <= 1'b1;

    always @(posedge clk)
     if ((!cntr_reset_n) || (start_cnt) || (cnt_done))
        count <= {{(C_COUNTER_WIDTH-1){1'b0}},1'b1};
     else if (cnt_enb)        
        count <= count + 1'b1;

endmodule
