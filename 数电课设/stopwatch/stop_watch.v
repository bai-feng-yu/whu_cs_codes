module stop_watch(
    input clk, 
    input rstn, // start and reset; 1 for start, 0 for reset
    input sw_i,// pause and continue; 0 for continue, 1 for pause
    input [3:0]sw,
    output [3:0] disp_an_o, // which to be lighted
    output [7:0] disp_seg_o, // which num to be displayed
    output led_1,
    output led_2
);
   wire clk_1hz, clk_16div;
   wire [7:0] seconds; // [0, 59]
   wire [7:0] minutes; // [0, 100]

   // generate 1hz clk
   clock_divider clk_div(
       .clk(clk),
       .clk_1hz(clk_1hz)
   );

   clk_16divider clk16div(
       .clk(clk),
       .clk_16div(clk_16div)
   );

    // an selector to control which an is lighted
    an_selector an_st(
        .clk(clk_16div), // a quick clk 
        .seconds(seconds),
        .minutes(minutes),
        .an(disp_an_o),
        .seg(disp_seg_o)
    );

    // to count seconds and minutes properly
    my_counter counter(
        .clk(clk_1hz),
        .q_clk(clk),
        .rstn(rstn),
        .pause(sw_i),
        .sw(sw),
        .seconds(seconds),
        .minutes(minutes)
    );



    assign led_1 = rstn;
    assign led_2 = sw_i;

endmodule

module clock_divider(
    input clk,
    output reg clk_1hz
);
    reg [25:0] counter;
    always @(posedge clk) begin
            if(counter == 26'd25000000 - 1) begin
                counter <= 0;
                clk_1hz <= ~clk_1hz; // toggle clk_1hz
            end 
            else if(counter <26'd25_000_000-1)
            begin
                counter <= counter + 1;
            end
            else begin
                counter<=26'd0;
                clk_1hz<=1'b0;
            end
        end
endmodule

module clk_16divider(
    input clk,
    output wire clk_16div
);
    reg [16:0] counter;  // 4-bit counter to divide clock by 16

    always @(posedge clk) begin
        counter <= counter + 1'b1;
    end

    assign clk_16div = counter[8];

endmodule


module an_selector(
    input clk,
    input rstn,
    input [7:0] seconds,
    input [7:0] minutes,
    output reg [3:0] an,
    output reg [7:0] seg
);
    reg [3:0] m1, m0; // MM
    reg [3:0] s1, s0; // SS

    always @(minutes, seconds) begin
        m1 = minutes / 10;
        m0 = minutes % 10;
        s1 = seconds / 10;
        s0 = seconds % 10;
    end 

    reg [1: 0] selector; // [0, 3]

    always @(posedge clk) begin
        selector <= selector + 1;
    end

    function [7:0] seven_seg_translator;
        input [3:0] digit; // [0, 9]
        case (digit) // 共阳极数码管
            0: seven_seg_translator = 8'hc0;
            1: seven_seg_translator = 8'hf9;
            2: seven_seg_translator = 8'ha4;
            3: seven_seg_translator = 8'hb0;
            4: seven_seg_translator = 8'h99;
            5: seven_seg_translator = 8'h92;
            6: seven_seg_translator = 8'h82;
            7: seven_seg_translator = 8'hf8;
            8: seven_seg_translator = 8'h80;
            9: seven_seg_translator = 8'h90;
            default: seven_seg_translator = 8'hff;
        endcase
    endfunction
    
    always @(selector, m1, m0, s1, s0) begin
        case (selector)
            2'b00: begin
                an = 4'b1110;
                seg = seven_seg_translator(s0);
            end
            2'b01: begin
                an = 4'b1101;
                seg = seven_seg_translator(s1);
            end
            2'b10: begin
                an = 4'b1011;
                seg = seven_seg_translator(m0);
            end
            2'b11: begin
                an = 4'b0111;
                seg = seven_seg_translator(m1);
            end
        endcase
    end
endmodule

module my_counter(
    input clk,
    input q_clk,
    input rstn,
    input pause,
    input [3:0]sw,
    output reg [7:0] seconds,
    output reg [7:0] minutes
);
    parameter MAX_SECONDS = 59;
    parameter MAX_MINUTES = 59;
    parameter PRESET_SECONDS = 55;
    parameter PRESET_MINUTES = 59;

    reg reg_first;

    always @(posedge clk) begin
        if(~reg_first) begin
            seconds <= PRESET_SECONDS;
            minutes <= PRESET_MINUTES;
            reg_first<=1'b1;
        end
        else if (rstn) begin
            seconds <= PRESET_SECONDS;
            minutes <= PRESET_MINUTES;
        end 
        else if(pause&&sw[0]&&sw[1]&&sw[2]&&sw[3])begin
            seconds <= seconds;
            minutes <= minutes;
        end
        else if(pause && ~sw[0])begin
             if(minutes>=6'b110010)begin
                 minutes<=minutes-6'b110010;
                 end
             else begin
                 minutes<=minutes+4'b1010;
                 end
             end
        else if(pause && ~sw[2])begin
             if(seconds>=6'b110010)begin
                 seconds<=seconds-6'b110010;
                 end
             else begin
                 seconds<=seconds+4'b1010;
                 end
             end
        else if(pause && ~sw[1])begin
             if(minutes==MAX_MINUTES)begin
                 minutes<=0;
             end
             else begin
                 minutes<=minutes+1'b1;
             end
        end
        else if(pause && ~sw[3])begin
             if(seconds==MAX_SECONDS)begin
                 seconds<=0;
             end
             else begin
                 seconds<=seconds+1'b1;
             end
        end
        else if(~pause)begin
                if (seconds == MAX_SECONDS) begin
                    seconds <= 0;
                    if (minutes == MAX_MINUTES) begin
                        minutes <= 0;
                        end 
                    else begin
                        minutes <= minutes + 1;
                    end
                end 
                else begin
                    seconds <= seconds + 1;
                end
            end
        end

    
endmodule
