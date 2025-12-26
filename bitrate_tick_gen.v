// bitrate_tick_gen.v
`timescale 1ns/1ps

module bitrate_tick_gen #(
    parameter integer CLK_HZ = 130_000_000
)(
    input  wire       clk,        // clk_130M
    input  wire       rst_n,

    input  wire [2:0] rate_sel,    // 系统速率选择
    input  wire       enable,      // 发送期间=1

    output reg        bit_tick     // 1clk 宽脉冲
);

    // rate_sel -> 分频映射（内部使用）
    reg [7:0] div_val;
    always @(*) begin
        case (rate_sel)
            3'd0: div_val = 8'd64; // 2 Mbps
            3'd1: div_val = 8'd32; // 4 Mbps
            3'd2: div_val = 8'd16; // 8 Mbps
            3'd3: div_val = 8'd8;  // 16 Mbps
            3'd4: div_val = 8'd4;  // 32 Mbps
            3'd5: div_val = 8'd2;  // 65 Mbps
            default: div_val = 8'd64;
        endcase
    end

    reg [7:0] cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt      <= 8'd0;
            bit_tick <= 1'b0;
        end else begin
            bit_tick <= 1'b0;

            if (!enable) begin
                cnt <= 8'd0;
            end else begin
                if (cnt == div_val - 1) begin
                    cnt      <= 8'd0;
                    bit_tick <= 1'b1;
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end
        end
    end

endmodule
