`timescale 1ns/1ps

// ============================================================
// cdc_pulse_sync
// 功能：
//   将 src_clk 域的 1clk 脉冲，安全地同步到 dst_clk 域，
//   并在 dst_clk 域输出 1clk 宽度的脉冲
//
// 原理：
//   - 源域：把 pulse 翻转成 toggle
//   - 目的域：对 toggle 打两拍
//   - 检测 toggle 翻转 → 产生 1clk pulse
// ============================================================
module cdc_pulse_sync (
    input  wire src_clk,
    input  wire src_rst_n,
    input  wire src_pulse,   // src_clk 域的 1clk 脉冲

    input  wire dst_clk,
    input  wire dst_rst_n,
    output wire dst_pulse    // dst_clk 域的 1clk 脉冲
);

    // ------------------------------------------------
    // 源时钟域：pulse → toggle
    // ------------------------------------------------
    reg src_toggle;

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n)
            src_toggle <= 1'b0;
        else if (src_pulse)
            src_toggle <= ~src_toggle;
    end

    // ------------------------------------------------
    // 目的时钟域：同步 toggle
    // ------------------------------------------------
    reg dst_toggle_ff1, dst_toggle_ff2;

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            dst_toggle_ff1 <= 1'b0;
            dst_toggle_ff2 <= 1'b0;
        end else begin
            dst_toggle_ff1 <= src_toggle;
            dst_toggle_ff2 <= dst_toggle_ff1;
        end
    end

    // ------------------------------------------------
    // toggle 边沿检测 → 1clk pulse
    // ------------------------------------------------
    assign dst_pulse = dst_toggle_ff1 ^ dst_toggle_ff2;

endmodule
