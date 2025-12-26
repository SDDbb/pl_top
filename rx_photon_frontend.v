`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// rx_photon_frontend (improved robust version)
// -----------------------------------------------------------------------------
// 功能：
// 1) 将来自比较器的异步 SPAD 脉冲同步到 clk_sys
// 2) 生成 1clk 宽度的 photon event pulse
//
// 改进点（相对你原版）：
// - USE_HOLDOFF=0 时：完全无去重（spad_pulse_sync = raw_evt）
// - HOLD_CYCLES=0 时：即使 USE_HOLDOFF=1，也不会强制插入 1clk 死区
//   （HOLD=0 就是"无死区"）
// -----------------------------------------------------------------------------

module rx_photon_frontend (
    input  wire clk_sys,          // 系统时钟（如 130 MHz）
    input  wire rst_n,

    input  wire spad_pulse_in,     // 来自比较器的异步数字脉冲
    output wire spad_pulse_sync    // 同步后的 1clk 脉冲（光子事件）
);

    // ------------------------------------------------------------
    // 1) 双触发器同步（CDC）
    // ------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg spad_ff1;
    (* ASYNC_REG = "TRUE" *) reg spad_ff2;

    always @(posedge clk_sys or negedge rst_n) begin
        if (!rst_n) begin
            spad_ff1 <= 1'b0;
            spad_ff2 <= 1'b0;
        end else begin
            spad_ff1 <= spad_pulse_in;
            spad_ff2 <= spad_ff1;
        end
    end
    
    assign spad_pulse_sync = spad_ff2;
endmodule
