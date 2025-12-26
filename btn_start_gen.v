`timescale 1ns/1ps

// =====================================================================
// btn_start_gen
// 使用板载按键产生"开始一帧"的单周期脉冲 start_pulse
//
// 功能：
// 1) 将异步按键输入同步到 clk 域
// 2) 做 10ms 去抖动，得到稳定按键电平 btn_stable
// 3) 对 btn_stable 的上升沿做检测，输出 1clk 宽度的 start_pulse
//
// 说明：
// - 假设按键为低有效：按下 = 0，松开 = 1
//   如果你的按键是高有效，把 button_level = ~button_sync2 改成 button_level = button_sync2
// =====================================================================
module btn_start_gen #(
    parameter integer CLK_FREQ_HZ     = 50_000_000, // 时钟频率
    parameter integer DEBOUNCE_MS     = 10          // 去抖时间 (ms)
)(
    input  wire clk,
    input  wire rst_n,         // 低有效复位

    input  wire button_in,     // 来自 PL 按键的原始信号
    output wire start_pulse    // 1 个 clk 周期的启动脉冲
);

    // --------------------------------------------------------------
    // 1. 按键同步到 clk 域
    // --------------------------------------------------------------
    reg button_sync1, button_sync2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            button_sync1 <= 1'b1;
            button_sync2 <= 1'b1;
        end else begin
            button_sync1 <= button_in;
            button_sync2 <= button_sync1;
        end
    end

    // 假设按键为低有效：按下=0，松开=1
    // 若高有效，改成 button_level = button_sync2;
    wire button_level = ~button_sync2;

    // --------------------------------------------------------------
    // 2. 去抖：只有当电平持续稳定 DEBOUNCE_MS 才认为状态改变
    // --------------------------------------------------------------
    localparam integer DEBOUNCE_CNT_MAX =
        (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

    localparam integer DEBOUNCE_CNT_BITS =
        $clog2(DEBOUNCE_CNT_MAX+1);

    reg [DEBOUNCE_CNT_BITS-1:0] db_cnt;
    reg                         btn_stable;      // 去抖后的稳定按键电平

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            db_cnt    <= {DEBOUNCE_CNT_BITS{1'b0}};
            btn_stable <= 1'b0;
        end else begin
            // 如果当前读到的按键电平和上一次稳定值不同 → 重新计时
            if (button_level != btn_stable) begin
                if (db_cnt < DEBOUNCE_CNT_MAX[DEBOUNCE_CNT_BITS-1:0]) begin
                    db_cnt <= db_cnt + 1'b1;
                end else begin
                    // 电平已经稳定超过去抖时间 → 更新稳定值
                    btn_stable <= button_level;
                    db_cnt     <= {DEBOUNCE_CNT_BITS{1'b0}};
                end
            end else begin
                // 电平没变，计数清零
                db_cnt <= {DEBOUNCE_CNT_BITS{1'b0}};
            end
        end
    end

    // --------------------------------------------------------------
    // 3. 上升沿检测：由"未按下"到"按下"的瞬间输出 1clk 脉冲
    //    （每按一次键只触发一次 start_pulse）
    // --------------------------------------------------------------
    reg btn_stable_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            btn_stable_d <= 1'b0;
        else
            btn_stable_d <= btn_stable;
    end

    assign start_pulse = btn_stable & ~btn_stable_d;

endmodule
