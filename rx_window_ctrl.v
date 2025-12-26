`timescale 1ns/1ps

module rx_window_ctrl #(
    parameter integer CNT_W = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire                  start_pulse,     // 1clk pulse (130M域)
    input  wire [CNT_W-1:0]      delay_cycles,    // 延迟: clk 周期数
    input  wire [CNT_W-1:0]      window_cycles,   // 窗口长度: clk 周期数

    output reg                   rx_en,           // 接收窗口使能(电平)
    output reg                   rx_start,        // 1clk pulse: 窗口开始/启动preamble计数
    output reg                   rx_done,         // 1clk pulse: 窗口结束
    output reg                   busy
);

    localparam [1:0] ST_IDLE  = 2'd0;
    localparam [1:0] ST_DELAY = 2'd1;
    localparam [1:0] ST_WIN   = 2'd2;

    reg [1:0] state;

    reg [CNT_W-1:0] delay_cnt;
    reg [CNT_W-1:0] win_cnt;

    wire delay_done = (delay_cycles == {CNT_W{1'b0}}) ? 1'b1
                      : (delay_cnt == delay_cycles - 1);

    wire win_done   = (window_cycles == {CNT_W{1'b0}}) ? 1'b1
                      : (win_cnt == window_cycles - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            delay_cnt <= {CNT_W{1'b0}};
            win_cnt   <= {CNT_W{1'b0}};
            rx_en     <= 1'b0;
            rx_start  <= 1'b0;
            rx_done   <= 1'b0;
            busy      <= 1'b0;
        end else begin
            rx_start <= 1'b0;
            rx_done  <= 1'b0;

            case (state)
                ST_IDLE: begin
                    rx_en <= 1'b0;
                    busy  <= 1'b0;
                    delay_cnt <= {CNT_W{1'b0}};
                    win_cnt   <= {CNT_W{1'b0}};

                    if (start_pulse) begin
                        busy <= 1'b1;

                        if (delay_cycles == {CNT_W{1'b0}}) begin
                            state    <= ST_WIN;
                            rx_en    <= 1'b1;
                            rx_start <= 1'b1;
                            win_cnt  <= {CNT_W{1'b0}};
                        end else begin
                            state     <= ST_DELAY;
                            delay_cnt <= {CNT_W{1'b0}};
                        end
                    end
                end

                ST_DELAY: begin
                    busy <= 1'b1;
                    rx_en <= 1'b0;

                    if (delay_done) begin
                        state    <= ST_WIN;
                        rx_en    <= 1'b1;
                        rx_start <= 1'b1;
                        win_cnt  <= {CNT_W{1'b0}};
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                ST_WIN: begin
                    busy <= 1'b1;
                    rx_en <= 1'b1;

                    if (win_done) begin
                        rx_en   <= 1'b0;
                        rx_done <= 1'b1;
                        state   <= ST_IDLE;
                        busy    <= 1'b0;
                    end else begin
                        win_cnt <= win_cnt + 1'b1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
