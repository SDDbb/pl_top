`timescale 1ns/1ps

module frame_builder #(
    parameter integer HEADER_BITS = 384,
    parameter [HEADER_BITS-1:0] HEADER_PATTERN = {HEADER_BITS{1'b1}}
)(
    input  wire        clk,        // clk_130M
    input  wire        rst_n,
    input  wire        start,       // 1clk pulse
    input  wire        bit_tick,    // 1clk pulse = 1 bit
    input  wire [2:0]  rate_sel,    // ★ 唯一速率控制

    // FIFO 接口
    input  wire        fifo_data,
    input  wire        fifo_empty,
    output reg         fifo_re,

    // 输出
    output reg         tx_bit,
    output reg         busy,
    output reg         done
);

    // ------------------------------------------------
    // 固定字段长度（bit 数）
    // ------------------------------------------------
    localparam integer START_BITS   = 100;
    localparam integer SILENCE_BITS = 100;
    localparam integer SYNC_BITS    = 144;
    localparam integer FILL_BITS    = 100;

    // ------------------------------------------------
    // DATA_BITS：由 rate_sel 决定（10ms）
    // ------------------------------------------------
    function automatic [31:0] data_bits_by_rate(input [2:0] r);
        begin
            case (r)
                3'd0: data_bits_by_rate = 32'd20_000;   // 2 Mbps
                3'd1: data_bits_by_rate = 32'd40_000;   // 4 Mbps
                3'd2: data_bits_by_rate = 32'd80_000;   // 8 Mbps
                3'd3: data_bits_by_rate = 32'd160_000;  // 16 Mbps
                3'd4: data_bits_by_rate = 32'd320_000;  // 32 Mbps
                3'd5: data_bits_by_rate = 32'd650_000;  // 65 Mbps
                default: data_bits_by_rate = 32'd20_000;
            endcase
        end
    endfunction

    // ------------------------------------------------
    // FSM
    // ------------------------------------------------
    localparam [3:0]
        S_IDLE   = 4'd0,
        S_START  = 4'd1,
        S_SIL1   = 4'd2,
        S_SYNC   = 4'd3,
        S_SIL2   = 4'd4,
        S_HEADER = 4'd5,
        S_SIL3   = 4'd6,
        S_DATA   = 4'd7,
        S_FILL   = 4'd8,
        S_DONE   = 4'd9;

    reg [3:0]  state;
    reg [31:0] bit_cnt;
    reg        start_bit;

    // ------------------------------------------------
    // 字段长度查询
    // ------------------------------------------------
    function automatic [31:0] field_len(input [3:0] st);
        begin
            case (st)
                S_START : field_len = START_BITS;
                S_SIL1  : field_len = SILENCE_BITS;
                S_SYNC  : field_len = SYNC_BITS;
                S_SIL2  : field_len = SILENCE_BITS;
                S_HEADER: field_len = HEADER_BITS;
                S_SIL3  : field_len = SILENCE_BITS;
                S_DATA  : field_len = data_bits_by_rate(rate_sel);
                S_FILL  : field_len = FILL_BITS;
                default : field_len = 32'd0;
            endcase
        end
    endfunction

    // ------------------------------------------------
    // Pattern helpers
    // ------------------------------------------------
    localparam [15:0] SYNC_PATTERN = 16'hAABB;
    wire [3:0] sync_idx   = bit_cnt[3:0];
    wire       sync_bit   = SYNC_PATTERN[15 - sync_idx];
    wire       header_bit = HEADER_PATTERN[HEADER_BITS-1 - bit_cnt];

    // ------------------------------------------------
    // 主时序（严格 bit_tick 驱动）
    // ------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            bit_cnt   <= 32'd0;
            start_bit <= 1'b0;
            tx_bit    <= 1'b0;
            busy      <= 1'b0;
            done      <= 1'b0;
            fifo_re   <= 1'b0;
        end else begin
            done    <= 1'b0;
            fifo_re <= 1'b0;

            if (state == S_IDLE) begin
                busy    <= 1'b0;
                tx_bit  <= 1'b0;
                bit_cnt <= 32'd0;
                start_bit <= 1'b0;

                if (start) begin
                    state <= S_START;
                    busy  <= 1'b1;
                end
            end else begin
                busy <= 1'b1;

                if (bit_tick) begin
                    // 输出当前 bit
                    case (state)
                        S_START: begin
                            tx_bit    <= start_bit;
                            start_bit <= ~start_bit;
                        end
                        S_SIL1, S_SIL2, S_SIL3, S_FILL:
                            tx_bit <= 1'b0;
                        S_SYNC:
                            tx_bit <= sync_bit;
                        S_HEADER:
                            tx_bit <= header_bit;
                        S_DATA: begin
                            fifo_re <= 1'b1;
                            tx_bit  <= fifo_empty ? 1'b0 : fifo_data;
                        end
                        default:
                            tx_bit <= 1'b0;
                    endcase

                    // bit 计数 & 状态跳转
                    if (bit_cnt == field_len(state) - 1) begin
                        bit_cnt <= 32'd0;
                        case (state)
                            S_START : state <= S_SIL1;
                            S_SIL1  : state <= S_SYNC;
                            S_SYNC  : state <= S_SIL2;
                            S_SIL2  : state <= S_HEADER;
                            S_HEADER: state <= S_SIL3;
                            S_SIL3  : state <= S_DATA;
                            S_DATA  : state <= S_FILL;
                            S_FILL  : state <= S_DONE;
                            default : state <= S_IDLE;
                        endcase
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end

                if (state == S_DONE) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    state <= S_IDLE;
                end
            end
        end
    end

endmodule
