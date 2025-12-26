`timescale 1ns/1ps

module phase_search_core_dyn #(
    parameter integer NUM_PHASE = 16,
    parameter integer CNT_WIDTH = 16
)(
    input  wire                         clk_fast,
    input  wire                         rst_n,

    input  wire                         enable,
    input  wire                         preamble_end,
    input  wire                         adc_pulse,

    input  wire [7:0]                   div_ratio,     // 动态：每 bit 周期 clk 数

    output wire                         sync_clk,
    output reg                          sync_locked,
    output reg  [$clog2(NUM_PHASE)-1:0] sel_phase_idx,

    output wire [NUM_PHASE-1:0]         dbg_gate_clk,
    output wire [NUM_PHASE*CNT_WIDTH-1:0] dbg_phase_count
);

    localparam integer PHASE_BITS = $clog2(NUM_PHASE);

    integer i, j;

    // enable 上升沿
    reg enable_d;
    wire enable_rise = enable & ~enable_d;
    always @(posedge clk_fast or negedge rst_n) begin
        if (!rst_n) enable_d <= 1'b0;
        else        enable_d <= enable;
    end

    // preamble_end 上升沿
    reg pre_end_d;
    wire pre_end_rise = preamble_end & ~pre_end_d;
    always @(posedge clk_fast or negedge rst_n) begin
        if (!rst_n) pre_end_d <= 1'b0;
        else        pre_end_d <= preamble_end;
    end

    // ============================================================
    // [MOD#1] 门控宽度：从 50% 改为 25%（div_ratio/4）
    //        并加保护：gate_w 至少为 1，避免 div_ratio 很小时变 0
    // ============================================================
    wire [7:0] gate_w_raw = (div_ratio >> 1);               // 25% duty
    wire [7:0] gate_w     = (gate_w_raw == 8'd0) ? 8'd1
                                                  : gate_w_raw;

    // 相位计数器与门控方波
    reg [7:0] phase_cnt [0:NUM_PHASE-1];
    reg [NUM_PHASE-1:0] gate_clk_r;
    assign dbg_gate_clk = gate_clk_r;

    always @(posedge clk_fast or negedge rst_n) begin
        if (!rst_n) begin
            for (i=0; i<NUM_PHASE; i=i+1) begin
                phase_cnt[i]  <= 8'd0;
                gate_clk_r[i] <= 1'b0;
            end
        end else begin
            if (enable_rise) begin
                // 初始化相位偏移：div_ratio * i / NUM_PHASE
                for (i=0; i<NUM_PHASE; i=i+1) begin
                    phase_cnt[i] <= (div_ratio * i) / NUM_PHASE;
                end
            end else begin
                for (i=0; i<NUM_PHASE; i=i+1) begin
                    if (phase_cnt[i] == div_ratio - 1)
                        phase_cnt[i] <= 8'd0;
                    else
                        phase_cnt[i] <= phase_cnt[i] + 1'b1;
                end
            end

            // ====================================================
            // [MOD#2] 门控：从 (div_ratio>>1) 改为 gate_w（≈div_ratio/4）
            // ====================================================
            for (i=0; i<NUM_PHASE; i=i+1) begin
                gate_clk_r[i] <= (phase_cnt[i] < gate_w);
            end
        end
    end

    // 相位计数（光子计数）
    reg [CNT_WIDTH-1:0] phase_count [0:NUM_PHASE-1];

    genvar gi;
    generate
        for (gi=0; gi<NUM_PHASE; gi=gi+1) begin : GEN_FLAT
            assign dbg_phase_count[(gi+1)*CNT_WIDTH-1 : gi*CNT_WIDTH] = phase_count[gi];
        end
    endgenerate

    always @(posedge clk_fast or negedge rst_n) begin
        if (!rst_n) begin
            for (i=0; i<NUM_PHASE; i=i+1)
                phase_count[i] <= {CNT_WIDTH{1'b0}};
        end else begin
            if (enable_rise) begin
                for (i=0; i<NUM_PHASE; i=i+1)
                    phase_count[i] <= {CNT_WIDTH{1'b0}};
            end else if (enable && !sync_locked) begin
                if (adc_pulse) begin
                    for (i=0; i<NUM_PHASE; i=i+1) begin
                        if (gate_clk_r[i]) begin
                            if (phase_count[i] != {CNT_WIDTH{1'b1}})
                                phase_count[i] <= phase_count[i] + 1'b1;
                        end
                    end
                end
            end
        end
    end
    
    // Arm preamble_end edge detection 1 cycle after enable goes high
reg en_armed;
always @(posedge clk_fast or negedge rst_n) begin
    if (!rst_n) begin
        en_armed <= 1'b0;
    end else if (!enable) begin
        en_armed <= 1'b0;
    end else if (enable_rise) begin
        en_armed <= 1'b0;   // first cycle after enable: NOT armed
    end else begin
        en_armed <= 1'b1;   // from second cycle onward: armed
    end
end


    // 选最大计数相位
    reg [CNT_WIDTH-1:0]  max_val;
    reg [PHASE_BITS-1:0] max_idx;

    always @(posedge clk_fast or negedge rst_n) begin
        if (!rst_n) begin
            sync_locked   <= 1'b0;
            sel_phase_idx <= {PHASE_BITS{1'b0}};
        end else begin
            if (enable_rise) begin
                sync_locked   <= 1'b0;
                sel_phase_idx <= {PHASE_BITS{1'b0}};
            end else if (pre_end_rise && enable && en_armed) begin
                max_val = {CNT_WIDTH{1'b0}};
                max_idx = {PHASE_BITS{1'b0}};

                for (j=0; j<NUM_PHASE; j=j+1) begin
                    if (phase_count[j] > max_val) begin
                        max_val = phase_count[j];
                        max_idx = j[PHASE_BITS-1:0];
                    end
                end

                sel_phase_idx <= max_idx;
                sync_locked   <= 1'b1;
            end
        end
    end

    wire [NUM_PHASE-1:0] gate_clk_bus = gate_clk_r;
    assign sync_clk = sync_locked ? gate_clk_bus[sel_phase_idx] : gate_clk_bus[0];

endmodule
