`timescale 1ns/1ps

module pl_top (
    input  wire sys_clk,     // 50 MHz
    input  wire rst_n,
    input  wire btn_start,    // 低有效按键

    output wire tx_bit,       // TX -> LED driver
    input  wire spad_cmp_in   // comparator -> FPGA IO (async)
);

    // =========================================================================
    // 1) button -> start pulse in 50M
    // =========================================================================
    wire start_50m;

    btn_start_gen #(
        .CLK_FREQ_HZ (50_000_000),
        .DEBOUNCE_MS (10)
    ) u_btn (
        .clk         (sys_clk),
        .rst_n       (rst_n),
        .button_in   (btn_start),
        .start_pulse (start_50m)
    );

    // =========================================================================
    // 2) clock: 50M -> 130M
    // =========================================================================
    wire clk_130M;
    wire clk_locked;

    clk_wiz_0 u_clk (
        .clk_in1  (sys_clk),
        .reset    (~rst_n),
        .clk_130M (clk_130M),
        .locked   (clk_locked)
    );

    // =========================================================================
    // 3) start CDC: 50M -> 130M
    // =========================================================================
    wire start_130m;
    wire rst_130m_n = rst_n & clk_locked;

    cdc_pulse_sync u_start_cdc (
        .src_clk    (sys_clk),
        .src_rst_n  (rst_n),
        .src_pulse  (start_50m),

        .dst_clk    (clk_130M),
        .dst_rst_n  (rst_130m_n),
        .dst_pulse  (start_130m)
    );

    // =========================================================================
    // 4) rate_sel (TX/RX must match)
    // =========================================================================
    wire [2:0] rate_sel = 3'd0; // 2 Mbps

    // =========================================================================
    // 5) TX
    // =========================================================================
    wire tx_busy, tx_done;
    wire bit_tick, prbs_valid, fifo_re;

    // =========================================================================
    // TX ref RAM debug read (rolling address when idle)
    // =========================================================================
    localparam integer TX_REF_AW = 14;  // must match uwoc_tx_chain REF_AW
    wire [TX_REF_AW-1:0] tx_ref_raddr;
    wire                 tx_ref_rdata;
    reg  [TX_REF_AW-1:0] tx_ref_raddr_r;

    assign tx_ref_raddr = tx_ref_raddr_r;

    always @(posedge clk_130M or negedge rst_130m_n) begin
        if (!rst_130m_n) begin
            tx_ref_raddr_r <= {TX_REF_AW{1'b0}};
        end else if (!tx_busy) begin
            // idle: rolling read
            tx_ref_raddr_r <= tx_ref_raddr_r + {{(TX_REF_AW-1){1'b0}}, 1'b1};
        end
    end

    wire dbg_tx_bit_dummy;

    uwoc_tx_chain u_tx (
        .clk_130M      (clk_130M),
        .rst_n         (rst_130m_n),
        .start         (start_130m),
        .rate_sel      (rate_sel),

        .tx_bit        (tx_bit),
        .tx_busy       (tx_busy),
        .tx_done       (tx_done),

        .dbg_bit_tick  (bit_tick),
        .dbg_tx_bit    (dbg_tx_bit_dummy),

        .dbg_prbs_vld  (prbs_valid),
        .dbg_fifo_re   (fifo_re),

        // TX reference RAM debug read
        .dbg_ref_raddr (tx_ref_raddr),
        .dbg_ref_rdata (tx_ref_rdata)
    );

    // =========================================================================
    // 6) RX window control
    // =========================================================================
    localparam [31:0] RX_DELAY_CYCLES  = 32'd0;
    localparam [31:0] RX_WINDOW_CYCLES = 32'd1_560_000;  // 12ms @130MHz

    wire rx_en;
    wire rx_start;
    wire rx_done;
    wire rx_ctrl_busy;

    rx_window_ctrl u_rx_win (
        .clk           (clk_130M),
        .rst_n         (rst_130m_n),
        .start_pulse   (start_130m),

        .delay_cycles  (RX_DELAY_CYCLES),
        .window_cycles (RX_WINDOW_CYCLES),

        .rx_en         (rx_en),
        .rx_start      (rx_start),
        .rx_done       (rx_done),
        .busy          (rx_ctrl_busy)
    );

    // =========================================================================
    // 7) RX chain: uwoc_rx_chain_gpio
    // =========================================================================
    localparam integer NUM_PHASE   = 16;
    localparam integer CNT_WIDTH   = 16;
    localparam integer PHASE_IDX_W = 4;

    wire                     rx_sync_clk;
    wire                     rx_sync_locked;
    wire [PHASE_IDX_W-1:0]   rx_sel_phase_idx;

    wire                     rx_bit;
    wire                     rx_bit_vld;

    wire                     dbg_spad_pulse_sync;
    wire [NUM_PHASE-1:0]     dbg_gate_clk;
    wire [NUM_PHASE*CNT_WIDTH-1:0] dbg_phase_count;

    wire                     frame_sync_ok;
    wire                     frame_sync_fail;

    wire [15:0]              dbg_pre_cnt;
    wire                     dbg_preamble_active;
    wire                     dbg_preamble_end;
    wire                     dbg_sync_locked_core;

    wire [255:0]             dbg_frame_pack;

    uwoc_rx_chain_gpio #(
        .NUM_PHASE (NUM_PHASE),
        .CNT_WIDTH (CNT_WIDTH)
    ) u_rx_chain (
        .clk_130M            (clk_130M),
        .rst_n               (rst_130m_n),

        .rx_en               (rx_en),
        .rx_start            (rx_start),
        .rate_sel            (rate_sel),

        .spad_cmp_in         (spad_cmp_in),

        .sync_clk            (rx_sync_clk),
        .sync_locked         (rx_sync_locked),
        .sel_phase_idx       (rx_sel_phase_idx),

        .rx_bit              (rx_bit),
        .rx_bit_vld          (rx_bit_vld),

        .dbg_spad_pulse_sync (dbg_spad_pulse_sync),
        .dbg_gate_clk        (dbg_gate_clk),
        .dbg_phase_count     (dbg_phase_count),

        .frame_sync_ok       (frame_sync_ok),
        .frame_sync_fail     (frame_sync_fail),

        .dbg_pre_cnt         (dbg_pre_cnt),
        .dbg_preamble_active (dbg_preamble_active),
        .dbg_preamble_end    (dbg_preamble_end),
        .dbg_sync_locked_core(dbg_sync_locked_core),

        .dbg_frame_pack      (dbg_frame_pack)
    );

    // =========================================================================
    // 8) RX data capture + BER statistics (online compare + small caches)
    // =========================================================================
    wire [15:0] pulses_in_bit = dbg_frame_pack[71:56];

    localparam [1:0] PRBS_SEL = 2'b11; // keep same as TX (default PRBS31)

    localparam integer RX_FIRST_N = 8192;
    localparam integer RX_ERR_WIN = 256;
    localparam integer RX_ERR_PRE = 96;
    localparam integer RX_ERR_POST= 96;

    wire                    rx_first_we;
    wire [$clog2(RX_FIRST_N)-1:0] rx_first_waddr;
    wire                    rx_first_wdata;

    wire                    rx_err_we;
    wire [$clog2(RX_ERR_WIN)-1:0] rx_err_waddr;
    wire                    rx_err_wdata;

    wire                    cap_active;
    wire                    rx_data_done;

    wire [31:0]             ber_total_bits;
    wire [31:0]             ber_err_bits;
    wire [31:0]             ber_frame_cnt;
    wire [31:0]             ber_frame_err_cnt;

    wire [15:0]             max_consecutive_err;
    wire [15:0]             err_run_hist0, err_run_hist1, err_run_hist2, err_run_hist3;
    wire [15:0]             err_run_hist4, err_run_hist5, err_run_hist6, err_run_hist7;

    wire [47:0]             sum_pulses_total, sum_pulses_ok, sum_pulses_err;
    wire [31:0]             bits_ok, bits_err;

    wire                    err_latched;
    wire [31:0]             err_bit_index;
    wire [$clog2(RX_ERR_WIN)-1:0] err_base_ptr;

    rx_frame_capture #(
        .SILENCE_BITS (100),
        .HEADER_BITS  (384),
        .FIRST_N      (RX_FIRST_N),
        .ERR_WIN      (RX_ERR_WIN),
        .ERR_PRE      (RX_ERR_PRE),
        .ERR_POST     (RX_ERR_POST)
    ) u_rx_cap (
        .clk            (clk_130M),
        .rst_n          (rst_130m_n),

        .rx_en          (rx_en),
        .rate_sel       (rate_sel),
        .prbs_sel       (PRBS_SEL),

        .frame_sync_ok  (frame_sync_ok),
        .rx_bit         (rx_bit),
        .rx_bit_vld     (rx_bit_vld),
        .pulses_in_bit  (pulses_in_bit),

        .first_we       (rx_first_we),
        .first_waddr    (rx_first_waddr),
        .first_wdata    (rx_first_wdata),

        .err_we         (rx_err_we),
        .err_waddr      (rx_err_waddr),
        .err_wdata      (rx_err_wdata),

        .cap_active     (cap_active),
        .data_done      (rx_data_done),

        .total_bits     (ber_total_bits),
        .err_bits       (ber_err_bits),
        .frame_cnt      (ber_frame_cnt),
        .frame_err_cnt  (ber_frame_err_cnt),

        .max_consecutive_err (max_consecutive_err),
        .err_run_hist0  (err_run_hist0),
        .err_run_hist1  (err_run_hist1),
        .err_run_hist2  (err_run_hist2),
        .err_run_hist3  (err_run_hist3),
        .err_run_hist4  (err_run_hist4),
        .err_run_hist5  (err_run_hist5),
        .err_run_hist6  (err_run_hist6),
        .err_run_hist7  (err_run_hist7),

        .sum_pulses_total (sum_pulses_total),
        .sum_pulses_ok    (sum_pulses_ok),
        .sum_pulses_err   (sum_pulses_err),
        .bits_ok          (bits_ok),
        .bits_err         (bits_err),

        .err_latched    (err_latched),
        .err_bit_index  (err_bit_index),
        .err_base_ptr   (err_base_ptr)
    );

    wire rx_first_rdata, rx_err_rdata;

    reg [$clog2(RX_FIRST_N)-1:0] rx_first_raddr;
    reg [$clog2(RX_ERR_WIN)-1:0] rx_err_raddr;

    wire [$clog2(RX_ERR_WIN)-1:0] rx_err_phys_raddr = err_base_ptr + rx_err_raddr;

    simple_bit_ram #(
        .DEPTH  (RX_FIRST_N),
        .ADDR_W ($clog2(RX_FIRST_N))
    ) u_rx_first_ram (
        .clk   (clk_130M),
        .we    (rx_first_we),
        .waddr (rx_first_waddr),
        .wdata (rx_first_wdata),
        .raddr (rx_first_raddr),
        .rdata (rx_first_rdata)
    );

    simple_bit_ram #(
        .DEPTH  (RX_ERR_WIN),
        .ADDR_W ($clog2(RX_ERR_WIN))
    ) u_rx_err_ram (
        .clk   (clk_130M),
        .we    (rx_err_we),
        .waddr (rx_err_waddr),
        .wdata (rx_err_wdata),
        .raddr (rx_err_phys_raddr),
        .rdata (rx_err_rdata)
    );

    // free-running read address (便于 ILA 看缓存内容)
    always @(posedge clk_130M or negedge rst_130m_n) begin
        if (!rst_130m_n) begin
            rx_first_raddr <= {($clog2(RX_FIRST_N)){1'b0}};
            rx_err_raddr   <= {($clog2(RX_ERR_WIN)){1'b0}};
        end else begin
            if (!cap_active) begin
                rx_first_raddr <= rx_first_raddr + 1'b1;
                rx_err_raddr   <= rx_err_raddr + 1'b1;
            end
        end
    end

    // =========================================================================
    // RAM验收模式：把 ILA 的两个16-bit debug口改成"读地址+读数据"
    // =========================================================================
    // probe9 : {tx_done, tx_busy, tx_ref_rdata, padding}
    wire [15:0] cap_dbg16 = {
        8'h00,        // padding
        tx_done,      // [7]
        tx_busy,      // [6]
        tx_ref_rdata, // [5]
        5'b0          // [4:0]
    };

    // probe10: {tx_done, tx_busy, tx_ref_raddr[13:0]}
    wire [15:0] ber_dbg16 = {
        tx_done,      // [15]
        tx_busy,      // [14]
        tx_ref_raddr  // [13:0]
    };

    // =========================================================================
    // 9) RX phase count debug compression (LIGHT) - Pure Verilog
    // =========================================================================
    function [CNT_WIDTH-1:0] get_cnt;
        input [PHASE_IDX_W-1:0] idx;
        begin
            case (idx)
                4'd0:  get_cnt = dbg_phase_count[ 15:  0];
                4'd1:  get_cnt = dbg_phase_count[ 31: 16];
                4'd2:  get_cnt = dbg_phase_count[ 47: 32];
                4'd3:  get_cnt = dbg_phase_count[ 63: 48];
                4'd4:  get_cnt = dbg_phase_count[ 79: 64];
                4'd5:  get_cnt = dbg_phase_count[ 95: 80];
                4'd6:  get_cnt = dbg_phase_count[111: 96];
                4'd7:  get_cnt = dbg_phase_count[127:112];
                4'd8:  get_cnt = dbg_phase_count[143:128];
                4'd9:  get_cnt = dbg_phase_count[159:144];
                4'd10: get_cnt = dbg_phase_count[175:160];
                4'd11: get_cnt = dbg_phase_count[191:176];
                4'd12: get_cnt = dbg_phase_count[207:192];
                4'd13: get_cnt = dbg_phase_count[223:208];
                4'd14: get_cnt = dbg_phase_count[239:224];
                4'd15: get_cnt = dbg_phase_count[255:240];
                default: get_cnt = {CNT_WIDTH{1'b0}};
            endcase
        end
    endfunction

    reg [CNT_WIDTH-1:0] phase_max_cnt;
    reg [CNT_WIDTH-1:0] phase_second_cnt;
    reg [PHASE_IDX_W-1:0] phase_max_idx;

    integer k;
    reg [CNT_WIDTH-1:0] c;

    always @(*) begin
        phase_max_cnt    = get_cnt(4'd0);
        phase_second_cnt = {CNT_WIDTH{1'b0}};
        phase_max_idx    = 4'd0;

        for (k = 1; k < NUM_PHASE; k = k + 1) begin
            c = get_cnt(k[PHASE_IDX_W-1:0]);

            if (c > phase_max_cnt) begin
                phase_second_cnt = phase_max_cnt;
                phase_max_cnt    = c;
                phase_max_idx    = k[PHASE_IDX_W-1:0];
            end else if (c > phase_second_cnt) begin
                phase_second_cnt = c;
            end
        end
    end

    wire [CNT_WIDTH-1:0] phase_margin = phase_max_cnt - phase_second_cnt;

    localparam [CNT_WIDTH-1:0] PHASE_MARGIN_TH = 16'd50;
    wire phase_peak_good = (phase_margin >= PHASE_MARGIN_TH);

    // =========================================================================
    // 10) ILA (LIGHT, 16 probes)
    // =========================================================================
    ila_0 u_ila_rx_light (
        .clk (clk_130M),

        .probe0  (start_130m),
        .probe1  (rx_en),
        .probe2  (rx_start),
        .probe3  (rx_done),

        .probe4  (dbg_spad_pulse_sync),
        .probe5  (rx_sync_locked),
        .probe6  (phase_peak_good),

        .probe7  (phase_max_idx),
        .probe8  (phase_max_cnt),
        .probe9  (cap_dbg16),
        .probe10 (ber_dbg16),

        .probe11 (rx_bit_vld),
        .probe12 (rx_bit),
        .probe13 (frame_sync_ok),
        .probe14 (frame_sync_fail),
        .probe15 (clk_locked)
    );

endmodule
