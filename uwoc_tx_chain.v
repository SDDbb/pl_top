`timescale 1ns/1ps

module uwoc_tx_chain #(
    parameter integer FIFO_DEPTH = 700_000,

    // TX 参考缓存（只缓存前 REF_N bit）
    parameter integer REF_N  = 16384,
    parameter integer REF_AW = 14      // 2^14=16384
)(
    input  wire        clk_130M,    // 130 MHz
    input  wire        rst_n,
    input  wire        start,       // 1clk pulse（已去抖）

    input  wire [2:0]  rate_sel,    // ★ 唯一速率控制

    output wire        tx_bit,
    output wire        tx_busy,
    output wire        tx_done,

    // -------- Debug --------
    output wire        dbg_bit_tick,
    output wire        dbg_tx_bit,
    output wire        dbg_prbs_vld,
    output wire        dbg_fifo_re,

    // -------- TX ref RAM debug read (可接 ILA/VIO/PS) --------
    input  wire [REF_AW-1:0] dbg_ref_raddr,
    output wire              dbg_ref_rdata
);

    //====================================================
    // 2. bitrate tick generator
    //====================================================
    wire bit_tick;

    bitrate_tick_gen u_tick (
        .clk      (clk_130M),
        .rst_n    (rst_n ),
        .rate_sel (rate_sel),
        .enable   (tx_busy),
        .bit_tick (bit_tick)
    );

    //====================================================
    // 3. PRBS generator (bit_tick driven)
    //    ★更新：增加 ref_we/ref_waddr/ref_wdata
    //====================================================
    wire prbs_data, prbs_valid, prbs_done;

    wire              tx_ref_we;
    wire [REF_AW-1:0] tx_ref_waddr;
    wire              tx_ref_wdata;

    prbs_gen u_prbs (
        .clk        (clk_130M),
        .rst_n      (rst_n),
        .start      (start),
        .bit_tick   (bit_tick),
        .rate_sel   (rate_sel),
        .prbs_sel   (2'b00),

        .prbs_data  (prbs_data),
        .valid      (prbs_valid),
        .done       (prbs_done),

        // 新增端口：参考缓存写口（只写前 REF_N bit）
        .ref_we     (tx_ref_we),
        .ref_waddr  (tx_ref_waddr),
        .ref_wdata  (tx_ref_wdata)
    );

    //====================================================
    // 3.1 TX ref small window RAM (first REF_N bits)
    //====================================================
simple_bit_ram #(
    .DEPTH  (REF_N),
    .ADDR_W (REF_AW)
) u_tx_ref_ram (
    .clk    (clk_130M),
    .we     (tx_ref_we),
    .waddr  (tx_ref_waddr),
    .wdata  (tx_ref_wdata),
    .raddr  (dbg_ref_raddr),
    .rdata  (dbg_ref_rdata)
);


    //====================================================
    // 4. FIFO (同步 FIFO)
    //====================================================
    wire fifo_empty;
    wire fifo_re;
    wire fifo_data_out;

    fifo_1 #(
        .FIFO_DEPTH(FIFO_DEPTH),
        .WIDTH(1)
    ) u_fifo (
        .clk_in    (clk_130M),
        .rst_in_n  (rst_n),
        .we        (prbs_valid),
        .data_in   (prbs_data),
        .full      (),

        .clk_out   (clk_130M),
        .rst_out_n (rst_n),
        .re        (fifo_re),
        .data_out  (fifo_data_out),
        .empty     (fifo_empty)
    );

    //====================================================
    // 5. Frame builder (bit_tick driven FSM)
    //====================================================
    frame_builder u_frame (
        .clk        (clk_130M),
        .rst_n      (rst_n),
        .start      (start),
        .bit_tick   (bit_tick),
        .rate_sel   (rate_sel),

        .fifo_data  (fifo_data_out),
        .fifo_empty (fifo_empty),
        .fifo_re    (fifo_re),

        .tx_bit     (tx_bit),
        .busy       (tx_busy),
        .done       (tx_done)
    );

    //====================================================
    // Debug assigns
    //====================================================
    assign dbg_bit_tick = bit_tick;
    assign dbg_tx_bit   = tx_bit;
    assign dbg_prbs_vld = prbs_valid;
    assign dbg_fifo_re  = fifo_re;

endmodule
