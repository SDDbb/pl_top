`timescale 1ns/1ps

module uwoc_rx_chain_gpio #(
    parameter integer NUM_PHASE      = 16,
    parameter integer CNT_WIDTH      = 16,

    // align with frame_builder.v
    parameter integer START_BITS     = 100,
    parameter integer SILENCE_BITS   = 100,   // 前使茫
    parameter integer SYNC_BITS      = 144,   // 9 * 16
    parameter [15:0]  SYNC_PATTERN   = 16'hAABB
)(

    input  wire        clk_130M,
    input  wire        rst_n,

    input  wire        rx_en,
    input  wire        rx_start,
    input  wire [2:0]  rate_sel,

    input  wire        spad_cmp_in,

    output wire        sync_clk,
    output reg         sync_locked,
    output wire [$clog2(NUM_PHASE)-1:0] sel_phase_idx,

    output reg         rx_bit,
    output reg         rx_bit_vld,

    output wire        dbg_spad_pulse_sync,
    output wire [NUM_PHASE-1:0] dbg_gate_clk,
    output wire [NUM_PHASE*CNT_WIDTH-1:0] dbg_phase_count,

    output reg         frame_sync_ok,
    output reg         frame_sync_fail,

    // ---- debug taps (preamble/lock) ----
    output wire [15:0] dbg_pre_cnt,
    output wire        dbg_preamble_active,
    output wire        dbg_preamble_end,
    output wire        dbg_sync_locked_core,

    // ---- packed debug bus ----
    output wire [255:0] dbg_frame_pack
);

    //============================================================
    // rate_sel -> div_val (must match bitrate_tick_gen)
    //============================================================
    reg [7:0] div_val;
    always @(*) begin
        case (rate_sel)
            3'd0: div_val = 8'd64;  // 2 Mbps @130MHz
            3'd1: div_val = 8'd32;
            3'd2: div_val = 8'd16;
            3'd3: div_val = 8'd8;
            3'd4: div_val = 8'd4;
            3'd5: div_val = 8'd2;
            default: div_val = 8'd64;
        endcase
    end

    //============================================================
    // PULSE threshold by rate_sel
    // - bit time = div_val * (1/130MHz)
    // - assume one photon event occupies ~25ns, so max events/bit 鈮? floor(Tbit/25ns)
    // - threshold = half of max (rounded up), clamped to at least 1
    //============================================================
    function automatic [15:0] pulse_thr_by_rate(input [2:0] r);
        begin
            case (r)
                3'd0: pulse_thr_by_rate = 16'd8; // div64: T鈮?492.3ns, max鈮?19, half->10
                3'd1: pulse_thr_by_rate = 16'd4;  // div32: T鈮?246.2ns, max鈮?9,  half->5
                3'd2: pulse_thr_by_rate = 16'd2;  // div16: T鈮?123.1ns, max鈮?4,  half->2
                3'd3: pulse_thr_by_rate = 16'd1;  // div8 : T鈮?61.5ns,  max鈮?2,  half->1
                3'd4: pulse_thr_by_rate = 16'd1;  // div4 : T鈮?30.8ns,  max鈮?1,  half->1
                3'd5: pulse_thr_by_rate = 16'd1;  // div2 : T鈮?15.4ns,  max<1 => clamp to 1
                default: pulse_thr_by_rate = 16'd10;
            endcase
        end
    endfunction

    wire [15:0] pulse_thr = pulse_thr_by_rate(rate_sel);



    //============================================================
    // 1) photon frontend
    //============================================================
    rx_photon_frontend u_frontend (
        .clk_sys         (clk_130M),
        .rst_n           (rst_n),
        .spad_pulse_in   (spad_cmp_in),
        .spad_pulse_sync (dbg_spad_pulse_sync)
    );
    wire spad_pulse = dbg_spad_pulse_sync;

    //============================================================
    // 2) preamble window: START_BITS * div_val cycles
    //============================================================
    reg        preamble_active;
    reg        preamble_end;
    reg [31:0] pre_cnt;

    reg        rx_start_d;
    wire       rx_start_rise = rx_start & ~rx_start_d;

    reg        pre_end_hold;

    wire [31:0] PRE_CYCLES = START_BITS * div_val;

    always @(posedge clk_130M or negedge rst_n) begin
        if (!rst_n) rx_start_d <= 1'b0;
        else        rx_start_d <= rx_start;
    end

    always @(posedge clk_130M or negedge rst_n) begin
        if (!rst_n) begin
            preamble_active <= 1'b0;
            preamble_end    <= 1'b0;
            pre_cnt         <= 32'd0;
            pre_end_hold    <= 1'b0;
        end else begin
            preamble_end <= 1'b0; // 1-cycle pulse

            if (!rx_en) begin
                preamble_active <= 1'b0;
                pre_cnt         <= 32'd0;
                pre_end_hold    <= 1'b0;

            end else if (rx_start_rise) begin
                preamble_active <= 1'b1;
                pre_cnt         <= 32'd0;
                pre_end_hold    <= 1'b0;

            end else if (preamble_active) begin
                if (pre_end_hold) begin
                    preamble_active <= 1'b0;
                    pre_end_hold    <= 1'b0;
                end else begin
                    if (pre_cnt == (PRE_CYCLES - 1)) begin
                        preamble_end <= 1'b1;
                        pre_end_hold <= 1'b1;
                    end else begin
                        pre_cnt <= pre_cnt + 1'b1;
                    end
                end
            end
        end
    end

    assign dbg_pre_cnt         = pre_cnt[15:0];
    assign dbg_preamble_active = preamble_active;
    assign dbg_preamble_end    = preamble_end;

    //============================================================
    // 3) phase search
    //============================================================
    wire sync_locked_core;

    phase_search_core_dyn #(
        .NUM_PHASE (NUM_PHASE),
        .CNT_WIDTH (CNT_WIDTH)
    ) u_phase_search (
        .clk_fast        (clk_130M),
        .rst_n           (rst_n),
        .enable          (preamble_active),
        .preamble_end    (preamble_end),
        .adc_pulse       (spad_pulse),
        .div_ratio       (div_val),

        .sync_clk        (sync_clk),
        .sync_locked     (sync_locked_core),
        .sel_phase_idx   (sel_phase_idx),

        .dbg_gate_clk    (dbg_gate_clk),
        .dbg_phase_count (dbg_phase_count)
    );

    assign dbg_sync_locked_core = sync_locked_core;

    // 越锥危frame_sync_fail 原呒
    always @(posedge clk_130M or negedge rst_n) begin
        if (!rst_n) begin
            sync_locked <= 1'b0;
        end else begin
            if (!rx_en) begin
                sync_locked <= 1'b0;
            end else if (frame_sync_fail) begin
                sync_locked <= sync_locked; // 裕
            end else begin
                sync_locked <= sync_locked_core;
            end
        end
    end

    //============================================================
    // 4) bit decision by counting pulses per recovered bit window
    //============================================================
    reg sync_clk_d;
    wire sync_clk_rise = sync_clk & ~sync_clk_d;

    always @(posedge clk_130M or negedge rst_n) begin
        if (!rst_n) sync_clk_d <= 1'b0;
        else        sync_clk_d <= sync_clk;
    end

    reg [15:0] pulses_in_bit;
    reg        sync_locked_d;
    reg        align_wait; // discard 1st partial window after lock

    always @(posedge clk_130M or negedge rst_n) begin
        if (!rst_n) begin
            pulses_in_bit <= 16'd0;
            rx_bit        <= 1'b0;
            rx_bit_vld    <= 1'b0;
            sync_locked_d <= 1'b0;
            align_wait    <= 1'b0;
        end else begin
            rx_bit_vld    <= 1'b0;
            sync_locked_d <= sync_locked;

            if (!sync_locked) begin
                pulses_in_bit <= 16'd0;
                align_wait    <= 1'b0;
            end else begin
                if (!sync_locked_d && sync_locked) begin
                    pulses_in_bit <= 16'd0;
                    align_wait    <= 1'b1;
                end

                if (spad_pulse) begin
                    if (pulses_in_bit != 16'hFFFF)
                        pulses_in_bit <= pulses_in_bit + 1'b1;
                end

                if (sync_clk_rise) begin
                    if (align_wait) begin
                        pulses_in_bit <= 16'd0;
                        align_wait    <= 1'b0;
                    end else begin
                        rx_bit        <= (pulses_in_bit >= pulse_thr);
                        rx_bit_vld    <= 1'b1;
                        pulses_in_bit <= 16'd0;
                    end
                end
            end
        end
    end

    //============================================================
    // 5) Frame sync: require 9 consecutive 16-bit AABB (=144 bits)
    //============================================================
    localparam [2:0] ST_IDLE   = 3'd0,
                     ST_SEARCH = 3'd1,   // 业1AABB
                     ST_VERIFY = 3'd2,   // 16bit直呓证AABB欠
                     ST_PASS   = 3'd3,
                     ST_FAIL   = 3'd4;

    // 蓿咏SEARCH悖?
    //  >= 144一些/械露
    localparam integer SEARCH_MAX_BITS = 1024;

    reg [2:0]  state;

    // 位始展/鄄欤?
    reg [15:0] shift16;

    // VERIFY茫16bit刍
    reg [15:0] word16;
    reg [3:0]  word_bit_cnt;   // 0..15
    reg [3:0]  rep_cnt;        // 1..9要9AABB

    reg [15:0] search_cnt;

    wire [15:0] next_shift16 = {shift16[14:0], rx_bit};
    wire [15:0] next_word16  = {word16[14:0],  rx_bit};

    always @(posedge clk_130M or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_IDLE;
            shift16         <= 16'd0;
            word16          <= 16'd0;
            word_bit_cnt    <= 4'd0;
            rep_cnt         <= 4'd0;
            search_cnt      <= 16'd0;

            frame_sync_ok   <= 1'b0;
            frame_sync_fail <= 1'b0;

        end else begin
            if (!rx_en) begin
                state           <= ST_IDLE;
                shift16         <= 16'd0;
                word16          <= 16'd0;
                word_bit_cnt    <= 4'd0;
                rep_cnt         <= 4'd0;
                search_cnt      <= 16'd0;

                frame_sync_ok   <= 1'b0;
                frame_sync_fail <= 1'b0;

            end else begin
                // PASS/FAIL 帽停"帧全同/失"澹?
                if (state == ST_PASS) frame_sync_ok   <= 1'b1;
                if (state == ST_FAIL) frame_sync_fail <= 1'b1;

                // 同蹋丫 align_wait 拇
                if (state == ST_IDLE) begin
                    if (sync_locked && !align_wait) begin
                        state        <= ST_SEARCH;
                        shift16       <= 16'd0;
                        word16        <= 16'd0;
                        word_bit_cnt  <= 4'd0;
                        rep_cnt       <= 4'd0;
                        search_cnt    <= 16'd0;

                        frame_sync_ok   <= 1'b0;
                        frame_sync_fail <= 1'b0;
                    end
                end

                // SEARCH/VERIFY诩洌蔽碢ASS -> FAIL
                if ((state == ST_SEARCH || state == ST_VERIFY) && (search_cnt == (SEARCH_MAX_BITS-1))) begin
                    state           <= ST_FAIL;
                    frame_sync_fail <= 1'b1;
                end

                if (rx_bit_vld) begin
                    // SEARCH/VERIFY 诩洌好縝itsearch_cnt
                    if (state == ST_SEARCH || state == ST_VERIFY) begin
                        if (search_cnt != 16'hFFFF)
                            search_cnt <= search_cnt + 1'b1;
                    end

                    // shift16 始展SEARCH/VERIFY/PASS/FAIL鄄欤灰仓籗EARCH/VERIFY
                    shift16 <= next_shift16;

                    case (state)
                        ST_SEARCH: begin
                            // 业一 AABB
                            if (next_shift16 == SYNC_PATTERN) begin
                                // 巡1AABB末硕耄?
                                state        <= ST_VERIFY;
                                rep_cnt      <= 4'd1;

                                // 为16bit证准辗拇
                                word16       <= 16'd0;
                                word_bit_cnt <= 4'd0;
                            end
                        end

                        ST_VERIFY: begin
                            // 每16bit纬一word冉欠AABB
                            word16 <= next_word16;

                            if (word_bit_cnt == 4'd15) begin
                                // 16bitword欠为AABB
                                if (next_word16 == SYNC_PATTERN) begin
                                    // 一AABB
                                    if (rep_cnt == 4'd8) begin
                                        // 堑91 + 8 = 9
                                        state         <= ST_PASS;
                                        frame_sync_ok <= 1'b1;
                                        rep_cnt       <= 4'd9;
                                    end else begin
                                        rep_cnt <= rep_cnt + 1'b1;
                                    end

                                    // 准一16bit
                                    word16       <= 16'd0;
                                    word_bit_cnt <= 4'd0;

                                end else begin
                                    // 远耍氐SEARCH业1AABB
                                    state        <= ST_SEARCH;
                                    rep_cnt      <= 4'd0;
                                    word16       <= 16'd0;
                                    word_bit_cnt <= 4'd0;
                                end
                            end else begin
                                word_bit_cnt <= word_bit_cnt + 1'b1;
                            end
                        end

                        ST_PASS: begin
                            // OK
                            frame_sync_ok <= 1'b1;
                        end

                        ST_FAIL: begin
                            // FAIL
                            frame_sync_fail <= 1'b1;
                        end

                        default: ;
                    endcase
                end
            end
        end
    end

    //============================================================
    // dbg_frame_pack[255:0]  (员憧?"9"墓)
    //============================================================
    // pack layout (位)
    // [2:0]    state
    // [3]      align_wait
    // [4]      sync_locked
    // [5]      sync_locked_core
    // [6]      rx_bit
    // [7]      rx_bit_vld
    // [15:8]   search_cnt[7:0]
    // [19:16]  rep_cnt[3:0]
    // [23:20]  word_bit_cnt[3:0]
    // [39:24]  shift16[15:0]
    // [55:40]  word16[15:0]
    // [71:56]  pulses_in_bit[15:0]
    // [87:72]  dbg_pre_cnt[15:0]
    // [88]     preamble_active
    // [89]     preamble_end
    // [93:90]  sel_phase_idx[3:0]
    // others   0
    assign dbg_frame_pack = {
        162'd0,
        sel_phase_idx[3:0],          // [93:90]
        preamble_end,                // [89]
        preamble_active,             // [88]
        pre_cnt[15:0],               // [87:72]
        pulses_in_bit[15:0],         // [71:56]
        word16[15:0],                // [55:40]
        shift16[15:0],               // [39:24]
        word_bit_cnt[3:0],           // [23:20]
        rep_cnt[3:0],                // [19:16]
        search_cnt[7:0],             // [15:8]
        rx_bit_vld,                  // [7]
        rx_bit,                      // [6]
        sync_locked_core,            // [5]
        sync_locked,                 // [4]
        align_wait,                  // [3]
        state                        // [2:0]
    };

endmodule
