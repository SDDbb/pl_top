// rx_frame_capture.v
`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// rx_frame_capture (online compare + small caches)
// -----------------------------------------------------------------------------
module rx_frame_capture #(
    parameter integer SILENCE_BITS = 100,
    parameter integer HEADER_BITS  = 384,

    parameter integer FIRST_N      = 8192,
    parameter integer FIRST_AW     = $clog2(FIRST_N),

    // ERR_WIN 建议为 2^k，环形指针自然溢出即可实现取模
    parameter integer ERR_WIN      = 256,
    parameter integer ERR_AW       = $clog2(ERR_WIN),
    parameter integer ERR_PRE      = 96,
    parameter integer ERR_POST     = 96
)(
    input  wire         clk,
    input  wire         rst_n,

    input  wire         rx_en,
    input  wire [2:0]   rate_sel,
    input  wire [1:0]   prbs_sel,

    input  wire         frame_sync_ok,
    input  wire         rx_bit,
    input  wire         rx_bit_vld,
    input  wire [15:0]  pulses_in_bit,

    // --- first N buffer write ---
    output reg                  first_we,
    output reg [FIRST_AW-1:0]   first_waddr,
    output reg                  first_wdata,

    // --- error window ring buffer write ---
    output reg                  err_we,
    output reg [ERR_AW-1:0]     err_waddr,
    output reg                  err_wdata,

    // --- capture state / done ---
    output reg                  cap_active,
    output reg                  data_done,

    // --- BER/FER ---
    output reg [31:0]           total_bits,
    output reg [31:0]           err_bits,
    output reg [31:0]           frame_cnt,
    output reg [31:0]           frame_err_cnt,

    // --- error distribution ---
    output reg [15:0]           max_consecutive_err,
    output reg [15:0]           err_run_hist0, // run=1
    output reg [15:0]           err_run_hist1, // run=2
    output reg [15:0]           err_run_hist2, // run=3..4
    output reg [15:0]           err_run_hist3, // run=5..8
    output reg [15:0]           err_run_hist4, // run=9..16
    output reg [15:0]           err_run_hist5, // run=17..32
    output reg [15:0]           err_run_hist6, // run=33..64
    output reg [15:0]           err_run_hist7, // run>=65

    // --- photon count stats ---
    output reg [47:0]           sum_pulses_total,
    output reg [47:0]           sum_pulses_ok,
    output reg [47:0]           sum_pulses_err,
    output reg [31:0]           bits_ok,
    output reg [31:0]           bits_err,

    // --- debug: error window mapping ---
    output reg                  err_latched,
    output reg [31:0]           err_bit_index,
    output reg [ERR_AW-1:0]     err_base_ptr
);

    // =========================================================================
    // DATA_BITS(10ms) by rate_sel
    // =========================================================================
    function automatic [31:0] data_bits_10ms(input [2:0] r);
        begin
            case (r)
                3'd0: data_bits_10ms = 32'd20000;
                3'd1: data_bits_10ms = 32'd40000;
                3'd2: data_bits_10ms = 32'd80000;
                3'd3: data_bits_10ms = 32'd160000;
                3'd4: data_bits_10ms = 32'd320000;
                3'd5: data_bits_10ms = 32'd650000;
                default: data_bits_10ms = 32'd20000;
            endcase
        end
    endfunction
    wire [31:0] DATA_BITS = data_bits_10ms(rate_sel);

    // =========================================================================
    // frame_sync_ok rising edge
    // =========================================================================
    reg frame_sync_ok_d;
    wire frame_sync_ok_rise = frame_sync_ok & ~frame_sync_ok_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) frame_sync_ok_d <= 1'b0;
        else        frame_sync_ok_d <= frame_sync_ok;
    end

    // =========================================================================
    // PRBS reference step (match prbs_gen)
    // return {next_lfsr[30:0], ref_bit}
    // =========================================================================
    function automatic [31:0] prbs_step_fn(input [30:0] lfsr_in, input [1:0] sel);
        reg [30:0] n;
        reg        o;
        begin
            n = lfsr_in;
            o = 1'b0;
            case (sel)
                2'b00: begin o = n[6];  n[6:0]  = {n[5:0],  (n[6]^n[5])};   end
                2'b01: begin o = n[8];  n[8:0]  = {n[7:0],  (n[8]^n[4])};   end
                2'b10: begin o = n[14]; n[14:0] = {n[13:0], (n[14]^n[13])}; end
                2'b11: begin o = n[30]; n       = {n[29:0], (n[30]^n[27])}; end
            endcase
            prbs_step_fn = {n, o};
        end
    endfunction

    reg  [30:0] lfsr31;
    wire [31:0] prbs_step_w = prbs_step_fn(lfsr31, prbs_sel);
    wire        ref_bit_w   = prbs_step_w[0];
    wire [30:0] next_lfsr_w = prbs_step_w[31:1];

    // =========================================================================
    // Capture FSM
    // =========================================================================
    localparam [2:0]
        ST_IDLE      = 3'd0,
        ST_SKIP_SIL2 = 3'd1,
        ST_SKIP_HDR  = 3'd2,
        ST_SKIP_SIL3 = 3'd3,
        ST_CAP_DATA  = 3'd4,
        ST_DONE      = 3'd5;

    reg [2:0]  st;
    reg [31:0] skip_cnt;
    reg [31:0] data_idx;

    reg        frame_has_err;

    // error-run tracking
    reg        last_err;
    reg [15:0] cur_run;

    // error window control
    reg [ERR_AW-1:0] err_wptr;
    reg [15:0]       err_post_left;

    task automatic hist_commit(input [15:0] run_len);
        begin
            if (run_len == 16'd0) begin
            end else if (run_len == 16'd1) begin
                err_run_hist0 <= err_run_hist0 + 1'b1;
            end else if (run_len == 16'd2) begin
                err_run_hist1 <= err_run_hist1 + 1'b1;
            end else if (run_len <= 16'd4) begin
                err_run_hist2 <= err_run_hist2 + 1'b1;
            end else if (run_len <= 16'd8) begin
                err_run_hist3 <= err_run_hist3 + 1'b1;
            end else if (run_len <= 16'd16) begin
                err_run_hist4 <= err_run_hist4 + 1'b1;
            end else if (run_len <= 16'd32) begin
                err_run_hist5 <= err_run_hist5 + 1'b1;
            end else if (run_len <= 16'd64) begin
                err_run_hist6 <= err_run_hist6 + 1'b1;
            end else begin
                err_run_hist7 <= err_run_hist7 + 1'b1;
            end
        end
    endtask

    // =========================================================================
    // Main sequential
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= ST_IDLE;
            skip_cnt    <= 32'd0;
            data_idx    <= 32'd0;

            cap_active  <= 1'b0;
            data_done   <= 1'b0;

            first_we    <= 1'b0;
            first_waddr <= {FIRST_AW{1'b0}};
            first_wdata <= 1'b0;

            err_we      <= 1'b0;
            err_waddr   <= {ERR_AW{1'b0}};
            err_wdata   <= 1'b0;

            lfsr31      <= 31'h7fffffff;

            total_bits    <= 32'd0;
            err_bits      <= 32'd0;
            frame_cnt     <= 32'd0;
            frame_err_cnt <= 32'd0;
            frame_has_err <= 1'b0;

            max_consecutive_err <= 16'd0;
            last_err            <= 1'b0;
            cur_run             <= 16'd0;

            err_run_hist0 <= 16'd0;
            err_run_hist1 <= 16'd0;
            err_run_hist2 <= 16'd0;
            err_run_hist3 <= 16'd0;
            err_run_hist4 <= 16'd0;
            err_run_hist5 <= 16'd0;
            err_run_hist6 <= 16'd0;
            err_run_hist7 <= 16'd0;

            sum_pulses_total <= 48'd0;
            sum_pulses_ok    <= 48'd0;
            sum_pulses_err   <= 48'd0;
            bits_ok          <= 32'd0;
            bits_err         <= 32'd0;

            err_wptr      <= {ERR_AW{1'b0}};
            err_post_left <= 16'd0;
            err_latched   <= 1'b0;
            err_bit_index <= 32'd0;
            err_base_ptr  <= {ERR_AW{1'b0}};
        end else begin
            first_we  <= 1'b0;
            err_we    <= 1'b0;
            data_done <= 1'b0;

            if (!rx_en) begin
                st         <= ST_IDLE;
                cap_active <= 1'b0;
            end

            case (st)
                ST_IDLE: begin
                    cap_active <= 1'b0;
                    if (rx_en && frame_sync_ok_rise) begin
                        st         <= ST_SKIP_SIL2;
                        cap_active <= 1'b1;

                        skip_cnt <= 32'd0;
                        data_idx <= 32'd0;

                        frame_cnt     <= frame_cnt + 1'b1;
                        frame_has_err <= 1'b0;

                        lfsr31 <= 31'h7fffffff;

                        last_err <= 1'b0;
                        cur_run  <= 16'd0;

                        first_waddr <= {FIRST_AW{1'b0}};
                        err_wptr    <= {ERR_AW{1'b0}};
                        err_post_left <= 16'd0;
                        err_latched <= 1'b0;
                        err_bit_index <= 32'd0;
                        err_base_ptr  <= {ERR_AW{1'b0}};
                    end
                end

                ST_SKIP_SIL2: begin
                    if (rx_bit_vld) begin
                        if (skip_cnt + 1 >= SILENCE_BITS) begin
                            st       <= ST_SKIP_HDR;
                            skip_cnt <= 32'd0;
                        end else skip_cnt <= skip_cnt + 1'b1;
                    end
                end

                ST_SKIP_HDR: begin
                    if (rx_bit_vld) begin
                        if (skip_cnt + 1 >= HEADER_BITS) begin
                            st       <= ST_SKIP_SIL3;
                            skip_cnt <= 32'd0;
                        end else skip_cnt <= skip_cnt + 1'b1;
                    end
                end

                ST_SKIP_SIL3: begin
                    if (rx_bit_vld) begin
                        if (skip_cnt + 1 >= SILENCE_BITS) begin
                            st       <= ST_CAP_DATA;
                            skip_cnt <= 32'd0;
                        end else skip_cnt <= skip_cnt + 1'b1;
                    end
                end

                ST_CAP_DATA: begin
                    if (rx_bit_vld) begin
                        // step PRBS once per DATA bit
                        lfsr31 <= next_lfsr_w;

                        // always accumulate photons
                        sum_pulses_total <= sum_pulses_total + pulses_in_bit;

                        // cache first N bits
                        if (data_idx < FIRST_N) begin
                            first_we    <= 1'b1;
                            first_wdata <= rx_bit;
                            // waddr uses old first_waddr at this clk edge
                            first_waddr <= first_waddr + 1'b1;
                        end

                        // error window ring cache
                        if (!err_latched) begin
                            err_we    <= 1'b1;
                            err_wdata <= rx_bit;
                            err_waddr <= err_wptr;
                            err_wptr  <= err_wptr + 1'b1;
                        end else if (err_post_left != 16'd0) begin
                            err_we    <= 1'b1;
                            err_wdata <= rx_bit;
                            err_waddr <= err_wptr;
                            err_wptr  <= err_wptr + 1'b1;
                            err_post_left <= err_post_left - 1'b1;
                        end

                        // online compare
                        total_bits <= total_bits + 1'b1;

                        if (rx_bit ^ ref_bit_w) begin
                            err_bits      <= err_bits + 1'b1;
                            frame_has_err <= 1'b1;

                            sum_pulses_err <= sum_pulses_err + pulses_in_bit;
                            bits_err       <= bits_err + 1'b1;

                            // error run
                            if (last_err) cur_run <= cur_run + 1'b1;
                            else          cur_run <= 16'd1;
                            last_err <= 1'b1;

                            // max run
                            if (last_err) begin
                                if (cur_run + 1 > max_consecutive_err)
                                    max_consecutive_err <= cur_run + 1;
                            end else begin
                                if (16'd1 > max_consecutive_err)
                                    max_consecutive_err <= 16'd1;
                            end

                            // latch first error
                            if (!err_latched) begin
                                err_latched   <= 1'b1;
                                err_bit_index <= data_idx;
                                err_base_ptr  <= (err_wptr + 1'b1) - ERR_PRE[ERR_AW-1:0];
                                err_post_left <= ERR_POST[15:0];
                            end
                        end else begin
                            sum_pulses_ok <= sum_pulses_ok + pulses_in_bit;
                            bits_ok       <= bits_ok + 1'b1;

                            if (last_err) begin
                                hist_commit(cur_run);
                                cur_run <= 16'd0;
                            end
                            last_err <= 1'b0;
                        end

                        // advance data idx / finish
                        if (data_idx + 1 >= DATA_BITS) begin
                            if (last_err) begin
                                hist_commit(cur_run);
                                cur_run  <= 16'd0;
                                last_err <= 1'b0;
                            end

                            if (frame_has_err) frame_err_cnt <= frame_err_cnt + 1'b1;

                            data_done <= 1'b1;
                            st        <= ST_DONE;
                        end else begin
                            data_idx <= data_idx + 1'b1;
                        end
                    end
                end

                ST_DONE: begin
                    cap_active <= 1'b0;
                    if (!rx_en) st <= ST_IDLE;
                    else if (frame_sync_ok_rise) begin
                        st         <= ST_SKIP_SIL2;
                        cap_active <= 1'b1;

                        skip_cnt <= 32'd0;
                        data_idx <= 32'd0;

                        frame_cnt     <= frame_cnt + 1'b1;
                        frame_has_err <= 1'b0;

                        lfsr31 <= 31'h7fffffff;

                        last_err <= 1'b0;
                        cur_run  <= 16'd0;

                        first_waddr <= {FIRST_AW{1'b0}};
                        err_wptr    <= {ERR_AW{1'b0}};
                        err_post_left <= 16'd0;
                        err_latched <= 1'b0;
                        err_bit_index <= 32'd0;
                        err_base_ptr  <= {ERR_AW{1'b0}};
                    end
                end

                default: st <= ST_IDLE;
            endcase
        end
    end

endmodule
