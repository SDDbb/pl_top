// prbs_gen.v (rate_sel + bit_tick)
// UPDATED: export reference-buffer write port (store TX data stream for debug/compare)
`timescale 1ns/1ps

module prbs_gen #(
    parameter integer CLK_FREQ_HZ = 130_000_000,
    parameter integer REF_N        = 8192,
    parameter integer REF_AW       = $clog2(REF_N)
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,       // 1clk pulse
    input  wire        bit_tick,    // 1clk pulse = 1 bit
    input  wire [2:0]  rate_sel,
    input  wire [1:0]  prbs_sel,

    output reg         prbs_data,
    output reg         valid,
    output reg         done,

    // reference buffer write
    output reg         ref_we,
    output reg [REF_AW-1:0] ref_waddr,
    output reg         ref_wdata
);

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
    wire [31:0] bits_10ms = data_bits_10ms(rate_sel);

    reg [31:0] bit_cnt;
    reg        active;

    reg [30:0] lfsr31;

    // combinational: current output bit (before stepping)
    reg out_bit;
    always @(*) begin
        case (prbs_sel)
            2'b00: out_bit = lfsr31[6];
            2'b01: out_bit = lfsr31[8];
            2'b10: out_bit = lfsr31[14];
            default: out_bit = lfsr31[30];
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prbs_data  <= 1'b0;
            valid      <= 1'b0;
            done       <= 1'b0;

            bit_cnt    <= 32'd0;
            active     <= 1'b0;

            lfsr31     <= 31'h7fffffff;

            ref_we     <= 1'b0;
            ref_waddr  <= {REF_AW{1'b0}};
            ref_wdata  <= 1'b0;
        end else begin
            valid  <= 1'b0;
            done   <= 1'b0;
            ref_we <= 1'b0;

            if (start) begin
                active    <= 1'b1;
                bit_cnt   <= 32'd0;
                lfsr31    <= 31'h7fffffff;
                ref_waddr <= {REF_AW{1'b0}};
            end

            if (active && bit_tick) begin
                prbs_data <= out_bit;
                valid     <= 1'b1;

                // write reference cache (first REF_N bits)
                if (bit_cnt < REF_N) begin
                    ref_we    <= 1'b1;
                    ref_wdata <= out_bit;
                    ref_waddr <= ref_waddr + 1'b1;
                end

                // step lfsr
                case (prbs_sel)
                    2'b00: lfsr31[6:0]  <= {lfsr31[5:0],  (lfsr31[6]^lfsr31[5])};
                    2'b01: lfsr31[8:0]  <= {lfsr31[7:0],  (lfsr31[8]^lfsr31[4])};
                    2'b10: lfsr31[14:0] <= {lfsr31[13:0], (lfsr31[14]^lfsr31[13])};
                    2'b11: lfsr31       <= {lfsr31[29:0], (lfsr31[30]^lfsr31[27])};
                endcase

                bit_cnt <= bit_cnt + 1'b1;

                if (bit_cnt + 1 >= bits_10ms) begin
                    done   <= 1'b1;
                    active <= 1'b0;
                end
            end
        end
    end

endmodule
