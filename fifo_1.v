`timescale 1ns/1ps

module fifo_1 #(
    parameter FIFO_DEPTH = 400000,
    parameter WIDTH      = 1,
    parameter ADDR_WIDTH = $clog2(FIFO_DEPTH)
)(
    input  wire              clk_in,
    input  wire              rst_in_n,
    input  wire              we,
    input  wire [WIDTH-1:0]  data_in,
    output wire              full,

    input  wire              clk_out,
    input  wire              rst_out_n,
    input  wire              re,
    output reg  [WIDTH-1:0]  data_out,
    output wire              empty
);
    localparam PTR_WIDTH = ADDR_WIDTH + 1;

    reg [WIDTH-1:0] mem [0:FIFO_DEPTH-1];

    reg [PTR_WIDTH-1:0] wr_ptr_bin, wr_ptr_gray;
    reg [PTR_WIDTH-1:0] wr_ptr_bin_next, wr_ptr_gray_next;

    reg [PTR_WIDTH-1:0] rd_ptr_bin, rd_ptr_gray;
    reg [PTR_WIDTH-1:0] rd_ptr_bin_next, rd_ptr_gray_next;

    reg [PTR_WIDTH-1:0] sync_rd_gray_wclk_1, sync_rd_gray_wclk_2;
    reg [PTR_WIDTH-1:0] sync_wr_gray_rclk_1, sync_wr_gray_rclk_2;

    function [PTR_WIDTH-1:0] bin2gray;
        input [PTR_WIDTH-1:0] bin;
        begin
            bin2gray = (bin >> 1) ^ bin;
        end
    endfunction

    // write side
    always @(*) begin
        wr_ptr_bin_next  = wr_ptr_bin;
        wr_ptr_gray_next = wr_ptr_gray;
        if (we && !full) begin
            wr_ptr_bin_next  = wr_ptr_bin + 1'b1;
            wr_ptr_gray_next = bin2gray(wr_ptr_bin_next);
        end
    end

    always @(posedge clk_in or negedge rst_in_n) begin
        if (!rst_in_n) begin
            wr_ptr_bin  <= 0;
            wr_ptr_gray <= 0;
        end else begin
            wr_ptr_bin  <= wr_ptr_bin_next;
            wr_ptr_gray <= wr_ptr_gray_next;
        end
    end

    wire [ADDR_WIDTH-1:0] wr_addr = wr_ptr_bin[ADDR_WIDTH-1:0];
    always @(posedge clk_in) begin
        if (we && !full) mem[wr_addr] <= data_in;
    end

    always @(posedge clk_in or negedge rst_in_n) begin
        if (!rst_in_n) begin
            sync_rd_gray_wclk_1 <= 0;
            sync_rd_gray_wclk_2 <= 0;
        end else begin
            sync_rd_gray_wclk_1 <= rd_ptr_gray;
            sync_rd_gray_wclk_2 <= sync_rd_gray_wclk_1;
        end
    end

    wire [PTR_WIDTH-1:0] rd_gray_for_full =
        {~sync_rd_gray_wclk_2[PTR_WIDTH-1],
         ~sync_rd_gray_wclk_2[PTR_WIDTH-2],
          sync_rd_gray_wclk_2[PTR_WIDTH-3:0]};

    assign full = (wr_ptr_gray_next == rd_gray_for_full);

    // read side
    always @(*) begin
        rd_ptr_bin_next  = rd_ptr_bin;
        rd_ptr_gray_next = rd_ptr_gray;
        if (re && !empty) begin
            rd_ptr_bin_next  = rd_ptr_bin + 1'b1;
            rd_ptr_gray_next = bin2gray(rd_ptr_bin_next);
        end
    end

    always @(posedge clk_out or negedge rst_out_n) begin
        if (!rst_out_n) begin
            rd_ptr_bin  <= 0;
            rd_ptr_gray <= 0;
        end else begin
            rd_ptr_bin  <= rd_ptr_bin_next;
            rd_ptr_gray <= rd_ptr_gray_next;
        end
    end

// 读地址
wire [ADDR_WIDTH-1:0] rd_addr = rd_ptr_bin[ADDR_WIDTH-1:0];

// 去掉 mem_q，直接输出
always @(posedge clk_out or negedge rst_out_n) begin
    if (!rst_out_n) begin
        data_out <= {WIDTH{1'b0}};
    end else if (re && !empty) begin
        data_out <= mem[rd_addr];
    end
end


    always @(posedge clk_out or negedge rst_out_n) begin
        if (!rst_out_n) begin
            sync_wr_gray_rclk_1 <= 0;
            sync_wr_gray_rclk_2 <= 0;
        end else begin
            sync_wr_gray_rclk_1 <= wr_ptr_gray;
            sync_wr_gray_rclk_2 <= sync_wr_gray_rclk_1;
        end
    end

    assign empty = (rd_ptr_gray == sync_wr_gray_rclk_2);

endmodule
