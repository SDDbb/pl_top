// simple_bit_ram.v
`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// simple_bit_ram
// - single-clock RAM, 1-bit wide
// - 1 write port + 1 read port (both synchronous to clk)
// - read data is registered (1-cycle latency)
// - intended to infer BRAM in Vivado (ram_style="block")
// -----------------------------------------------------------------------------
module simple_bit_ram #(
    parameter integer DEPTH  = 8192,
    parameter integer ADDR_W = $clog2(DEPTH)
)(
    input  wire                 clk,

    input  wire                 we,
    input  wire [ADDR_W-1:0]    waddr,
    input  wire                 wdata,

    input  wire [ADDR_W-1:0]    raddr,
    output reg                  rdata
);

    (* ram_style = "block" *)
    reg mem [0:DEPTH-1];

    integer i;
    initial begin
        for (i = 0; i < DEPTH; i = i + 1) mem[i] = 1'b0;
    end

    always @(posedge clk) begin
        if (we) begin
            mem[waddr] <= wdata;
        end
        rdata <= mem[raddr];
    end

endmodule
