package cpu

import chisel3._
import chisel3.util._

class MemoryBlackBox extends BlackBox with HasBlackBoxResource {
  setInline("MemoryBlackBox.v",
    s"""
      |module MemoryBlackBox (
      |    input         clock,
      |    input         io_inst_bus_req,
      |    input  [15:0] io_inst_bus_addr,
      |    output        io_inst_bus_ack,
      |    output [31:0] io_inst_bus_rddata,
      |    input         io_data_bus_req,
      |    input  [1:0]  io_data_bus_cmd,
      |    input  [15:0] io_data_bus_addr,
      |    input  [2:0]  io_data_bus_size,
      |    input  [63:0] io_data_bus_wrdata,
      |    output [63:0] io_data_bus_rddata,
      |    input  [15:0] io_ext_bus_addr,
      |    output [31:0] io_ext_bus_rddata
      |);
      | reg [63: 0] mem[0: 65535];
      |always @(posedge clock) begin
      |   if (io_data_bus_cmd == 0) begin
      |      mem[io_data_bus_addr] <= io_data_bus_dwrdata;
      |   end
      |end
      |endmodule
    """.stripMargin)
}
