package cpu

import chisel3._
import chisel3.util._

class MemoryBlackBox [Conf <: RVConfig](conf: Conf) extends BlackBox with HasBlackBoxInline {
  val io = IO(new Bundle {
    val clock = Input(Clock())
    val mem = new MemoryIo(conf)
  })

  setInline("MemoryBlackBox.v",
    s"""
|module MemoryBlackBox
|  (
|   input logic         clock,
|   input logic         mem_inst_bus_req,
|   input logic [15:0]  mem_inst_bus_addr,
|   output logic        mem_inst_bus_ack,
|   output logic [31:0] mem_inst_bus_rddata,
|   input logic         mem_data_bus_req,
|   input logic [1:0]   mem_data_bus_cmd,
|   input logic [15:0]  mem_data_bus_addr,
|   input logic [2:0]   mem_data_bus_size,
|   input logic [63:0]  mem_data_bus_wrdata,
|   output logic [63:0] mem_data_bus_rddata,
|   input logic         mem_ext_bus_req,
|   input logic [15:0]  mem_ext_bus_addr,
|   input logic [31:0]  mem_ext_bus_data,
|   output logic [31:0] mem_ext_bus_rddata
|   );
|reg [63: 0]      mem[0: 65535];
|
|always @(posedge clock) begin
|  if (mem_data_bus_req & (mem_data_bus_cmd == 1)) begin
|    mem[mem_data_bus_addr] <= mem_data_bus_wrdata;
|  end
|  mem_data_bus_rddata <= mem[mem_data_bus_addr];
|end
|
|always @(posedge clock) begin
|  mem_inst_bus_rddata <= mem[mem_inst_bus_addr];
|end
|
|always @(posedge clock) begin
|  if (mem_ext_bus_req) begin
|    mem[mem_extbus_addr] <= mem_ext_bus_data;
|  end
|end
|
|assign mem_ext_bus_rddata = mem[mem_extbus_addr];
|
|endmodule // MemoryBlackBox
    """.stripMargin)
}
