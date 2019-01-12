module MemoryResourceBox
  (
   input logic         clock,
   input logic         io_mem_inst_bus_req,
   input logic [15:0]  io_mem_inst_bus_addr,
   output logic        io_mem_inst_bus_ack,
   output logic [31:0] io_mem_inst_bus_rddata,
   input logic         io_mem_data_bus_req,
   input logic [1:0]   io_mem_data_bus_cmd,
   input logic [15:0]  io_mem_data_bus_addr,
   input logic [2:0]   io_mem_data_bus_size,
   input logic [63:0]  io_mem_data_bus_wrdata,
   output logic [63:0] io_mem_data_bus_rddata,
   input logic         io_mem_ext_bus_req,
   input logic [15:0]  io_mem_ext_bus_addr,
   input logic [31:0]  io_mem_ext_bus_data,
   output logic [31:0] io_mem_ext_bus_rddata
   );

logic [15: 0]          w_addr;
logic [31: 0]          w_rddata;
logic                  w_req;
logic [31: 0]          w_wrdata;

assign w_addr   = io_mem_ext_bus_req ? io_mem_ext_bus_addr : io_mem_inst_bus_addr;
assign w_wrdata = io_mem_ext_bus_req ? io_mem_ext_bus_data : 32'h0;
assign w_we     = io_mem_ext_bus_req ? io_mem_ext_bus_req  : io_mem_inst_bus_req;

assign io_mem_ext_bus_rddata  = w_rddata;
assign io_mem_inst_bus_rddata = w_rddata;

always @ (posedge clock) begin
  io_mem_inst_bus_ack <= io_mem_inst_bus_req;
end

memory_core
  #(.DATAW(64), .ADDRW(16))
u_mem
  (
   .clock (clock),

   .addra(io_mem_data_bus_addr),
   .rda  (io_mem_data_bus_rddata),
   .wea  (io_mem_data_bus_req & (io_mem_data_bus_cmd == 1)),
   .wda  (io_mem_data_bus_wrdata),

   .addrb(w_addr),
   .rdb  (w_rddata),
   .web  (w_we),
   .wdb  (w_wrdata)

   // .addrb(io_mem_ext_bus_addr)
   // .rdb  (io_mem_ext_bus_data)
   // .web  (io_mem_ext_bus_req)
   // .wdb  (io_mem_ext_bus_rddata)
   );

endmodule // MemoryResourceBox

// always @(posedge clock) begin
//   io_mem_inst_bus_rddata <= mem[];
//   io_mem_inst_bus_ack <= io_mem_inst_bus_req;
// end
//
// endmodule // MemoryBlackBox


module memory_core
#(
  parameter DATAW = 64,
  parameter ADDRW = 16
  )
(
 input logic               clock,
 input logic [ADDRW-1: 0]  addra,
 output logic [DATAW-1: 0] rda,
 input logic               wea,
 input logic [DATAW-1: 0]  wda,

 input logic [ADDRW-1: 0]  addrb,
 output logic [DATAW-1: 0] rdb,
 input logic               web,
 input logic [DATAW-1: 0]  wdb
 );

reg [DATAW-1: 0]            mem[0: 1 << ADDRW -1];

always @ (posedge clock) begin
  if (wea) begin
    mem[addra] <= wda;
  end
  rda <= mem[addra];
end

always @ (posedge clock) begin
  if (web) begin
    mem[addrb] <= wdb;
  end
  rdb <= mem[addrb];
end

endmodule // memory_core
