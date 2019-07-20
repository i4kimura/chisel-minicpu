package ooo

import chisel3._
import chisel3.util._
import chisel3.Bool

import DecodeConsts._


class FrontEndReqIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new FrontEndReqIo(conf).asInstanceOf[this.type]

  val addr = UInt(conf.bus_width.W)
}

class FrontEndRespIo [Conf <: RVConfig](conf: Conf) extends Bundle {
  override def cloneType: this.type =
    new FrontEndRespIo(conf).asInstanceOf[this.type]

  val data = UInt(64.W)
}


class OooCore [Conf <: RVConfig](conf: Conf) extends Module {
  val io = IO (new Bundle {
    val front_req = Decoupled(new FrontEndReqIo(conf))
    val front_resp = Flipped(Decoupled(new FrontEndRespIo(conf)))

    val lsu_req = Decoupled(new LsuReqIo(conf))
    val lsu_resp = Flipped(Decoupled(new LsuRespIo(conf)))
  })

  io.front_req.valid     := true.B
  io.front_req.bits.addr := 0.U

  io.front_resp.ready := true.B

  /* Decoder */
  val decode_units = for (w <- 0 until conf.fetch_width) yield {
    val u_cpath = Module (new CtlPath);
    u_cpath
  }
  val cpath = Wire(Vec(conf.fetch_width, new CtrlSignals()))
  for (w <- 0 until conf.fetch_width) {
    decode_units(w).io.inst := io.front_resp.bits.data(w*32+31, w*32)
    cpath(w) := decode_units(w).io.ctl
  }

  /* Register Read */
  val u_regs = Module (new Regs(conf))
  val reg_read_data = Wire(Vec(conf.fetch_width, Vec(2, SInt(conf.xlen.W))))
  for (w <- 0 until conf.fetch_width) {
    u_regs.io.read(w).en  (0) := true.B
    u_regs.io.read(w).addr(0) := decode_units(w).io.inst(24,20)

    u_regs.io.read(w).en  (1) := true.B
    u_regs.io.read(w).addr(1) := decode_units(w).io.inst(19,15)

    for(port <- 0 until 2) {
      reg_read_data(w)(port) := u_regs.io.read(w).data(port)
    }
  }

  /* ALU */
  val u_alu = for (w <- 0 until conf.fetch_width) yield {
    val u_alu = Module (new Alu(conf))

    val ex_inst = decode_units(w).io.inst

    val ex_imm_i      = ex_inst(31, 20).asSInt
    val ex_imm_b      = Cat(ex_inst(31), ex_inst(7), ex_inst(30,25), ex_inst(11,8))
    val ex_imm_b_sext = Cat(Fill(19,ex_imm_b(11)), ex_imm_b, 0.U)
    val ex_imm_s      = Cat(ex_inst(31, 25), ex_inst(11,7)).asSInt
    val ex_imm_j      = Cat(ex_inst(31), ex_inst(19,12), ex_inst(20), ex_inst(30,21), 0.U(1.W))
    val ex_imm_j_sext = Cat(Fill(64-21, ex_imm_j(20)), ex_imm_j, 0.U(1.W))

    val ex_alu_op0 = Wire (SInt(conf.xlen.W))
    val ex_alu_op1 = Wire (SInt(conf.xlen.W))

    u_alu.io.func := cpath(w).alu_fun
    u_alu.io.op0  := ex_alu_op0
    u_alu.io.op1  := ex_alu_op1

    val ex_op0_sel = cpath(w).op0_sel
    val ex_op1_sel = cpath(w).op1_sel

    ex_alu_op0 := 0.S
    switch(ex_op0_sel) {
      is (OP0_RS1) {
        // Register Forwarding
        // when (mem_inst_valid & mem_inst_wb_en & (mem_inst_rd === ex_inst_rs0)) {
        //   ex_alu_op0 := mem_alu_res
        // } .elsewhen (wb_inst_valid & wb_inst_wb_en & (wb_inst_rd === ex_inst_rs0)) {
        //   ex_alu_op0 := Mux (wb_ctrl_mem_cmd === MCMD_RD, wb_mem_rdval, wb_alu_res)
        // } .otherwise {
        ex_alu_op0 := reg_read_data(w)(0)
        // }
      }
      is (OP0_IMU) { ex_alu_op0 := Cat(ex_inst(31, 12), Fill(12,0.U)).asSInt }
      is (OP0_IMZ) { ex_alu_op0 := Cat(Fill(27,0.U), ex_inst(19,15)).asSInt }
    }

    ex_alu_op1 := 0.S
    switch(ex_op1_sel) {
      // Register Forwarding
      // is (OP1_PC ) { ex_alu_op1 := ex_inst_addr.asSInt }
      is (OP1_PC ) { ex_alu_op1 := 0.S }
      is (OP1_RS2) {
        // when (mem_inst_valid & mem_inst_wb_en & (mem_inst_rd === ex_inst_rs1)) {
        //   ex_alu_op1 := mem_alu_res
        // } .elsewhen (wb_inst_valid & wb_inst_wb_en & (wb_inst_rd === ex_inst_rs1)) {
        //   ex_alu_op1 := Mux (wb_ctrl_mem_cmd === MCMD_RD, wb_mem_rdval, wb_alu_res)
        // } .elsewhen (ex_csr_wbcsr =/= CSR.X) {
        //   ex_alu_op1 := u_csrfile.io.rw.rdata.asSInt
        // } .otherwise {
        ex_alu_op1 := reg_read_data(w)(1)
        // }
      }
      is (OP1_IMI) { ex_alu_op1 := Cat(Fill(20,ex_imm_i(11)), ex_imm_i).asSInt }
      is (OP1_IMS) { ex_alu_op1 := Cat(Fill(20,ex_imm_s(11)), ex_imm_s).asSInt }
    }

    u_alu
  }

  /* LSU */
  val lsu = Module (new Lsu(conf))
  lsu.io.ctrl <> cpath(1)
  lsu.io.op0 := reg_read_data(1)(0)
  lsu.io.op1 := reg_read_data(1)(1)

  io.lsu_req  <> lsu.io.lsu_req
  lsu.io.lsu_resp <> io.lsu_resp

  /* WriteBack */
  u_regs.io.write.en   := false.B
  u_regs.io.write.addr := 0.U
  u_regs.io.write.data := 0.S

  when (lsu.io.lsu_read.valid) {
    u_regs.io.write.en   := true.B
    // u_regs.io.write.addr :=
    u_regs.io.write.data := lsu.io.lsu_read.bits.rddata
  }

  when (cpath(0).wb_en) {
    u_regs.io.write.en   := true.B
    // u_regs.io.write.addr :=
    u_regs.io.write.data := u_alu(0).io.res
  }

  when (cpath(1).wb_en) {
    u_regs.io.write.en   := true.B
    // u_regs.io.write.addr :=
    u_regs.io.write.data := u_alu(1).io.res
  }
}


object OooCore extends App {
  chisel3.Driver.execute(args, () => new OooCore(new RV64IConfig))
}
