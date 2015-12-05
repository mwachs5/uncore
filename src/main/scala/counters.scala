package uncore

import Chisel._
import junctions._
import cde.{Parameters, Field}

case object UseL2BankCounters extends Field[Boolean]
case object L2CounterWidth extends Field[Int]

class L2BankCounters(implicit val p: Parameters) extends Module 
  with HasL2HellaCacheParameters {
  val io = new Bundle {
    val conf = new SMIIO(p(L2CounterWidth),12).flip
    val hit = Bool(INPUT)
    val miss = Bool(INPUT)
  }
  val ren = io.conf.req.fire() && !io.conf.req.bits.rw
  val wen = io.conf.req.fire() && io.conf.req.bits.rw
  val addr = io.conf.req.bits.addr
  val nHits = Reg(init = Bits(0, width = p(L2CounterWidth)))
  when (io.hit) { nHits := nHits + Bits(1) }
  val nMisses = Reg(init = Bits(0, width = p(L2CounterWidth)))
  when (io.miss) { nMisses := nMisses + Bits(1) }
  val read_mapping = collection.mutable.LinkedHashMap[Int,Bits](
    0 -> Bits(cacheId),
    1 -> nHits,
    2 -> nMisses)
  val wdata = io.conf.req.bits.data
  val decoded_addr = read_mapping map { case (k, v) => k -> (addr === Bits(k)) }
  val rdata = Mux1H(for ((k, v) <- read_mapping) yield decoded_addr(k) -> v)
  val rdata_reg = Reg(init = Bits(0))
  val resp_valid = Reg(init = Bool(false))
  io.conf.resp.valid := resp_valid
  io.conf.resp.bits := rdata_reg
  io.conf.req.ready := !resp_valid
  read_mapping foreach { case (k, v) => if (k != 0) { when (wen && addr === Bits(k)){ v := wdata } } }
  when (io.conf.resp.fire()) {
    resp_valid := Bool(false)
  }
  when (io.conf.req.fire())  {
    resp_valid := Bool(true)
    rdata_reg := rdata
  }
}
