package uncore

import Chisel._
import junctions._
import cde.{Parameters, Field}

case object UseVLS extends Field[Boolean]
case object NVLSCacheSegments extends Field[Int]

trait HasVLSParameters {
  implicit val p: Parameters
  val nVLSCacheSegments = p(NVLSCacheSegments)
  val pAddrBits = p(PAddrBits)
}

class VLSAllocation(implicit val p: Parameters) extends Bundle
  with HasVLSParameters {
    val pbase = Bits(INPUT, width = pAddrBits)
    val size = Bits(INPUT, width = pAddrBits)
}

class VLSManager(implicit val p: Parameters) extends Module
  with HasVLSParameters 
  with HasCacheParameters {
  val io = new Bundle {
    val conf = new SMIIO(pAddrBits,17).flip // 1M addr map space
    val vls = Vec.fill(nVLSCacheSegments)(new VLSAllocation().flip)
  }
  def mask(in: Int) = Bits((1 << in) - 1)
  def genSize(in: Bits) = {
    val rndUp = in + mask(untagBits)
    rndUp & (mask(wayBits) << untagBits)
  }
  val addrWidth = 20
  val vlsBases = List.fill(nVLSCacheSegments)(Reg(init = Bits(0, width = pAddrBits)))
  val vlsSizes = List.fill(nVLSCacheSegments)(Reg(init = Bits(0, width = pAddrBits)))
  val ren = io.conf.req.fire() && !io.conf.req.bits.rw
  val wen = io.conf.req.fire() && io.conf.req.bits.rw
  val addr = io.conf.req.bits.addr
  val nSegs = Bits(nVLSCacheSegments)
  val read_mapping = collection.mutable.LinkedHashMap[Int,Bits](0 -> nSegs)
  val wdata = io.conf.req.bits.data
  val base_mask = mask(tagBits - wayBits) << (untagBits + wayBits)
  val wdata_base = wdata & base_mask
  val wdata_size = genSize(wdata)
  (0 until nVLSCacheSegments).foreach(i => {
    read_mapping += 2*(i+1) -> vlsBases(i)
    read_mapping += 2*(i+1)+1 -> vlsSizes(i)
    io.vls(i).pbase := vlsBases(i)
    io.vls(i).size := vlsSizes(i)
    when (wen && (addr === Bits(2*(i+1)))) { vlsBases(i) := wdata_base }
    when (wen && (addr === Bits(2*(i+1)+1))) { vlsSizes(i) := wdata_size }
  })
  val decoded_addr = read_mapping map { case (k, v) => k -> (addr === Bits(k)) }
  val rdata = Mux1H(for ((k, v) <- read_mapping) yield decoded_addr(k) -> v)
  val rdata_reg = Reg(init = Bits(0))
  val resp_valid = Reg(init = Bool(false))
  io.conf.resp.valid := resp_valid
  io.conf.resp.bits := rdata_reg
  io.conf.req.ready := !resp_valid
  when (io.conf.resp.fire()) {
    resp_valid := Bool(false)
  }
  when (io.conf.req.fire())  {
    resp_valid := Bool(true)
    rdata_reg := rdata
  }


}
