package uncore

import Chisel._
import junctions._
import junctions.NastiConstants._
import cde.{Parameters, Field}

class NastiROM(contents: Seq[Byte])(implicit p: Parameters) extends Module {
  val io = new NastiIO().flip
  val ar = Queue(io.ar, 1)

  // This assumes ROMs are in read-only parts of the address map.
  // Reuse b_queue code from NastiErrorSlave if this assumption is bad.
  when (ar.valid) { assert(ar.bits.len === UInt(0), "Can't burst-read from NastiROM") }
  assert(!(io.aw.valid || io.w.valid), "Can't write to NastiROM")
  io.aw.ready := Bool(false)
  io.w.ready := Bool(false)
  io.b.valid := Bool(false)

  val byteWidth = io.r.bits.nastiXDataBits / 8
  val rows = (contents.size + byteWidth - 1)/byteWidth + 1
  val rom = Vec.tabulate(rows) { i =>
    val slice = contents.slice(i*byteWidth, (i+1)*byteWidth)
    UInt(slice.foldRight(BigInt(0)) { case (x,y) => (y << 8) + (x.toInt & 0xFF) })
  }
  val rdata_word = rom(if (rows == 1) UInt(0) else ar.bits.addr(log2Up(contents.size)-1,log2Up(byteWidth)))
  val rdata = new LoadGen(Cat(UInt(1), ar.bits.size), ar.bits.addr, rdata_word, Bool(false), byteWidth).data

  io.r <> ar
  io.r.bits := NastiReadDataChannel(ar.bits.id, rdata)
}

class NastiRAM(depth: Int)(implicit p: Parameters) extends NastiModule()(p) {
  val wordBytes = nastiXDataBits / 8
  val byteAddrBits = log2Up(wordBytes)
  val wordAddrBits = log2Up(depth)
  val fullAddrBits = wordAddrBits + byteAddrBits

  val io = new NastiIO().flip

  val mem = SeqMem(depth, Bits(width = nastiXDataBits))

  //def read_data(addr: UInt, size: UInt): UInt = {
  //  val word = mem.read(addr(fullAddrBits - 1, byteAddrBits)).toBits
  //  val shift = Cat(addr(byteAddrBits - 1, 0), UInt(0, 3))
  //  val nbits = Cat(UInt(1) << size, UInt(0, 3))
  //  val mask = (UInt(1) << nbits) - UInt(1)
  //  (word >> shift) & mask
  //}

  //def write_data(addr: UInt, size: UInt, strb: UInt, data: UInt) {
  //  val shift = Cat(addr(byteAddrBits - 1, 0), UInt(0, 3))
  //  val nbytes = UInt(1) << size
  //  val size_mask = (UInt(1) << (UInt(1) << size)) - UInt(1)
  //  val mask = (size_mask & strb) << addr(byteAddrBits - 1, 0)
  //  val word = data << shift
  //  val data_vec = Vec.tabulate(wordBytes) { i => word((i + 1) * 8 - 1, i * 8) }
  //  mem.write(addr(fullAddrBits - 1, byteAddrBits), data_vec, mask)
  //}

  val rid = Reg(UInt(width = nastiXIdBits))
  val raddr = Reg(UInt(width = wordAddrBits))
  val rlen = Reg(UInt(width = nastiXLenBits))

  val s_read_addr :: s_read_mem :: s_read_resp :: Nil = Enum(Bits(), 3)
  val read_state = Reg(init = s_read_addr)

  when (io.ar.fire()) {
    rid := io.ar.bits.id
    raddr := io.ar.bits.addr(fullAddrBits - 1, byteAddrBits)
    rlen := io.ar.bits.len
    read_state := s_read_mem
  }

  when (read_state === s_read_mem) {
    read_state := s_read_resp
  }

  when (io.r.fire()) {
    when (io.r.bits.last) {
      read_state := s_read_addr
    } .otherwise {
      rlen := rlen - UInt(1)
      raddr := raddr + UInt(1)
      read_state := s_read_mem
    }
  }

  io.ar.ready := (read_state === s_read_addr)
  io.r.valid := (read_state === s_read_resp)
  io.r.bits := NastiReadDataChannel(
    id = rid,
    data = mem.read(raddr),
    last = rlen === UInt(0))

  val wid = Reg(UInt(width = nastiXIdBits))
  val waddr = Reg(UInt(width = wordAddrBits))

  val s_write_addr :: s_write_data :: s_write_resp :: Nil = Enum(Bits(), 3)
  val write_state = Reg(init = s_write_addr)

  when (io.aw.fire()) {
    wid := io.aw.bits.id
    waddr := io.aw.bits.addr(fullAddrBits - 1, byteAddrBits)
    write_state := s_write_data

  }

  when (io.w.fire()) {
    waddr := waddr + UInt(1)
    mem.write(waddr, io.w.bits.data)
    when (io.w.bits.last) { write_state := s_write_resp }
  }

  when (io.b.fire()) { write_state := s_write_addr }

  io.aw.ready := (write_state === s_write_addr)
  io.w.ready := (write_state === s_write_data)
  io.b.valid := (write_state === s_write_resp)
  io.b.bits := NastiWriteResponseChannel(id = wid)

  assert(!io.aw.valid || io.aw.bits.len === UInt(0) || io.aw.bits.burst === BURST_INCR,
    "BackupMemory only supports incrementing bursts")
  assert(!io.aw.valid || io.aw.bits.size === UInt(log2Up(wordBytes)),
    s"BackupMemory only supports ${nastiXDataBits}-bit writes")
  assert(!io.w.valid || io.w.bits.strb.andR,
    "BackupMemory does not support partial writes")
  assert(!io.ar.valid || io.ar.bits.len === UInt(0) || io.ar.bits.burst === BURST_INCR,
    "BackupMemory only supports incrementing bursts")
  assert(!io.ar.valid || io.ar.bits.size === UInt(log2Up(wordBytes)),
    s"BackupMemory only supports ${nastiXDataBits}-bit reads")
}
