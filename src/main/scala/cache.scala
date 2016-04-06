// See LICENSE for license details.

package uncore
import Chisel._
import scala.reflect.ClassTag
import junctions._
import cde.{Parameters, Field}

case object CacheName extends Field[String]
case object NSets extends Field[Int]
case object NWays extends Field[Int]
case object RowBits extends Field[Int]
case object Replacer extends Field[() => ReplacementPolicy]
case object L2Replacer extends Field[() => SeqReplacementPolicy]
case object AmoAluOperandBits extends Field[Int]
case object NPrimaryMisses extends Field[Int]
case object NSecondaryMisses extends Field[Int]
case object CacheBlockBytes extends Field[Int]
case object CacheBlockOffsetBits extends Field[Int]
case object ECCCode extends Field[Option[Code]]
case object CacheIdBits extends Field[Int]
case object CacheId extends Field[Int]
case object SplitMetadata extends Field[Boolean]

trait HasCacheParameters {
  implicit val p: Parameters
  val nSets = p(NSets)
  val blockOffBits = p(CacheBlockOffsetBits)
  val cacheIdBits = p(CacheIdBits)
  val idxBits = log2Up(nSets)
  val untagBits = blockOffBits + cacheIdBits + idxBits
  val tagBits = p(PAddrBits) - untagBits
  val nWays = p(NWays)
  val wayBits = log2Up(nWays)
  val isDM = nWays == 1
  val rowBits = p(RowBits)
  val rowBytes = rowBits/8
  val rowOffBits = log2Up(rowBytes)
  val code = p(ECCCode).getOrElse(new IdentityCode)
  val hasSplitMetadata = p(SplitMetadata)
}

abstract class CacheModule(implicit val p: Parameters) extends Module
  with HasCacheParameters
abstract class CacheBundle(implicit val p: Parameters) extends ParameterizedBundle()(p)
  with HasCacheParameters

abstract class ReplacementPolicy {
  def way: UInt
  def miss: Unit
  def hit: Unit
}

class RandomReplacement(ways: Int) extends ReplacementPolicy {
  private val replace = Wire(Bool())
  replace := Bool(false)
  val lfsr = LFSR16(replace)

  def way = if(ways == 1) UInt(0) else lfsr(log2Up(ways)-1,0)
  def miss = replace := Bool(true)
  def hit = {}
}

abstract class SeqReplacementPolicy {
  def access(set: UInt): Unit
  def update(valid: Bool, hit: Bool, set: UInt, way: UInt): Unit
  def way: UInt
}

class SeqRandom(n_ways: Int) extends SeqReplacementPolicy {
  val logic = new RandomReplacement(n_ways)
  def access(set: UInt) = { }
  def update(valid: Bool, hit: Bool, set: UInt, way: UInt) = {
    when (valid && !hit) { logic.miss }
  }
  def way = logic.way
}

class PseudoLRU(n: Int)
{
  val state_reg = Reg(Bits(width = n))
  def access(way: UInt) {
    state_reg := get_next_state(state_reg,way)
  }
  def get_next_state(state: UInt, way: UInt) = {
    var next_state = state
    var idx = UInt(1,1)
    for (i <- log2Up(n)-1 to 0 by -1) {
      val bit = way(i)
      val mask = (UInt(1,n) << idx)(n-1,0)
      next_state = next_state & ~mask | Mux(bit, UInt(0), mask)
      //next_state.bitSet(idx, !bit)
      idx = Cat(idx, bit)
    }
    next_state
  }
  def replace = get_replace_way(state_reg)
  def get_replace_way(state: Bits) = {
    var idx = UInt(1,1)
    for (i <- 0 until log2Up(n))
      idx = Cat(idx, state(idx))
    idx(log2Up(n)-1,0)
  }
}

class SeqPLRU(n_sets: Int, n_ways: Int) extends SeqReplacementPolicy {
  val state = SeqMem(n_sets, Bits(width = n_ways-1))
  val logic = new PseudoLRU(n_ways)
  val current_state = Wire(Bits())
  val plru_way = logic.get_replace_way(current_state)
  val next_state = Wire(Bits())

  def access(set: UInt) = {
    current_state := Cat(state.read(set), Bits(0, width = 1))
  }

  def update(valid: Bool, hit: Bool, set: UInt, way: UInt) = {
    val update_way = Mux(hit, way, plru_way)
    next_state := logic.get_next_state(current_state, update_way)
    when (valid) { state.write(set, next_state(n_ways-1,1)) }
  }

  def way = plru_way
}

abstract class Metadata(implicit p: Parameters) extends CacheBundle()(p) {
  val tag = Bits(width = tagBits)
  val coh: CoherenceMetadata
}

class MetaReadReq(implicit p: Parameters) extends CacheBundle()(p) {
  val idx  = Bits(width = idxBits)
  val way_en = Bits(width = nWays)
}

class MetaWriteReq[T <: Metadata](gen: T)(implicit p: Parameters) extends MetaReadReq()(p) {
  val data = gen.cloneType
  override def cloneType = new MetaWriteReq(gen)(p).asInstanceOf[this.type]
}

class MetadataArray[T <: Metadata](onReset: () => T)(implicit p: Parameters) extends CacheModule()(p) {
  val rstVal = onReset()
  val io = new Bundle {
    val read = Decoupled(new MetaReadReq).flip
    val write = Decoupled(new MetaWriteReq(rstVal)).flip
    val resp = Vec(nWays, rstVal.cloneType).asOutput
  }
  val rst_cnt = Reg(init=UInt(0, log2Up(nSets+1)))
  val rst = rst_cnt < UInt(nSets)
  val waddr = Mux(rst, rst_cnt, io.write.bits.idx)
  val wdata = Mux(rst, rstVal, io.write.bits.data).toBits
  val wmask = Mux(rst, SInt(-1), io.write.bits.way_en.toSInt).toBools
  val rmask = Mux(rst, SInt(-1), io.read.bits.way_en.toSInt).toBools
  when (rst) { rst_cnt := rst_cnt+UInt(1) }

  val metabits = rstVal.getWidth

  if (hasSplitMetadata) {
    val tag_arrs = List.fill(nWays){ SeqMem(nSets, UInt(width = metabits)) }
    val tag_readout = Wire(Vec(nWays,rstVal.cloneType))
    val tags_vec = Wire(Vec(nWays, UInt(width = metabits)))
    (0 until nWays).foreach { (i) =>
      when (rst || (io.write.valid && wmask(i))) {
        tag_arrs(i).write(waddr, wdata)
      }
      tags_vec(i) := tag_arrs(i).read(io.read.bits.idx, io.read.valid && rmask(i))
    }
    io.resp := io.resp.fromBits(tags_vec.toBits)
  } else {
    val tag_arr = SeqMem(nSets, Vec(nWays, UInt(width = metabits)))
    when (rst || io.write.valid) {
      tag_arr.write(waddr, Vec.fill(nWays)(wdata), wmask)
    }
    val tags = tag_arr.read(io.read.bits.idx, io.read.valid).toBits
    io.resp := io.resp.fromBits(tags)
  }

  io.read.ready := !rst && !io.write.valid // so really this could be a 6T RAM
  io.write.ready := !rst
}

case object L2DirectoryRepresentation extends Field[DirectoryRepresentation]

trait HasOuterCacheParameters extends HasCacheParameters with HasCoherenceAgentParameters {
  val cacheId = p(CacheId)
  val idxLSB = cacheIdBits
  val idxMSB = idxLSB + idxBits - 1
  val tagLSB = idxLSB + idxBits
  def inSameSet(addr1: UInt, addr2: UInt): Bool = addr1(idxMSB,idxLSB) === addr2(idxMSB,idxLSB)
  def haveSameTag(addr1: UInt, addr2: UInt): Bool = addr1 >> UInt(tagLSB) === addr2 >> UInt(tagLSB)
  //val blockAddrBits = p(TLBlockAddrBits)
  val refillCyclesPerBeat = outerDataBits/rowBits
  val refillCycles = refillCyclesPerBeat*outerDataBeats
  val internalDataBeats = p(CacheBlockBytes)*8/rowBits
  require(refillCyclesPerBeat == 1)
  val amoAluOperandBits = p(AmoAluOperandBits)
  require(amoAluOperandBits <= innerDataBits)
  require(rowBits == innerDataBits) // TODO: relax this by improving s_data_* states
  val nSecondaryMisses = p(NSecondaryMisses)
  val isLastLevelCache = true
  val alwaysWriteFullBeat = !p(ECCCode).isEmpty
}

abstract class L2HellaCacheModule(implicit val p: Parameters) extends Module
    with HasOuterCacheParameters {
  def doInternalOutputArbitration[T <: Data : ClassTag](
      out: DecoupledIO[T],
      ins: Seq[DecoupledIO[T]]) {
    val arb = Module(new RRArbiter(out.bits, ins.size))
    out <> arb.io.out
    arb.io.in <> ins 
  }

  def doInternalInputRouting[T <: Bundle with HasL2Id](in: ValidIO[T], outs: Seq[ValidIO[T]]) {
    outs.map(_.bits := in.bits)
    outs.zipWithIndex.map { case (o,i) => o.valid := in.valid && in.bits.id === UInt(i) }
  }
}

abstract class L2HellaCacheBundle(implicit val p: Parameters) extends ParameterizedBundle()(p)
  with HasOuterCacheParameters

trait HasL2Id extends HasCoherenceAgentParameters {
  val id = UInt(width  = log2Up(nTransactors + 1))
}

trait HasL2InternalRequestState extends HasOuterCacheParameters {
  val tag_match = Bool()
  val meta = new L2Metadata
  val way_en = Bits(width = nWays)
}

trait HasL2BeatAddr extends HasOuterCacheParameters {
  val addr_beat = UInt(width = log2Up(refillCycles))
}

trait HasL2Data extends HasOuterCacheParameters
    with HasL2BeatAddr {
  val data = UInt(width = rowBits)
  def hasData(dummy: Int = 0) = Bool(true)
  def hasMultibeatData(dummy: Int = 0) = Bool(refillCycles > 1)
}

class L2Metadata(implicit p: Parameters) extends Metadata()(p) with HasOuterCacheParameters {
  val coh = new HierarchicalMetadata
}

object L2Metadata {
  def apply(tag: Bits, coh: HierarchicalMetadata)
      (implicit p: Parameters): L2Metadata = {
    val meta = Wire(new L2Metadata)
    meta.tag := tag
    meta.coh := coh
    meta
  }

  def apply(
        tag: Bits,
        inner: ManagerMetadata,
        outer: ClientMetadata)(implicit p: Parameters): L2Metadata = {
    val coh = Wire(new HierarchicalMetadata)
    coh.inner := inner
    coh.outer := outer
    apply(tag, coh)
  }
}

class L2MetaReadReq(implicit p: Parameters) extends MetaReadReq()(p) with HasL2Id {
  val tag = Bits(width = tagBits)
}

class L2MetaWriteReq(implicit p: Parameters) extends MetaWriteReq[L2Metadata](new L2Metadata)(p)
    with HasL2Id {
  override def cloneType = new L2MetaWriteReq().asInstanceOf[this.type]
}

class L2MetaResp(implicit p: Parameters) extends L2HellaCacheBundle()(p)
  with HasL2Id 
  with HasL2InternalRequestState

trait HasL2MetaReadIO extends HasOuterCacheParameters {
  val read = Decoupled(new L2MetaReadReq)
  val resp = Valid(new L2MetaResp).flip
}

trait HasL2MetaWriteIO extends HasOuterCacheParameters {
  val write = Decoupled(new L2MetaWriteReq)
}

class L2MetaRWIO(implicit p: Parameters) extends L2HellaCacheBundle()(p)
  with HasL2MetaReadIO
  with HasL2MetaWriteIO

class L2MetadataArray(implicit p: Parameters) extends L2HellaCacheModule()(p) {
  val io = new L2MetaRWIO().flip

  def onReset = L2Metadata(UInt(0), HierarchicalMetadata.onReset)
  val meta = Module(new MetadataArray(onReset _))
  meta.io.read <> io.read
  meta.io.write <> io.write
  val way_en_1h = (Vec.fill(nWays){Bool(true)}).toBits
  val s1_way_en_1h = RegEnable(way_en_1h, io.read.valid)
  meta.io.read.bits.way_en := way_en_1h

  val s1_tag = RegEnable(io.read.bits.tag, io.read.valid)
  val s1_id = RegEnable(io.read.bits.id, io.read.valid)
  def wayMap[T <: Data](f: Int => T) = Vec((0 until nWays).map(f))
  val s1_clk_en = Reg(next = io.read.fire())
  val s1_tag_eq_way = wayMap((w: Int) => meta.io.resp(w).tag === s1_tag)
  val s1_tag_match_way = wayMap((w: Int) => s1_tag_eq_way(w) && meta.io.resp(w).coh.outer.isValid() && s1_way_en_1h(w).toBool).toBits
  val s1_idx = RegEnable(io.read.bits.idx, io.read.valid) // deal with stalls?
  val s2_tag_match_way = RegEnable(s1_tag_match_way, s1_clk_en)
  val s2_tag_match = s2_tag_match_way.orR
  val s2_hit_coh = Mux1H(s2_tag_match_way, wayMap((w: Int) => RegEnable(meta.io.resp(w).coh, s1_clk_en)))

  val replacer = p(L2Replacer)()
  val s1_hit_way = Wire(Bits())
  s1_hit_way := Bits(0)
  (0 until nWays).foreach(i => when (s1_tag_match_way(i)) { s1_hit_way := Bits(i) })
  replacer.access(io.read.bits.idx)
  replacer.update(s1_clk_en, s1_tag_match_way.orR, s1_idx, s1_hit_way)

  val s1_replaced_way_en = UIntToOH(replacer.way)
  val s2_replaced_way_en = UIntToOH(RegEnable(replacer.way, s1_clk_en))
  val s2_repl_meta = Mux1H(s2_replaced_way_en, wayMap((w: Int) => 
    RegEnable(meta.io.resp(w), s1_clk_en && s1_replaced_way_en(w))).toSeq)

  io.resp.valid := Reg(next = s1_clk_en)
  io.resp.bits.id := RegEnable(s1_id, s1_clk_en)
  io.resp.bits.tag_match := s2_tag_match
  io.resp.bits.meta := Mux(s2_tag_match, 
    L2Metadata(s2_repl_meta.tag, s2_hit_coh), 
    s2_repl_meta)
  io.resp.bits.way_en := Mux(s2_tag_match, s2_tag_match_way, s2_replaced_way_en)
}

class L2DataReadReq(implicit p: Parameters) extends L2HellaCacheBundle()(p)
    with HasL2BeatAddr 
    with HasL2Id {
  val addr_idx = UInt(width = idxBits)
  val way_en = Bits(width = nWays)
}

object L2DataReadReq {
  def apply(
        id: UInt,
        way_en: UInt,
        addr_idx: UInt,
        addr_beat: UInt)(implicit p: Parameters) = {
    val req = Wire(new L2DataReadReq)
    req.id := id
    req.way_en := way_en
    req.addr_idx := addr_idx
    req.addr_beat := addr_beat
    req
  }
}

class L2DataWriteReq(implicit p: Parameters) extends L2DataReadReq()(p)
    with HasL2Data {
  val wmask  = Bits(width = rowBits/8)
}

object L2DataWriteReq {
  def apply(
        id: UInt,
        way_en: UInt,
        addr_idx: UInt,
        addr_beat: UInt,
        wmask: UInt,
        data: UInt)(implicit p: Parameters) = {
    val req = Wire(new L2DataWriteReq)
    req.id := id
    req.way_en := way_en
    req.addr_idx := addr_idx
    req.addr_beat := addr_beat
    req.wmask := wmask
    req.data := data
    req
  }
}

class L2DataResp(implicit p: Parameters) extends L2HellaCacheBundle()(p)
  with HasL2Id
  with HasL2Data

trait HasL2DataReadIO extends HasOuterCacheParameters { 
  val read = Decoupled(new L2DataReadReq)
  val resp = Valid(new L2DataResp).flip
}

trait HasL2DataWriteIO extends HasOuterCacheParameters { 
  val write = Decoupled(new L2DataWriteReq)
}

class L2DataRWIO(implicit p: Parameters) extends L2HellaCacheBundle()(p)
  with HasL2DataReadIO
  with HasL2DataWriteIO

class L2DataArray(delay: Int)(implicit p: Parameters) extends L2HellaCacheModule()(p) {
  val io = new L2DataRWIO().flip

  val array = SeqMem(nWays*nSets*refillCycles, Vec(rowBits/8, Bits(width=8)))
  val ren = !io.write.valid && io.read.valid
  val raddr = Cat(OHToUInt(io.read.bits.way_en), io.read.bits.addr_idx, io.read.bits.addr_beat)
  val waddr = Cat(OHToUInt(io.write.bits.way_en), io.write.bits.addr_idx, io.write.bits.addr_beat)
  val wdata = Vec.tabulate(rowBits/8)(i => io.write.bits.data(8*(i+1)-1,8*i))
  val wmask = io.write.bits.wmask.toBools
  when (io.write.valid) { array.write(waddr, wdata, wmask) }

  val r_req = Pipe(io.read.fire(), io.read.bits)
  io.resp := Pipe(r_req.valid, r_req.bits, delay)
  io.resp.bits.data := Pipe(r_req.valid, array.read(raddr, ren).toBits, delay).bits
  io.read.ready := !io.write.valid
  io.write.ready := Bool(true)
}

class L2HellaCacheBank(implicit p: Parameters) extends HierarchicalCoherenceAgent()(p)
    with HasOuterCacheParameters {
  require(isPow2(nSets))
  require(isPow2(nWays)) 

  val meta = Module(new L2MetadataArray) // TODO: add delay knob
  val data = Module(new L2DataArray(1))
  val tshrfile = Module(new TSHRFile)
  io.inner <> tshrfile.io.inner
  io.outer <> tshrfile.io.outer
  tshrfile.io.incoherent <> io.incoherent
  meta.io <> tshrfile.io.meta
  data.io <> tshrfile.io.data
}

class TSHRFileIO(implicit p: Parameters) extends HierarchicalTLIO()(p) {
  val meta = new L2MetaRWIO
  val data = new L2DataRWIO
}

class TSHRFile(implicit p: Parameters) extends L2HellaCacheModule()(p)
    with HasCoherenceAgentWiringHelpers {
  val io = new TSHRFileIO

  // Create TSHRs for outstanding transactions
  val trackerList = 
    (0 until nReleaseTransactors).map(id =>
      Module(new CacheVoluntaryReleaseTracker(id))) ++
    (nReleaseTransactors until nTransactors).map(id =>
      Module(new CacheAcquireTracker(id)))
  
  // WritebackUnit evicts data from L2, including invalidating L1s
  val wb = Module(new L2WritebackUnit(nTransactors))
  val trackerAndWbIOs = trackerList.map(_.io) :+ wb.io
  doInternalOutputArbitration(wb.io.wb.req, trackerList.map(_.io.wb.req))
  doInternalInputRouting(wb.io.wb.resp, trackerList.map(_.io.wb.resp))

  // Propagate incoherence flags
  (trackerList.map(_.io.incoherent) :+ wb.io.incoherent) foreach { _ := io.incoherent }

  // Handle acquire transaction initiation
  val irel_vs_iacq_conflict =
        io.inner.acquire.valid &&
        io.inner.release.valid &&
        inSameSet(io.inner.acquire.bits.addr_block, io.inner.release.bits.addr_block)
  doInputRoutingWithAllocation(
    io.inner.acquire,
    trackerList.map(_.io.inner.acquire),
    trackerList.map(_.io.matches.iacq),
    trackerList.map(_.io.alloc.iacq),
    allocOverride = !irel_vs_iacq_conflict)

  assert(PopCount(trackerList.map(_.io.alloc.iacq)) <= UInt(1),
    "At most a single tracker should now be allocated for any given Acquire")

  // Wire releases from clients
  doInputRoutingWithAllocation(
    io.inner.release,
    trackerAndWbIOs.map(_.inner.release),
    trackerAndWbIOs.map(_.matches.irel),
    trackerAndWbIOs.map(_.alloc.irel))

  assert(PopCount(trackerAndWbIOs.map(_.alloc.irel)) <= UInt(1),
    "At most a single tracker should now be allocated for any given Release")

  // Wire probe requests and grant reply to clients, finish acks from clients
  doOutputArbitration(io.inner.probe, trackerList.map(_.io.inner.probe) :+ wb.io.inner.probe)
  doOutputArbitration(io.inner.grant, trackerList.map(_.io.inner.grant) :+ wb.io.inner.grant)
  doInputRouting(io.inner.finish, trackerList.map(_.io.inner.finish))

  // Create an arbiter for the one memory port
  val outerList = trackerList.map(_.io.outer) :+ wb.io.outer
  val outer_arb = Module(new ClientTileLinkIOArbiter(outerList.size)
                                                    (p.alterPartial({ case TLId => p(OuterTLId)})))
  outer_arb.io.in <> outerList
  io.outer <> outer_arb.io.out

  // Wire local memory arrays
  doInternalOutputArbitration(io.meta.read, trackerList.map(_.io.meta.read))
  doInternalOutputArbitration(io.meta.write, trackerList.map(_.io.meta.write))
  doInternalOutputArbitration(io.data.read, trackerList.map(_.io.data.read) :+ wb.io.data.read)
  doInternalOutputArbitration(io.data.write, trackerList.map(_.io.data.write))
  doInternalInputRouting(io.meta.resp, trackerList.map(_.io.meta.resp))
  doInternalInputRouting(io.data.resp, trackerList.map(_.io.data.resp) :+ wb.io.data.resp)
}


class L2XactTrackerIO(implicit p: Parameters) extends HierarchicalXactTrackerIO()(p) {
  val data = new L2DataRWIO
  val meta = new L2MetaRWIO
  val wb = new L2WritebackIO
}

trait HasRowBeatCounters extends HasOuterCacheParameters with HasPendingBits {
  def connectDataBeatCounter[S <: L2HellaCacheBundle](inc: Bool, data: S, beat: UInt, full_block: Bool) = {
    if(data.refillCycles > 1) {
      val (multi_cnt, multi_done) = Counter(full_block && inc, data.refillCycles)
      (Mux(!full_block, beat, multi_cnt), Mux(!full_block, inc, multi_done))
    } else { (UInt(0), inc) }
  }

  def connectInternalDataBeatCounter[T <: L2HellaCacheBundle with HasL2BeatAddr](
      in: DecoupledIO[T],
      beat: UInt = UInt(0),
      full_block: Bool = Bool(true)): (UInt, Bool) = {
    connectDataBeatCounter(in.fire(), in.bits, beat, full_block)
  }

  def connectInternalDataBeatCounter[T <: L2HellaCacheBundle with HasL2Data](
      in: ValidIO[T],
      full_block: Bool): Bool = {
    connectDataBeatCounter(in.valid, in.bits, UInt(0), full_block)._2
  }

  def addPendingBitInternal[T <: L2HellaCacheBundle with HasL2BeatAddr](in: DecoupledIO[T]) =
    Fill(in.bits.refillCycles, in.fire()) & UIntToOH(in.bits.addr_beat)

  def addPendingBitInternal[T <: L2HellaCacheBundle with HasL2BeatAddr](in: ValidIO[T]) =
    Fill(in.bits.refillCycles, in.valid) & UIntToOH(in.bits.addr_beat)

  def dropPendingBit[T <: L2HellaCacheBundle with HasL2BeatAddr] (in: DecoupledIO[T]) =
    ~Fill(in.bits.refillCycles, in.fire()) | ~UIntToOH(in.bits.addr_beat)

  def dropPendingBitInternal[T <: L2HellaCacheBundle with HasL2BeatAddr] (in: ValidIO[T]) =
    ~Fill(in.bits.refillCycles, in.valid) | ~UIntToOH(in.bits.addr_beat)
}

trait HasAMOALU extends HasAcquireMetadataBuffer
    with HasByteWriteMaskBuffer
    with HasOuterCacheParameters {
  val io: L2XactTrackerIO

  // Provide a single ALU per tracker to merge Puts and AMOs with data being
  // refilled, written back, or extant in the cache
  val amoalu = Module(new AMOALU(rhsIsAligned = true))
  amoalu.io.addr := Cat(xact_addr_block, xact_addr_beat, xact_addr_byte)
  amoalu.io.cmd := xact_op_code
  amoalu.io.typ := xact_op_size
  amoalu.io.lhs := io.data.resp.bits.data // default, overwritten by calls to mergeData
  amoalu.io.rhs := data_buffer.head  // default, overwritten by calls to mergeData
  val amo_result = Reg(init = UInt(0, innerDataBits))

  // Utility function for applying any buffered stored data to the cache line
  // before storing it back into the data array
  override def mergeData(dataBits: Int)(beat: UInt, incoming: UInt) {
    val old_data = incoming     // Refilled, written back, or de-cached data
    val new_data = data_buffer(beat) // Newly Put data is already in the buffer
    val amo_shift_bits = xact_amo_shift_bytes << 3
    amoalu.io.lhs := old_data >> amo_shift_bits
    amoalu.io.rhs := new_data >> amo_shift_bits
    val wmask = FillInterleaved(8, wmask_buffer(beat))
    data_buffer(beat) := ~wmask & old_data |
                          wmask & Mux(xact_iacq.isAtomic(), amoalu.io.out << amo_shift_bits, new_data)
    when(xact_iacq.isAtomic() && xact_addr_beat === beat) { amo_result := old_data }
  }

  // TODO: Deal with the possibility that rowBits != tlDataBits
  def mergeDataInternal[T <: L2HellaCacheBundle with HasL2Data with HasL2BeatAddr](in: ValidIO[T]) {
    when(in.valid) { mergeData(rowBits)(in.bits.addr_beat, in.bits.data) }
  }
}

trait TriggersWritebacks extends HasCoherenceMetadataBuffer {
  def wbReq(next_state: UInt) {
    io.wb.req.valid := state === s_wb_req
    io.wb.req.bits.id := UInt(trackerId)
    io.wb.req.bits.idx := xact_addr_idx
    io.wb.req.bits.tag := xact_old_meta.tag
    io.wb.req.bits.coh := xact_old_meta.coh
    io.wb.req.bits.way_en := xact_way_en

    when(state === s_wb_req && io.wb.req.ready) { state := s_wb_resp }
    when(state === s_wb_resp && io.wb.resp.valid) { state := s_outer_acquire }
  }
}

trait HasCoherenceMetadataBuffer extends HasOuterCacheParameters
    with HasBlockAddressBuffer
    with TriggersInnerProbes {
  val trackerId: Int
  val io: L2XactTrackerIO

  lazy val xact_way_en = Reg{ Bits(width = nWays) }
  lazy val xact_old_meta = Reg{ new L2Metadata }
  lazy val pending_coh = Reg{ xact_old_meta.coh }
  lazy val pending_meta_write = Reg(init = Bool(false))

  lazy val inner_coh = pending_coh.inner
  lazy val outer_coh = pending_coh.outer

  lazy val xact_addr_idx = xact_addr_block(idxMSB,idxLSB)
  lazy val xact_addr_tag = xact_addr_block >> UInt(tagLSB) 

  // Utility function for updating the metadata that will be kept in this cache
  def updatePendingCohWhen(flag: Bool, next: HierarchicalMetadata) {
    when(flag && pending_coh =/= next) {
      pending_meta_write := Bool(true)
      pending_coh := next
    }
  }


  def metaRead(next_state: UInt) {
    io.meta.read.valid := state === s_meta_read
    io.meta.read.bits.id := UInt(trackerId)
    io.meta.read.bits.idx := xact_addr_idx
    io.meta.read.bits.tag := xact_addr_tag

    when(state === s_meta_read && io.meta.read.ready) { state := s_meta_resp }

    when(state === s_meta_resp && io.meta.resp.valid) {
      val coh = io.meta.resp.bits.meta.coh
      xact_old_meta := io.meta.resp.bits.meta
      xact_way_en := io.meta.resp.bits.way_en
      state := next_state
    }
  }
  
  def metaWrite(to_write: L2Metadata, next_state: UInt) {
    io.meta.write.valid := state === s_meta_write
    io.meta.write.bits.id := UInt(trackerId)
    io.meta.write.bits.idx := xact_addr_idx
    io.meta.write.bits.way_en := xact_way_en
    io.meta.write.bits.data := to_write

    when(state === s_meta_write && io.meta.write.ready) { state := next_state }
  }
}

class CacheVoluntaryReleaseTracker(trackerId: Int)(implicit p: Parameters) extends VoluntaryReleaseTracker(trackerId)(p)
    with HasRowBeatCounters
    with HasCoherenceMetadataBuffer
    with HasDataBuffer {
  val io = new L2XactTrackerIO
  pinAllReadyValidLow(io)

  // Avoid metatdata races with writebacks
  route(inSameSet(io.iacq().addr_block, xact_addr_block))

  // Initialize and accept pending Release beats
  accept(s_meta_read)

  // Begin a transaction by getting the current block metadata
  metaRead(s_busy)

  // Write the voluntarily written back data to this cache
  write(io.data.write, 
        L2DataWriteReq(
          id = UInt(trackerId),
          way_en = xact_way_en,
          addr_idx = xact_addr_idx,
          addr_beat = curr_write_beat,
          wmask = ~UInt(0, io.data.write.bits.wmask.getWidth),
          data = data_buffer(curr_write_beat)),
        dropPendingBit(io.data.write))

  // Send an acknowledgement
  acknowledge(!pending_writes.orR, Bool(false))

  when(state === s_busy && all_pending_done) { state := s_meta_write  }

  // End a transaction by updating the block metadata
  metaWrite(
    L2Metadata(
      tag = xact_addr_tag,
      inner = xact_old_meta.coh.inner.onRelease(xact_vol_irel),
      outer = Mux(xact_vol_irel.hasData(),
                  xact_old_meta.coh.outer.onHit(M_XWR),
                  xact_old_meta.coh.outer)),
    s_idle)

  // Checks for illegal behavior
  assert(!(state === s_meta_resp && io.meta.resp.valid && !io.meta.resp.bits.tag_match),
    "VoluntaryReleaseTracker accepted Release for a block not resident in this cache!")
  assert(!(state === s_idle && io.inner.release.fire() && !io.irel().isVoluntary()),
    "VoluntaryReleaseTracker accepted Release that wasn't voluntary!")
}

class CacheAcquireTracker(trackerId: Int)(implicit p: Parameters) extends AcquireTracker(trackerId)(p)
    with HasCoherenceMetadataBuffer
    with HasRowBeatCounters
    with HasAMOALU
    with TriggersWritebacks {
  val io = new L2XactTrackerIO
  pinAllReadyValidLow(io)

  // TileLink allows for Gets-under-Get
  // and Puts-under-Put, and either may also merge with a preceding prefetch
  // that requested the correct permissions (via op_code)
  def acquiresAreMergeable(sec: AcquireMetadata): Bool = {
    val allowedTypes = List((Acquire.getType, Acquire.getType),
                       (Acquire.putType, Acquire.putType),
                       (Acquire.putBlockType, Acquire.putBlockType),
                       (Acquire.getPrefetchType, Acquire.getPrefetchType),
                       (Acquire.putPrefetchType, Acquire.putPrefetchType),
                       (Acquire.getPrefetchType, Acquire.getType),
                       (Acquire.putPrefetchType, Acquire.putType),
                       (Acquire.putPrefetchType, Acquire.putBlockType))
    allowedTypes.map { case(a, b) => xact_iacq.isBuiltInType(a) && sec.isBuiltInType(b) }.reduce(_||_) &&
      xact_op_code === sec.op_code() &&
      sec.conflicts(xact_addr_block) &&
      xact_allocate
  }

  // These IOs are used for routing in the parent
  val iacq_in_same_set = inSameSet(xact_addr_block, io.iacq().addr_block)
  val irel_in_same_set = inSameSet(xact_addr_block, io.irel().addr_block)
  val before_wb_alloc = Vec(s_meta_read, s_meta_resp, s_wb_req).contains(state)
  route(
    iacq_in_same_set, 
    Mux(before_wb_alloc, irel_in_same_set, io.irel().conflicts(xact_addr_block)))

  // Actual transaction processing logic begins here:
  //
  // First, take care of accpeting new acquires or secondary misses
  val iacq_can_merge = acquiresAreMergeable(io.iacq()) &&
                         state =/= s_idle && state =/= s_meta_write &&
                         !all_pending_done &&
                         !io.inner.release.fire() &&
                         !io.outer.grant.fire() &&
                         !io.data.resp.valid &&
                         ignt_q.io.enq.ready && ignt_q.io.deq.valid

  accept(iacq_can_merge, Bool(true), s_meta_read)
  when(state === s_idle && io.inner.acquire.valid && io.alloc.iacq) {
    amo_result := UInt(0)
    pending_meta_write := Bool(false)
  }

  // Begin a transaction by getting the current block metadata
  // Defined here because of Chisel default wire demands, used in s_meta_resp
  val pending_coh_on_hit = HierarchicalMetadata(
    io.meta.resp.bits.meta.coh.inner,
    io.meta.resp.bits.meta.coh.outer.onHit(xact_op_code))
  val pending_coh_on_miss = HierarchicalMetadata.onReset
  val coh = io.meta.resp.bits.meta.coh
  val tag_match = io.meta.resp.bits.tag_match
  val is_hit = (if(!isLastLevelCache) tag_match && coh.outer.isHit(xact_op_code)
                else tag_match && coh.outer.isValid())
  val needs_writeback = !tag_match &&
                        xact_allocate && 
                        (coh.outer.requiresVoluntaryWriteback() ||
                           coh.inner.requiresProbesOnVoluntaryWriteback())
  val needs_inner_probes = tag_match && coh.inner.requiresProbes(xact_iacq)
  val should_update_meta = !tag_match && xact_allocate ||
                           is_hit && pending_coh_on_hit =/= coh
  metaRead(
    Mux(needs_writeback, s_wb_req,
      Mux(needs_inner_probes, s_inner_probe,
        Mux(!is_hit, s_outer_acquire, s_busy))))

  updatePendingCohWhen(
    io.meta.resp.valid, 
    Mux(is_hit, pending_coh_on_hit,
      Mux(tag_match, coh, pending_coh_on_miss)))

  when(state === s_meta_resp && io.meta.resp.valid) {
    // If some kind of Put is marked no-allocate but is already in the cache,
    // we need to write its data to the data array
    when(is_hit && !xact_allocate && xact_iacq.hasData()) {
      pending_writes := addPendingBitsFromAcquire(xact_iacq)
      xact_allocate := Bool(true)
    }
    when (needs_inner_probes) { 
      initializeProbes(coh.inner.full(), xact_iacq.client_id, xact_iacq.requiresSelfProbe())
    }
    //pending_meta_write := should_update_meta TODO what edge case was this covering?
  }

  // Issue a request to the writeback unit
  wbReq(s_outer_acquire)

  // Track which clients yet need to be probed and make Probe message
  // If we're probing, we know the tag matches, so if this is the
  // last level cache, we can use the data without upgrading permissions
  val skip_outer_acquire = 
    (if(!isLastLevelCache) xact_old_meta.coh.outer.isHit(xact_op_code)
     else xact_old_meta.coh.outer.isValid())

  innerProbe(
    pending_coh.inner.makeProbe(curr_probe_dst, xact_iacq, xact_addr_block),
    Mux(!skip_outer_acquire, s_outer_acquire, s_busy))

  // Handle incoming releases from clients, which may reduce sharer counts
  // and/or write back dirty data, and may be unexpected voluntary releases
  val irel_can_merge = io.irel().conflicts(xact_addr_block) &&
                         io.irel().isVoluntary() &&
                         !Vec(s_idle, s_meta_read, s_meta_resp, s_meta_write).contains(state) &&
                         !all_pending_done &&
                         !io.outer.grant.fire() &&
                         !io.inner.grant.fire() &&
                         !pending_vol_ignt

  innerRelease(irel_can_merge)

  val pending_coh_on_irel = HierarchicalMetadata(
                              pending_coh.inner.onRelease(io.irel()), // Drop sharer
                              Mux(io.irel().hasData(), // Dirty writeback
                                pending_coh.outer.onHit(M_XWR),
                                pending_coh.outer))
  updatePendingCohWhen(io.inner.release.fire(), pending_coh_on_irel)

  mergeDataInner(io.inner.release)

  // Send outer request
  outerAcquire(
    Mux(xact_allocate,
      xact_old_meta.coh.outer.makeAcquire(
        op_code = xact_op_code,
        client_xact_id = UInt(0),
        addr_block = xact_addr_block),
      BuiltInAcquireBuilder(
        a_type = xact_iacq.a_type,
        client_xact_id = UInt(0), // TODO  UInt(trackerId)? done in arbiter
        addr_block = xact_addr_block,
        addr_beat = oacq_data_idx, // TODO xact_addr_beat?
        data = data_buffer(oacq_data_idx),
        addr_byte = xact_addr_byte,
        operand_size = xact_op_size,
        opcode = xact_op_code,
        wmask = wmask_buffer(oacq_data_idx),
        alloc = Bool(false))
        (p.alterPartial({ case TLId => p(OuterTLId)}))),
    s_busy)

  // Handle the response from outer memory
  io.outer.grant.ready := state === s_busy

  val pending_coh_on_ognt = HierarchicalMetadata(
                              ManagerMetadata.onReset,
                              pending_coh.outer.onGrant(io.outer.grant.bits, xact_op_code))
  updatePendingCohWhen(ognt_data_done, pending_coh_on_ognt)

  mergeDataOuter(io.outer.grant)

  // Send read
  read(io.data.read, 
        L2DataReadReq(
          id = UInt(trackerId),
          way_en = xact_way_en,
          addr_idx = xact_addr_idx,
          addr_beat = curr_read_beat),
        dropPendingBit(io.data.read))

  // Get resp
  pending_resps := (pending_resps & dropPendingBitInternal(io.data.resp)) |
                     addPendingBitInternal(io.data.read)

  mergeDataInternal(io.data.resp)

  // Do write
  write(io.data.write, 
        L2DataWriteReq(
          id = UInt(trackerId),
          way_en = xact_way_en,
          addr_idx = xact_addr_idx,
          addr_beat = curr_write_beat,
          wmask = ~UInt(0, io.data.write.bits.wmask.getWidth),
          data = data_buffer(curr_write_beat)),
        dropPendingBit(io.data.write))

  // Acknowledge or respond with data
  val ignt_from_iacq = pending_coh.inner.makeGrant(
                              sec = ignt_q.io.deq.bits,
                              manager_xact_id = UInt(trackerId), 
                              data = Mux(xact_iacq.isAtomic(),
                                       amo_result,
                                       data_buffer(ignt_data_idx)))
  innerGrant(ignt_from_iacq, addPendingBitInternal(io.data.resp))

  val pending_coh_on_ignt = HierarchicalMetadata(
                              pending_coh.inner.onGrant(io.ignt()),
                              Mux(ognt_data_done,
                                pending_coh_on_ognt.outer,
                                pending_coh.outer))
  updatePendingCohWhen(io.inner.grant.fire() && io.ignt().last(), pending_coh_on_ignt)

  // We must wait for as many Finishes as we sent Grants
  io.inner.finish.ready := state === s_busy

  // Wait for everything to quiesce
  when(state === s_busy && all_pending_done) {
    wmask_buffer.foreach { w => w := UInt(0) } // This is the only reg that must be clear in s_idle
    state := Mux(pending_meta_write, s_meta_write, s_idle)
  }

  // End a transaction by updating the block metadata
  metaWrite(L2Metadata(xact_addr_tag, pending_coh), s_idle)
                                        
}

class L2WritebackReq(implicit p: Parameters) extends L2Metadata()(p) with HasL2Id {
  val idx  = Bits(width = idxBits)
  val way_en = Bits(width = nWays)
}

class L2WritebackResp(implicit p: Parameters) extends L2HellaCacheBundle()(p) with HasL2Id

class L2WritebackIO(implicit p: Parameters) extends L2HellaCacheBundle()(p) {
  val req = Decoupled(new L2WritebackReq)
  val resp = Valid(new L2WritebackResp).flip
}

class L2WritebackUnitIO(implicit p: Parameters) extends HierarchicalXactTrackerIO()(p) {
  val wb = new L2WritebackIO().flip
  val data = new L2DataRWIO
}

abstract class L2XactTracker(implicit p: Parameters) extends XactTracker()(p)
    with HasOuterCacheParameters
    with HasRowBeatCounters

class L2WritebackUnit(trackerId: Int)(implicit p: Parameters) extends L2XactTracker()(p) {
  val io = new L2WritebackUnitIO
  pinAllReadyValidLow(io)

  val s_idle :: s_inner_probe :: s_busy :: s_outer_grant :: s_wb_resp :: Nil = Enum(UInt(), 5)
  val state = Reg(init=s_idle)

  val xact = Reg(new L2WritebackReq)
  val data_buffer = Reg(init=Vec.fill(innerDataBeats)(UInt(0, width = innerDataBits)))
  val xact_vol_ir_r_type = Reg{ io.irel().r_type }
  val xact_vol_ir_src = Reg{ io.irel().client_id }
  val xact_vol_ir_client_xact_id = Reg{ io.irel().client_xact_id }

  val xact_addr_block = if (cacheIdBits == 0)
                          Cat(xact.tag, xact.idx)
                        else
                          Cat(xact.tag, xact.idx, UInt(cacheId, cacheIdBits))
  val xact_vol_irel = Release(
                        src = xact_vol_ir_src,
                        voluntary = Bool(true),
                        r_type = xact_vol_ir_r_type,
                        client_xact_id = xact_vol_ir_client_xact_id,
                        addr_block = xact_addr_block)

  val pending_irels = connectTwoWayBeatCounter(
    max = io.inner.tlNCachingClients,
    up = io.inner.probe,
    down = io.inner.release,
    trackDown = (r: Release) => !r.isVoluntary())._1

  val pending_vol_ignt = connectTwoWayBeatCounter(
    max = 1,
    up = io.inner.release,
    down = io.inner.grant,
    trackUp = (r: Release) => r.isVoluntary(),
    trackDown = (g: Grant) => g.isVoluntary())._1

  val (pending_ognt, orel_data_idx, orel_data_done, ognt_data_idx, ognt_data_done) =
    connectTwoWayBeatCounter(
      max = 1,
      up = io.outer.release,
      down = io.outer.grant)

  val pending_iprbs = Reg(init = Bits(0, width = io.inner.tlNCachingClients))
  val pending_reads = Reg(init=Bits(0, width = io.inner.tlDataBeats))
  val pending_resps = Reg(init=Bits(0, width = io.inner.tlDataBeats))
  val pending_orel_data = Reg(init=Bits(0, width = io.inner.tlDataBeats))

  // These IOs are used for routing in the parent
  io.matches.iacq := (state =/= s_idle) && io.iacq().conflicts(xact_addr_block)
  io.matches.irel := (state =/= s_idle) && io.irel().conflicts(xact_addr_block)
  io.matches.oprb := (state =/= s_idle) && io.oprb().conflicts(xact_addr_block)

  // Start the writeback sub-transaction
  io.wb.req.ready := state === s_idle

  // Track which clients yet need to be probed and make Probe message
  pending_iprbs := pending_iprbs & dropPendingBitAtDest(io.inner.probe)
  val curr_probe_dst = PriorityEncoder(pending_iprbs)
  io.inner.probe.valid := state === s_inner_probe && pending_iprbs.orR
  io.inner.probe.bits := xact.coh.inner.makeProbeForVoluntaryWriteback(curr_probe_dst, xact_addr_block)

  // Handle incoming releases from clients, which may reduce sharer counts
  // and/or write back dirty data
  val irel_can_merge = io.irel().conflicts(xact_addr_block) &&
                         io.irel().isVoluntary() &&
                         !pending_vol_ignt

  val irel_same_xact = io.irel().conflicts(xact_addr_block) &&
                         !io.irel().isVoluntary() &&
                         state === s_inner_probe

  val irel_accepted = io.inner.release.fire() &&
                         (io.alloc.irel || irel_can_merge || irel_same_xact)

  io.inner.release.ready := irel_can_merge || irel_same_xact
  val pending_coh_on_irel = HierarchicalMetadata(
                              xact.coh.inner.onRelease(io.irel()), // Drop sharer
                              Mux(io.irel().hasData(), // Dirty writeback
                                xact.coh.outer.onHit(M_XWR),
                                xact.coh.outer))
  when(io.inner.release.fire()) {
    xact.coh := pending_coh_on_irel
    when(io.irel().hasData()) { data_buffer(io.irel().addr_beat) := io.irel().data }
    when(irel_can_merge) {
      xact_vol_ir_r_type := io.irel().r_type
      xact_vol_ir_src := io.irel().client_id
      xact_vol_ir_client_xact_id := io.irel().client_xact_id
    }
  }

  // If a release didn't write back data, have to read it from data array
  pending_reads := (pending_reads &
                     dropPendingBit(io.data.read) &
                     dropPendingBitWhenBeatHasData(io.inner.release))
  val curr_read_beat = PriorityEncoder(pending_reads)
  io.data.read.valid := state === s_busy && pending_reads.orR
  io.data.read.bits.id := UInt(trackerId)
  io.data.read.bits.way_en := xact.way_en
  io.data.read.bits.addr_idx := xact.idx
  io.data.read.bits.addr_beat := curr_read_beat
  io.data.write.valid := Bool(false)

  pending_resps := (pending_resps & dropPendingBitInternal(io.data.resp)) |
                     addPendingBitInternal(io.data.read)
  when(io.data.resp.valid) { 
    data_buffer(io.data.resp.bits.addr_beat) := io.data.resp.bits.data
  }

  // Once the data is buffered we can write it back to outer memory
  pending_orel_data := pending_orel_data |
                       addPendingBitWhenBeatHasData(io.inner.release) |
                       addPendingBitInternal(io.data.resp)
  io.outer.release.valid := state === s_busy &&
                            (!io.orel().hasData() || pending_orel_data(orel_data_idx))
  io.outer.release.bits := xact.coh.outer.makeVoluntaryWriteback(
                             client_xact_id = UInt(trackerId),
                             addr_block = xact_addr_block,
                             addr_beat = orel_data_idx,
                             data = data_buffer(orel_data_idx))

  // Ack a voluntary release if we got one
  io.inner.grant.valid := pending_vol_ignt
  io.inner.grant.bits := xact.coh.inner.makeGrant(xact_vol_irel)

  // Wait for an acknowledgement
  io.outer.grant.ready := state === s_outer_grant

  // Respond to the initiating transaction handler signalling completion of the writeback
  io.wb.resp.valid := state === s_wb_resp
  io.wb.resp.bits.id := xact.id

  // State machine updates and transaction handler metadata intialization
  when(state === s_idle && io.wb.req.valid) {
    xact := io.wb.req.bits
    val coh = io.wb.req.bits.coh
    val needs_inner_probes = coh.inner.requiresProbesOnVoluntaryWriteback()
    when(needs_inner_probes) { pending_iprbs := coh.inner.full() & ~io.incoherent.toBits }
    pending_reads := ~UInt(0, width = innerDataBeats)
    pending_resps := UInt(0)
    pending_orel_data := UInt(0)
    state := Mux(needs_inner_probes, s_inner_probe, s_busy)
  }
  when(state === s_inner_probe && !(pending_iprbs.orR || pending_irels || pending_vol_ignt)) {
    state := Mux(xact.coh.outer.requiresVoluntaryWriteback(), s_busy, s_wb_resp)
  }
  when(state === s_busy && orel_data_done) {
    state := Mux(io.orel().requiresAck(), s_outer_grant, s_wb_resp)
  }
  when(state === s_outer_grant && ognt_data_done) { state := s_wb_resp }
  when(state === s_wb_resp ) { state := s_idle }
}
