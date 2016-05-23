// See LICENSE for license details.

package uncore
import Chisel._
import cde.{Parameters, Field}

trait HasTrackerAllocationIO extends Bundle {
  val matches = new Bundle {
    val iacq = Bool(OUTPUT)
    val irel = Bool(OUTPUT)
    val oprb = Bool(OUTPUT)
  }
  val alloc = new Bundle {
    val iacq = Bool(INPUT)
    val irel = Bool(INPUT)
    val oprb = Bool(INPUT)
  }
}

class ManagerXactTrackerIO(implicit p: Parameters) extends ManagerTLIO()(p)
  with HasTrackerAllocationIO

class HierarchicalXactTrackerIO(implicit p: Parameters) extends HierarchicalTLIO()(p)
  with HasTrackerAllocationIO

abstract class XactTracker(implicit p: Parameters) extends CoherenceAgentModule()(p)
    with HasXactTrackerStates
    with HasPendingBitHelpers {
  override val s_idle :: s_meta_read :: s_meta_resp :: s_wb_req :: s_wb_resp :: s_inner_probe :: s_outer_acquire :: s_busy :: s_meta_write :: Nil = Enum(UInt(), 9)
  val state = Reg(init=s_idle)

  def quiesce(next: UInt = s_idle) {
    all_pending_done := !scoreboard.foldLeft(Bool(false))(_||_)
    when(state === s_busy && all_pending_done) { state := next }
  }

  def pinAllReadyValidLow[T <: Data](b: Bundle) {
    b.elements.foreach {
      _._2 match {
        case d: DecoupledIO[_] =>
          if(d.ready.dir == OUTPUT) d.ready := Bool(false)
          else if(d.valid.dir == OUTPUT) d.valid := Bool(false)
        case v: ValidIO[_] => if(v.valid.dir == OUTPUT) v.valid := Bool(false) 
        case b: Bundle => pinAllReadyValidLow(b)
        case _ =>
      }
    }
  }
}

trait HasXactTrackerStates {
  def state: UInt 
  def s_idle: UInt = UInt(0)
  def s_meta_read: UInt
  def s_meta_resp: UInt
  def s_wb_req: UInt
  def s_wb_resp: UInt
  def s_inner_probe: UInt
  def s_outer_acquire: UInt
  def s_busy: UInt
  def s_meta_write: UInt
}

trait HasPendingBitHelpers extends HasDataBeatCounters {
  val scoreboard = scala.collection.mutable.ListBuffer.empty[Bool]
  val all_pending_done = Wire(Bool())

  def addPendingBitWhenBeat[T <: HasBeat](inc: Bool, in: T): UInt =
    Fill(in.tlDataBeats, inc) &  UIntToOH(in.addr_beat)

  def dropPendingBitWhenBeat[T <: HasBeat](dec: Bool, in: T): UInt =
    ~Fill(in.tlDataBeats, dec) | ~UIntToOH(in.addr_beat)

  def addPendingBitWhenId[T <: HasClientId](inc: Bool, in: T): UInt =
    Fill(in.tlNCachingClients, inc) &  UIntToOH(in.client_id)

  def dropPendingBitWhenId[T <: HasClientId](dec: Bool, in: T): UInt =
    ~Fill(in.tlNCachingClients, dec) | ~UIntToOH(in.client_id)

  def addPendingBitWhenBeatHasData[T <: HasBeat](in: DecoupledIO[T], inc: Bool = Bool(true)): UInt =
    addPendingBitWhenBeat(in.fire() && in.bits.hasData() && inc, in.bits)

  def addPendingBitWhenBeatHasDataAndAllocs(in: DecoupledIO[AcquireFromSrc]): UInt =
    addPendingBitWhenBeatHasData(in, in.bits.allocate())

  def addPendingBitWhenBeatNeedsRead(in: DecoupledIO[AcquireFromSrc], inc: Bool = Bool(true)): UInt = {
    val a = in.bits
    val needs_read = (a.isGet() || a.isAtomic() || a.hasPartialWritemask()) || inc
    addPendingBitWhenBeat(in.fire() && needs_read, a)
  }

  def addPendingBitWhenBeatHasPartialWritemask(in: DecoupledIO[AcquireFromSrc]): UInt =
    addPendingBitWhenBeat(in.fire() && in.bits.hasPartialWritemask(), in.bits)

  def addPendingBitsFromAcquire(a: SecondaryMissInfo): UInt =
    Mux(a.hasMultibeatData(), Fill(a.tlDataBeats, UInt(1, 1)), UIntToOH(a.addr_beat))

  def dropPendingBitWhenBeatHasData[T <: HasBeat](in: DecoupledIO[T]): UInt =
    dropPendingBitWhenBeat(in.fire() && in.bits.hasData(), in.bits)

  def dropPendingBitAtDest[T <: HasId](in: DecoupledIO[T]): UInt =
    dropPendingBitWhenId(in.fire(), in.bits)

  def dropPendingBitAtDestWhenVoluntary[T <: HasId with MightBeVoluntary](in: DecoupledIO[T]): UInt =
    dropPendingBitWhenId(in.fire() && in.bits.isVoluntary(), in.bits)

  def addPendingBitAtSrc[T <: HasId](in: DecoupledIO[T]): UInt =
    addPendingBitWhenId(in.fire(), in.bits)

  def addPendingBitAtSrcWhenVoluntary[T <: HasId with MightBeVoluntary](in: DecoupledIO[T]): UInt =
    addPendingBitWhenId(in.fire() && in.bits.isVoluntary(), in.bits)

  def addOtherBits(en: Bool, nBits: Int): UInt =
    Mux(en, Cat(Fill(nBits - 1, UInt(1, 1)), UInt(0, 1)), UInt(0, nBits))

  def addPendingBitsOnFirstBeat(in: DecoupledIO[Acquire]): UInt =
    addOtherBits(in.fire() &&
                 in.bits.hasMultibeatData() &&
                 in.bits.addr_beat === UInt(0),
                 in.bits.tlDataBeats)

  def dropPendingBitsOnFirstBeat(in: DecoupledIO[Acquire]): UInt =
    ~addPendingBitsOnFirstBeat(in)
}

trait HasDataBuffer extends HasCoherenceAgentParameters {
  val data_buffer = Reg(init=Vec.fill(innerDataBeats)(UInt(0, width = innerDataBits)))

  type TLDataBundle = TLBundle with HasTileLinkData with HasTileLinkBeatId

  def initDataInner[T <: Acquire](in: DecoupledIO[T], init: Bool) {
    when(in.fire() && init && in.bits.hasData()) { 
      data_buffer(in.bits.addr_beat) := in.bits.data
    }
  }

  // TODO: provide func for accessing when innerDataBeats =/= outerDataBeats or internalDataBeats
  def mergeData(dataBits: Int)(beat: UInt, incoming: UInt) {
    data_buffer(beat) := incoming 
  }

  def mergeDataInner[T <: TLDataBundle](in: DecoupledIO[T]) {
    when(in.fire() && in.bits.hasData()) { 
      mergeData(innerDataBits)(in.bits.addr_beat, in.bits.data)
    }
  }

  def mergeDataOuter[T <: TLDataBundle](in: DecoupledIO[T]) {
    when(in.fire() && in.bits.hasData()) { 
      mergeData(outerDataBits)(in.bits.addr_beat, in.bits.data)
    }
  }
}

trait HasByteWriteMaskBuffer extends HasDataBuffer { 
  val wmask_buffer = Reg(init=Vec.fill(innerDataBeats)(UInt(0, width = innerWriteMaskBits)))

  override def initDataInner[T <: Acquire](in: DecoupledIO[T], init: Bool) {
    when(in.fire() && init && in.bits.hasData()) { 
      val beat = in.bits.addr_beat
      val full = FillInterleaved(8, in.bits.wmask())
      data_buffer(beat) := (~full & data_buffer(beat)) | (full & in.bits.data)
      wmask_buffer(beat) := in.bits.wmask() | wmask_buffer(beat) // assumes wmask_buffer is zeroed
    }
  }

  override def mergeData(dataBits: Int)(beat: UInt, incoming: UInt) {
    val old_data = incoming     // Refilled, written back, or de-cached data
    val new_data = data_buffer(beat) // Newly Put data is already in the buffer
    val wmask = FillInterleaved(8, wmask_buffer(beat))
    data_buffer(beat) := ~wmask & old_data | wmask & new_data
  }
}

trait HasBlockAddressBuffer extends HasCoherenceAgentParameters {
  val xact_addr_block = Reg(init = UInt(0, width = blockAddrBits))
}


trait HasAcquireMetadataBuffer extends HasBlockAddressBuffer {
  val xact_allocate = Reg{ Bool() }
  val xact_amo_shift_bytes =  Reg{ UInt() }
  val xact_op_code =  Reg{ UInt() }
  val xact_addr_byte =  Reg{ UInt() }
  val xact_op_size =  Reg{ UInt() }
  def xact_addr_beat: UInt
  def xact_iacq: SecondaryMissInfo
}

trait HasVoluntaryReleaseMetadataBuffer extends HasBlockAddressBuffer
    with HasPendingBitHelpers
    with HasXactTrackerStates {
  def io: HierarchicalXactTrackerIO

  lazy val xact_vol_ir_r_type = Reg{ io.irel().r_type }
  lazy val xact_vol_ir_src = Reg{ io.irel().client_id }
  lazy val xact_vol_ir_client_xact_id = Reg{ io.irel().client_xact_id }

  lazy val xact_vol_irel = Release(
                        src = xact_vol_ir_src,
                        voluntary = Bool(true),
                        r_type = xact_vol_ir_r_type,
                        client_xact_id = xact_vol_ir_client_xact_id,
                        addr_block = xact_addr_block)
                        (p.alterPartial({ case TLId => p(InnerTLId) }))
}

trait AcceptsVoluntaryReleases extends HasVoluntaryReleaseMetadataBuffer {
  def inner_coh: ManagerMetadata

  val pending_irel_data = Reg(init=Bits(0, width = innerDataBeats))

  lazy val pending_vol_ignt = {
    connectTwoWayBeatCounter(
      up = io.inner.release,
      down = io.inner.grant,
      trackUp = (r: Release) => {
        Mux(state === s_idle, io.alloc.irel, io.matches.irel) &&
        r.isVoluntary() &&
        r.requiresAck()
      },
      trackDown = (g: Grant) => (state =/= s_idle) && g.isVoluntary())._1
  }

  def irel_can_merge: Bool
  def irel_same_xact: Bool
  def irel_is_accepted: Bool = io.inner.release.fire() && (io.alloc.irel || irel_can_merge || irel_same_xact)
  def irel_is_allocating: Bool = state === s_idle && io.inner.release.valid && io.alloc.irel

  def innerRelease(block_vol_ignt: Bool = Bool(false), next: UInt = s_busy) {
    pending_irel_data := (pending_irel_data & dropPendingBitWhenBeatHasData(io.inner.release))

    when(irel_is_allocating) {
      xact_addr_block := io.irel().addr_block
      state := next
    }

    when(io.inner.release.fire()) {
      when(io.alloc.irel || (irel_can_merge && io.irel().first())) {
        xact_vol_ir_r_type := io.irel().r_type
        xact_vol_ir_src := io.irel().client_id
        xact_vol_ir_client_xact_id := io.irel().client_xact_id
        pending_irel_data := Mux(io.irel().hasMultibeatData(),
                                dropPendingBitWhenBeatHasData(io.inner.release),
                                UInt(0))
      }
    }

    io.inner.grant.valid := (state === s_busy || state === s_inner_probe) &&
                              pending_vol_ignt &&
                              !(pending_irel_data.orR || block_vol_ignt)

    io.inner.grant.bits := inner_coh.makeGrant(xact_vol_irel)

    scoreboard += (pending_irel_data.orR, pending_vol_ignt)
  }

}

trait EmitsVoluntaryReleases extends HasVoluntaryReleaseMetadataBuffer {
  val pending_orel = Reg(init=Bool(false))
  val pending_orel_data = Reg(init=Bits(0, width = innerDataBeats))

  lazy val (pending_vol_ognt,
            orel_data_idx,
            orel_data_done,
            vol_ognt_data_idx,
            vol_ognt_data_done) = {
    connectTwoWayBeatCounter(
      up = io.outer.release,
      down = io.outer.grant,
      trackUp = (r: Release) => r.isVoluntary() && r.requiresAck(),
      trackDown = (g: Grant) => g.isVoluntary())
  }

  def outerRelease(
      coh: ClientMetadata,
      data: UInt = io.irel().data,
      addPendingBit: UInt = UInt(0)) {
    pending_orel_data := (pending_orel_data & dropPendingBitWhenBeatHasData(io.outer.release)) |
                          addPendingBitWhenBeatHasData(io.inner.release) |
                          addPendingBit
    io.outer.release.valid := state === s_busy &&
                                ((!io.orel().hasData() && pending_orel) || pending_orel_data(orel_data_idx))
    io.outer.release.bits := coh.makeVoluntaryWriteback(
      client_xact_id = UInt(0), // TODO was tracker id, but not needed?
      addr_block = xact_addr_block,
      addr_beat = orel_data_idx,
      data = data)

    when(pending_orel_data.orR) { pending_orel := Bool(true) }
    when(orel_data_done) { pending_orel := Bool(false) }

    io.outer.grant.ready := state === s_busy

    scoreboard += (pending_orel, pending_vol_ognt)
  }
}

trait EmitsInnerProbes extends HasBlockAddressBuffer
    with HasXactTrackerStates 
    with HasPendingBitHelpers {
  def io: HierarchicalXactTrackerIO

  val pending_iprbs = Reg(UInt(width = innerNCachingClients))
  val curr_probe_dst = PriorityEncoder(pending_iprbs)

  lazy val pending_irels = {
    connectTwoWayBeatCounter(
      up = io.inner.probe,
      down = io.inner.release,
      max = innerNCachingClients,
      trackDown = (r: Release) => (state =/= s_idle) && !r.isVoluntary())._1
  }

  def full_representation: UInt
  def initializeProbes() { pending_iprbs := full_representation & ~io.incoherent.toBits }
  def irel_same_xact = io.irel().conflicts(xact_addr_block) &&
                         !io.irel().isVoluntary() &&
                         state === s_inner_probe 

  def innerProbe(prb: Probe, next: UInt) {
    pending_iprbs := pending_iprbs & dropPendingBitAtDest(io.inner.probe)
    io.inner.probe.valid := state === s_inner_probe && pending_iprbs.orR
    io.inner.probe.bits := prb
    
    when(state === s_inner_probe && !(pending_iprbs.orR || pending_irels)) {
      state := next
    }

    //N.B. no pending bits added to scoreboard because all handled in s_inner_probe
  }
}

trait RoutesInParent extends HasBlockAddressBuffer
    with HasXactTrackerStates {
  def io: HierarchicalXactTrackerIO
  type AddrComparison = HasCacheBlockAddress => Bool
  def exactAddrMatch(a: HasCacheBlockAddress): Bool = a.conflicts(xact_addr_block)
  def routeInParent(iacqMatches: AddrComparison = exactAddrMatch,
            irelMatches: AddrComparison = exactAddrMatch,
            oprbMatches: AddrComparison = exactAddrMatch) {
    io.matches.iacq := (state =/= s_idle) && iacqMatches(io.iacq())
    io.matches.irel := (state =/= s_idle) && irelMatches(io.irel())
    io.matches.oprb := (state =/= s_idle) && oprbMatches(io.oprb())
  }
}

trait AcceptsInnerAcquires extends HasAcquireMetadataBuffer
    with AcceptsVoluntaryReleases
    with HasXactTrackerStates
    with HasPendingBitHelpers {
  def io: HierarchicalXactTrackerIO
  def nSecondaryMisses: Int
  def alwaysWriteFullBeat: Boolean
  def inner_coh: ManagerMetadata
  def trackerId: Int

  // Miss queue holds transaction metadata used to make grants
  lazy val ignt_q = Module(new Queue(
        new SecondaryMissInfo()(p.alterPartial({ case TLId => p(InnerTLId) })),
        1 + nSecondaryMisses))
  lazy val xact_iacq = ignt_q.io.deq.bits
  lazy val xact_addr_beat = ignt_q.io.deq.bits.addr_beat
  lazy val pending_ignt = ignt_q.io.count > UInt(0)

  lazy val (ignt_data_idx, ignt_data_done) =
    connectOutgoingDataBeatCounter(
      out = io.inner.grant,
      beat = ignt_q.io.deq.bits.addr_beat)

  lazy val pending_ifins =
    connectTwoWayBeatCounter(
      up = io.inner.grant,
      down = io.inner.finish,
      max = nSecondaryMisses,
      trackUp = (g: Grant) => g.requiresAck())._1

  lazy val pending_put_data = Reg(init=Bits(0, width = innerDataBeats))
  lazy val pending_ignt_data = Reg(init=Bits(0, width = innerDataBeats))

  def iacq_same_xact: Bool = {
    (xact_iacq.client_xact_id === io.iacq().client_xact_id) &&
      xact_iacq.hasMultibeatData() &&
      pending_ignt &&
      pending_put_data(io.iacq().addr_beat)
  }
  def iacq_can_merge: Bool
  def iacq_is_accepted: Bool = io.inner.acquire.fire() && (io.alloc.iacq || iacq_can_merge || iacq_same_xact)
  def iacq_is_allocating: Bool = state === s_idle && io.inner.acquire.valid && io.alloc.iacq

  def innerAcquire(can_alloc: Bool, next: UInt) {

    // Enqueue some metadata information that we'll use to make coherence updates with later
    ignt_q.io.enq.valid := iacq_is_accepted && io.iacq().first()
    ignt_q.io.enq.bits := io.iacq()

    // Track whether any beats are missing from a PutBlock
    pending_put_data := (pending_put_data &
        dropPendingBitWhenBeatHasData(io.inner.acquire)) |
        addPendingBitsOnFirstBeat(io.inner.acquire)

    // Intialize transaction metadata for accepted Acquire
    when(iacq_is_allocating) {
      xact_addr_block := io.iacq().addr_block
      xact_allocate := io.iacq().allocate() && can_alloc
      xact_amo_shift_bytes := io.iacq().amo_shift_bytes()
      xact_op_code := io.iacq().op_code()
      xact_addr_byte := io.iacq().addr_byte()
      xact_op_size := io.iacq().op_size()
      // Make sure to collect all data from a PutBlock
      pending_put_data := Mux(
        io.iacq().isBuiltInType(Acquire.putBlockType),
        dropPendingBitWhenBeatHasData(io.inner.acquire),
        UInt(0))
      pending_ignt_data := UInt(0)
      state := next
    }

    scoreboard += (pending_put_data.orR)
  }

  def innerGrant(
      ignt_data: UInt = io.ognt().data,
      ignt_pending: Bool = Bool(false),
      add: UInt = UInt(0)) {
    // Track which beats are ready for response
    pending_ignt_data := (pending_ignt_data & dropPendingBitWhenBeatHasData(io.inner.grant)) |
                         addPendingBitWhenBeatHasData(io.inner.release) |
                         addPendingBitWhenBeatHasData(io.outer.grant) |
                         add

    // We can issue a grant for a pending write once all data is
    // received and committed to the data array or outer memory
    val ignt_ack_ready = !(state === s_idle ||
                           state === s_meta_read ||
                           pending_put_data.orR)

    val ignt_from_iacq = inner_coh.makeGrant(
                            sec = ignt_q.io.deq.bits,
                            manager_xact_id = UInt(trackerId), 
                            data = ignt_data)

    // Make the Grant message using the data stored in the secondary miss queue
    ignt_q.io.deq.ready := Bool(false)
    when(!pending_vol_ignt) {
      ignt_q.io.deq.ready := ignt_data_done
      io.inner.grant.bits := ignt_from_iacq
      io.inner.grant.bits.addr_beat := ignt_data_idx // override based on outgoing counter
      when (state === s_busy && pending_ignt) {
        io.inner.grant.valid := !ignt_pending && 
                                  Mux(io.ignt().hasData(), pending_ignt_data(ignt_data_idx), ignt_ack_ready)
      }
    }

    // We must wait for as many Finishes as we sent Grants
    io.inner.finish.ready := state === s_busy

    scoreboard += (pending_ignt, pending_ifins)
  }

}

trait EmitsOuterAcquires extends AcceptsInnerAcquires {
  lazy val (pending_ognt,
            oacq_data_idx,
            oacq_data_done,
            ognt_data_idx,
            ognt_data_done) =
    connectTwoWayBeatCounter(
      up = io.outer.acquire,
      down = io.outer.grant,
      beat = xact_addr_beat,
      trackDown = (g: Grant) => !g.isVoluntary())

  // Handle misses or coherence permission upgrades by initiating a new transaction in the outer memory:
  //
  // If we're allocating in this cache, we can use the current metadata
  // to make an appropriate custom Acquire, otherwise we copy over the
  // built-in Acquire from the inner TL to the outer TL
  def outerAcquire(
      caching: Bool,
      coh: ClientMetadata,
      data: UInt = io.iacq().data,
      wmask: UInt = io.iacq().wmask(),
      next: UInt = s_busy) {
    io.outer.acquire.valid := state === s_outer_acquire &&
                              (xact_allocate || !pending_put_data(oacq_data_idx))
    io.outer.acquire.bits :=
      Mux(
        caching,
        coh.makeAcquire(
          op_code = xact_op_code,
          client_xact_id = UInt(0),
          addr_block = xact_addr_block),
        BuiltInAcquireBuilder(
          a_type = xact_iacq.a_type,
          client_xact_id = UInt(0),
          addr_block = xact_addr_block,
          addr_beat = oacq_data_idx,
          data = data,
          addr_byte = xact_addr_byte,
          operand_size = xact_op_size,
          opcode = xact_op_code,
          wmask = wmask,
          alloc = Bool(false))
          (p.alterPartial({ case TLId => p(OuterTLId)})))

    when(state === s_outer_acquire && oacq_data_done) { state := next }

    io.outer.grant.ready := state === s_busy

    scoreboard += pending_ognt
  }
}

abstract class VoluntaryReleaseTracker(val trackerId: Int)(implicit p: Parameters) extends XactTracker()(p)
    with AcceptsVoluntaryReleases
    with RoutesInParent {
  def irel_can_merge = Bool(false)
  def irel_same_xact = io.irel().conflicts(xact_addr_block) &&
                          io.irel().isVoluntary() &&
                          pending_irel_data.orR
}

abstract class AcquireTracker(val trackerId: Int)(implicit p: Parameters) extends XactTracker()(p)
    with AcceptsInnerAcquires
    with EmitsOuterAcquires
    with EmitsInnerProbes
    with RoutesInParent {
}
