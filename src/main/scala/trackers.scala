// See LICENSE for license details.

package uncore
import Chisel._
import cde.{Parameters, Field}

trait HasDataBuffer extends HasCoherenceAgentParameters {
  val data_buffer = Reg(init=Vec.fill(innerDataBeats)(UInt(0, width = innerDataBits)))

  // TODO: provide func for accessing when innerDataBeats =/= outerDataBeats or internalDataBeats
  def mergeData(dataBits: Int)(beat: UInt, incoming: UInt) {
    data_buffer(beat) := incoming 
  }

  def mergeDataInner[T <: TLBundle with HasTileLinkData with HasTileLinkBeatId](in: DecoupledIO[T]) {
    when(in.fire() && in.bits.hasData()) { 
      mergeData(innerDataBits)(in.bits.addr_beat, in.bits.data)
    }
  }

  def mergeDataOuter[T <: TLBundle with HasTileLinkData with HasTileLinkBeatId](in: DecoupledIO[T]) {
    when(in.fire() && in.bits.hasData()) { 
      mergeData(outerDataBits)(in.bits.addr_beat, in.bits.data)
    }
  }
}

trait HasByteWriteMaskBuffer extends HasDataBuffer { 
  val wmask_buffer = Reg(init=Vec.fill(innerDataBeats)(UInt(0, width = innerWriteMaskBits)))

  override def mergeData(dataBits: Int)(beat: UInt, incoming: UInt) {
    val old_data = incoming     // Refilled, written back, or de-cached data
    val new_data = data_buffer(beat) // Newly Put data is already in the buffer
    val wmask = FillInterleaved(8, wmask_buffer(beat))
    data_buffer(beat) := ~wmask & old_data | wmask & new_data
  }
}

trait HasBlockAddressBuffer extends HasCoherenceAgentParameters {
  val s_idle :: s_meta_read :: s_meta_resp :: s_wb_req :: s_wb_resp :: s_inner_probe :: s_outer_acquire :: s_busy :: s_meta_write :: Nil = Enum(UInt(), 9)
  val state = Reg(init=s_idle)

  val xact_addr_block = Reg(init = UInt(0, width = blockAddrBits))
}


trait HasAcquireMetadataBuffer extends HasBlockAddressBuffer {
  lazy val xact_allocate = Reg{ Bool() }
  lazy val xact_amo_shift_bytes =  Reg{ UInt() }
  lazy val xact_op_code =  Reg{ UInt() }
  lazy val xact_addr_byte =  Reg{ UInt() }
  lazy val xact_op_size =  Reg{ UInt() }
  val xact_addr_beat: UInt
  val xact_iacq: SecondaryMissInfo
}

trait HasReleaseMetadataBuffer extends HasBlockAddressBuffer {
  val io: HierarchicalXactTrackerIO

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

trait TriggersInnerProbes extends HasBlockAddressBuffer
    with HasPendingBits {
  val io: HierarchicalXactTrackerIO

  lazy val pending_iprbs = Reg(UInt(width = innerNCachingClients))
  lazy val curr_probe_dst = PriorityEncoder(pending_iprbs)

  lazy val pending_irels =
    connectTwoWayBeatCounter(
      max = innerNCachingClients,
      up = io.inner.probe,
      down = io.inner.release,
      trackDown = (r: Release) => !r.isVoluntary())._1

  def initializeProbes(full_sharers: UInt, client_id: UInt, self_probe: Bool) {
    val mask_self = Mux(self_probe,
                        full_sharers | UIntToOH(client_id),
                        full_sharers & ~UIntToOH(client_id))
    val mask_incoherent = mask_self & ~io.incoherent.toBits
    pending_iprbs := mask_incoherent
  }

  def innerProbe(prb: Probe, next_state: UInt) {
    pending_iprbs := pending_iprbs & dropPendingBitAtDest(io.inner.probe)
    io.inner.probe.valid := state === s_inner_probe && pending_iprbs.orR
    io.inner.probe.bits := prb
    
    when(state === s_inner_probe && !(pending_iprbs.orR || pending_irels)) {
      state := next_state
    }
  }
}

  
abstract class VoluntaryReleaseTracker(val trackerId: Int)(implicit p: Parameters) extends XactTracker()(p)
    with HasReleaseMetadataBuffer
    with HasDataBuffer {
  val io: HierarchicalXactTrackerIO
  val inner_coh: ManagerMetadata

  lazy val pending_irel_beats = Reg(init=Bits(0, width = io.inner.tlDataBeats))
  lazy val pending_writes = Reg(init=Bits(0, width = io.inner.tlDataBeats))
  lazy val pending_ignt = Reg(init=Bool(false))
  lazy val all_pending_done =
    !(pending_writes.orR ||
      pending_irel_beats.orR ||
      pending_ignt)

  lazy val curr_write_beat = PriorityEncoder(pending_writes)

  // Accept a voluntary Release (and any further beats of data)
  def accept(next_state: UInt) {
    pending_irel_beats := (pending_irel_beats & dropPendingBitWhenBeatHasData(io.inner.release))
      io.inner.release.ready := ((state === s_idle) && io.irel().isVoluntary()) || pending_irel_beats.orR
    when(io.inner.release.fire()) { data_buffer(io.irel().addr_beat) := io.irel().data }

    when(state === s_idle && io.inner.release.valid && io.alloc.irel) {
      xact_addr_block := io.irel().addr_block
      xact_vol_ir_r_type := io.irel().r_type
      xact_vol_ir_src := io.irel().client_id
      xact_vol_ir_client_xact_id := io.irel().client_xact_id
      pending_irel_beats := Mux(io.irel().hasMultibeatData(),
                              dropPendingBitWhenBeatHasData(io.inner.release),
                              UInt(0))
      pending_writes := addPendingBitWhenBeatHasData(io.inner.release)
      pending_ignt := io.irel().requiresAck()
      state := next_state
    }
  }

  // These IOs are used for routing in the parent
  def route(iacqMatches: Bool) {
    io.matches.iacq := (state =/= s_idle) && iacqMatches
    io.matches.irel := (state =/= s_idle) && io.irel().conflicts(xact_vol_irel)
    io.matches.oprb := (state =/= s_idle) && io.oprb().conflicts(xact_vol_irel)
  }

   // Write the voluntarily written back data
  def write[T <: Data](port: DecoupledIO[T], packet: T, drop: UInt) {
    pending_writes := (pending_writes & drop) |
                        addPendingBitWhenBeatHasData(io.inner.release)
    port.valid := state === s_busy && pending_writes.orR
    port.bits := packet
  }

  // Send an acknowledgement
  def acknowledge(committed: Bool, inner_ready: Bool) {
    io.inner.grant.valid := state === s_busy && pending_ignt && !pending_irel_beats.orR && committed
    io.inner.grant.bits := inner_coh.makeGrant(xact_vol_irel)
    when(io.inner.grant.fire()) { pending_ignt := Bool(false) }
    io.outer.grant.ready := state === s_busy && inner_ready
  }
}

abstract class AcquireTracker(val trackerId: Int)(implicit p: Parameters) extends XactTracker()(p)
    with HasAcquireMetadataBuffer
    with HasReleaseMetadataBuffer
    with HasByteWriteMaskBuffer
    with TriggersInnerProbes {
  val io: HierarchicalXactTrackerIO
  val inner_coh: ManagerMetadata
  val nSecondaryMisses: Int
  val alwaysWriteFullBeat: Boolean

  // Miss queue holds transaction metadata used to make grants
  lazy val ignt_q = Module(new Queue(
        new SecondaryMissInfo()(p.alterPartial({ case TLId => p(InnerTLId) })),
        1 + nSecondaryMisses))
  lazy val xact_iacq = ignt_q.io.deq.bits
  lazy val xact_addr_beat = ignt_q.io.deq.bits.addr_beat

  // Counters and scoreboard tracking progress made on processing this transaction
  lazy val pending_vol_ignt =
    connectTwoWayBeatCounter(
      max = 1,
      up = io.inner.release,
      down = io.inner.grant,
      trackUp = (r: Release) => r.isVoluntary(),
      trackDown = (g: Grant) => g.isVoluntary())._1

  lazy val (pending_ognt,
            oacq_data_idx,
            oacq_data_done,
            ognt_data_idx,
            ognt_data_done) =
    connectTwoWayBeatCounter(
      max = 1,
      up = io.outer.acquire,
      down = io.outer.grant,
      beat = xact_addr_beat)

  lazy val (ignt_data_idx, ignt_data_done) =
    connectOutgoingDataBeatCounter(
      out = io.inner.grant,
      beat = ignt_q.io.deq.bits.addr_beat)

  lazy val pending_ifins =
    connectTwoWayBeatCounter(
      max = nSecondaryMisses,
      up = io.inner.grant,
      down = io.inner.finish,
      trackUp = (g: Grant) => g.requiresAck())._1

  lazy val pending_puts = Reg(init=Bits(0, width = io.inner.tlDataBeats))
  lazy val pending_reads = Reg(init=Bits(0, width = io.inner.tlDataBeats))
  lazy val pending_irel_beats = Reg(init=Bits(0, width = io.inner.tlDataBeats))
  lazy val pending_writes = Reg(init=Bits(0, width = io.inner.tlDataBeats))
  lazy val pending_resps = Reg(init=Bits(0, width = io.inner.tlDataBeats))
  lazy val ignt_data_ready = Reg(init=Bits(0, width = io.inner.tlDataBeats))

  lazy val curr_read_beat = PriorityEncoder(pending_reads)
  lazy val curr_write_beat = PriorityEncoder(pending_writes)

  // Used to decide when to escape from s_busy
  lazy val all_pending_done =
    !(pending_reads.orR ||
      pending_writes.orR ||
      pending_resps.orR ||
      pending_puts.orR ||
      pending_irel_beats.orR ||
      pending_ognt ||
      ignt_q.io.count > UInt(0) ||
      pending_vol_ignt ||
      //pending_meta_write || // Has own state: s_meta_write
      pending_ifins)

  // These IOs are used for routing in the parent
  def route(iacqMatches: Bool, irelMatches: Bool) {
    io.matches.iacq := (state =/= s_idle) && iacqMatches
    io.matches.irel := (state =/= s_idle) && irelMatches
    io.matches.oprb := (state =/= s_idle) && io.oprb().conflicts(xact_addr_block)
  }

  def accept(iacq_can_merge: Bool, can_alloc: Bool, next_state: UInt) {
    val iacq_same_xact = 
      xact_iacq.client_xact_id === io.iacq().client_xact_id &&
      xact_iacq.hasMultibeatData() &&
      ignt_q.io.deq.valid && // i.e. state =/= s_idle
      pending_puts(io.iacq().addr_beat)

    val iacq_accepted = io.inner.acquire.fire() &&
                          (io.alloc.iacq || iacq_can_merge || iacq_same_xact)

    io.inner.acquire.ready := state === s_idle || iacq_can_merge || iacq_same_xact 

    // Handling of primary and secondary misses' data and write mask merging
    when(iacq_accepted && io.iacq().hasData()) {
      val beat = io.iacq().addr_beat
      val full = FillInterleaved(8, io.iacq().wmask())
      data_buffer(beat) := (~full & data_buffer(beat)) | (full & io.iacq().data)
      wmask_buffer(beat) := io.iacq().wmask() | wmask_buffer(beat) // assumes wmask_buffer is zeroed
    }

    // Enqueue some metadata information that we'll use to make coherence updates with later
    ignt_q.io.enq.valid := iacq_accepted && io.iacq().first()
    ignt_q.io.enq.bits := io.iacq()

    // Track whether any beats are missing from a PutBlock
    pending_puts := (pending_puts &
        dropPendingBitWhenBeatHasData(io.inner.acquire)) |
        addPendingBitsOnFirstBeat(io.inner.acquire)

    // Intialize transaction metadata for accepted Acquire
    when(state === s_idle && io.inner.acquire.valid && io.alloc.iacq) {
      xact_addr_block := io.iacq().addr_block
      xact_allocate := io.iacq().allocate() && can_alloc
      xact_amo_shift_bytes := io.iacq().amo_shift_bytes()
      xact_op_code := io.iacq().op_code()
      xact_addr_byte := io.iacq().addr_byte()
      xact_op_size := io.iacq().op_size()
      // Make sure to collect all data from a PutBlock
      pending_puts := Mux(
        io.iacq().isBuiltInType(Acquire.putBlockType),
        dropPendingBitWhenBeatHasData(io.inner.acquire),
        UInt(0))
      // Pick out the specific beats of data that need to be read
      pending_reads := Mux(
        io.iacq().isBuiltInType(Acquire.getBlockType) || !io.iacq().isBuiltInType(),
        ~UInt(0, width = innerDataBeats),
        addPendingBitWhenBeatNeedsRead(io.inner.acquire, Bool(alwaysWriteFullBeat)))
      pending_writes := addPendingBitWhenBeatHasDataAndAllocs(io.inner.acquire)
      pending_resps := UInt(0)
      ignt_data_ready := UInt(0)
      state := next_state
    }
  }

  def innerRelease(irel_can_merge: Bool) {
    val irel_same_xact = io.irel().conflicts(xact_addr_block) &&
                           !io.irel().isVoluntary() &&
                           state === s_inner_probe 

    val irel_accepted = io.inner.release.fire() &&
                           (io.alloc.irel || irel_can_merge || irel_same_xact)

    io.inner.release.ready := irel_can_merge || irel_same_xact

    when(io.inner.release.fire() && irel_can_merge) {
      xact_vol_ir_r_type := io.irel().r_type
      xact_vol_ir_src := io.irel().client_id
      xact_vol_ir_client_xact_id := io.irel().client_xact_id
      pending_irel_beats := Mux(io.irel().hasMultibeatData(),
                              dropPendingBitWhenBeatHasData(io.inner.release),
                              UInt(0))
    }
    pending_irel_beats := (pending_irel_beats & dropPendingBitWhenBeatHasData(io.inner.release))
  }

  // Handle misses or coherence permission upgrades by initiating a new transaction in the outer memory:
  //
  // If we're allocating in this cache, we can use the current metadata
  // to make an appropriate custom Acquire, otherwise we copy over the
  // built-in Acquire from the inner TL to the outer TL

  def outerAcquire(acq: Acquire, next_state: UInt) {
    io.outer.acquire.valid := state === s_outer_acquire &&
                              (xact_allocate || !pending_puts(oacq_data_idx))
    io.outer.acquire.bits := acq

    when(state === s_outer_acquire && oacq_data_done) { state := next_state }
  }

  // Going back to the original inner transaction:
  // We read from the the cache at this level if data wasn't written back or refilled.
  // We may still merge further Gets, requiring further beats to be read.
  // If ECC requires a full writemask, we'll read out data on partial writes as well.
  def read[T <: Data](port: DecoupledIO[T], packet: T, drop: UInt) {
    pending_reads := (pending_reads &
                         drop &
                         dropPendingBitWhenBeatHasData(io.inner.release) &
                         dropPendingBitWhenBeatHasData(io.outer.grant)) |
                       addPendingBitWhenBeatNeedsRead(io.inner.acquire, Bool(alwaysWriteFullBeat))
    port.valid := state === s_busy && pending_reads.orR && !pending_ognt
    port.bits := packet
  }

  // We write data to the cache at this level if it was Put here with allocate flag,
  // written back dirty, or refilled from outer memory.
  def write[T <: Data](port: DecoupledIO[T], packet: T, drop: UInt) {
    pending_writes := (pending_writes &
                        drop &
                        dropPendingBitsOnFirstBeat(io.inner.acquire)) |
                        addPendingBitWhenBeatHasDataAndAllocs(io.inner.acquire) |
                        addPendingBitWhenBeatHasData(io.inner.release) |
                        addPendingBitWhenBeatHasData(io.outer.grant, xact_allocate)
    port.valid := state === s_busy &&
                             pending_writes.orR &&
                             !pending_ognt &&
                             !pending_reads(curr_write_beat) &&
                             !pending_resps(curr_write_beat)
    port.bits := packet
  }

  // soon as the data is released, granted, put, or read from the cache
  def innerGrant(ignt_from_iacq: Grant, add: UInt) {
    ignt_data_ready := ignt_data_ready |
                         addPendingBitWhenBeatHasData(io.inner.release) |
                         addPendingBitWhenBeatHasData(io.outer.grant) |
                         add
    // We can issue a grant for a pending write once all data is
    // received and committed to the data array or outer memory
    val ignt_ack_ready = !(state === s_idle ||
                           state === s_meta_read ||
                           pending_puts.orR ||
                           pending_writes.orR ||
                           pending_ognt)

    ignt_q.io.deq.ready := !pending_vol_ignt && ignt_data_done
    io.inner.grant.valid := pending_vol_ignt ||
                              (state === s_busy &&
                              ignt_q.io.deq.valid &&
                              Mux(io.ignt().hasData(),
                                ignt_data_ready(ignt_data_idx),
                                ignt_ack_ready))
    val ignt_from_irel = inner_coh.makeGrant(xact_vol_irel)
    // Make the Grant message using the data stored in the secondary miss queue
    io.inner.grant.bits := Mux(pending_vol_ignt, ignt_from_irel, ignt_from_iacq)
    io.inner.grant.bits.addr_beat := ignt_data_idx // override based on outgoing counter
  }
}
