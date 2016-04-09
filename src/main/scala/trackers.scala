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
  val xact_allocate = Reg{ Bool() }
  val xact_amo_shift_bytes =  Reg{ UInt() }
  val xact_op_code =  Reg{ UInt() }
  val xact_addr_byte =  Reg{ UInt() }
  val xact_op_size =  Reg{ UInt() }
  def xact_addr_beat: UInt
  def xact_iacq: SecondaryMissInfo
}

trait HasVoluntaryReleaseMetadataBuffer extends HasBlockAddressBuffer with HasPendingBits {
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

  def innerRelease(irel_can_merge: Bool) {
    val irel_same_xact = io.irel().conflicts(xact_addr_block) &&
                           !io.irel().isVoluntary() &&
                           state === s_inner_probe 

    io.inner.release.ready := irel_can_merge || irel_same_xact

    when(io.inner.release.fire() && !irel_same_xact) {
      xact_vol_ir_r_type := io.irel().r_type
      xact_vol_ir_src := io.irel().client_id
      xact_vol_ir_client_xact_id := io.irel().client_xact_id
      pending_irel_data := Mux(io.irel().hasMultibeatData(),
                              dropPendingBitWhenBeatHasData(io.inner.release),
                              UInt(0))
    }
    pending_irel_data := (pending_irel_data & dropPendingBitWhenBeatHasData(io.inner.release))
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

  def outerRelease(rel: Release, addPendingBit: UInt) {
    pending_orel_data := (pending_orel_data & dropPendingBitWhenBeatHasData(io.outer.release)) |
                          addPendingBitWhenBeatHasData(io.inner.release) |
                          addPendingBit
    io.outer.release.valid := state === s_busy &&
                                ((!rel.hasData() && pending_orel) || pending_orel_data(orel_data_idx))
    io.outer.release.bits := rel
    when(orel_data_done) { pending_orel := Bool(false) }
  }
}

trait TriggersInnerProbes extends HasBlockAddressBuffer with HasPendingBits {
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

trait DoesDataReads extends HasCoherenceAgentParameters {
  val pending_reads = Reg(init=Bits(0, width = innerDataBeats))
  val pending_resps = Reg(init=Bits(0, width = innerDataBeats))
  val curr_read_beat = PriorityEncoder(pending_reads)

  def readData[T <: Data](port: DecoupledIO[T], packet: T, drop: DecoupledIO[T] => UInt): Unit
}

trait DoesDataWrites extends HasCoherenceAgentParameters {
  val pending_writes = Reg(init=Bits(0, width = innerDataBeats))
  val curr_write_beat = PriorityEncoder(pending_writes)

  def writeData[T <: Data](port: DecoupledIO[T], packet: T, drop: DecoupledIO[T] => UInt): Unit
}

trait RoutesInParent extends HasBlockAddressBuffer {
  def io: HierarchicalXactTrackerIO
  type AddrComparison = HasCacheBlockAddress => Bool
  def exactAddrMatch(a: HasCacheBlockAddress): Bool = a.conflicts(xact_addr_block)
  def route(iacqMatches: AddrComparison = exactAddrMatch,
            irelMatches: AddrComparison = exactAddrMatch,
            oprbMatches: AddrComparison = exactAddrMatch) {
    io.matches.iacq := (state =/= s_idle) && iacqMatches(io.iacq())
    io.matches.irel := (state =/= s_idle) && irelMatches(io.irel())
    io.matches.oprb := (state =/= s_idle) && oprbMatches(io.oprb())
  }
}

abstract class VoluntaryReleaseTracker(val trackerId: Int)(implicit p: Parameters) extends XactTracker()(p)
    with AcceptsVoluntaryReleases
    with EmitsVoluntaryReleases
    with HasDataBuffer
    with DoesDataWrites
    with RoutesInParent {
  def io: HierarchicalXactTrackerIO
  def inner_coh: ManagerMetadata

  lazy val all_pending_done =
    !(pending_writes.orR ||
      pending_irel_data.orR ||
      pending_vol_ignt ||
      pending_vol_ognt) 

  // Accept a voluntary Release (and any further beats of data)
  def accept(next_state: UInt) {
    pending_irel_data := (pending_irel_data & dropPendingBitWhenBeatHasData(io.inner.release))
    io.inner.release.ready := ((state === s_idle) && io.irel().isVoluntary()) || pending_irel_data.orR
    when(io.inner.release.fire()) { data_buffer(io.irel().addr_beat) := io.irel().data }

    when(state === s_idle && io.inner.release.valid && io.alloc.irel) {
      xact_addr_block := io.irel().addr_block
      xact_vol_ir_r_type := io.irel().r_type
      xact_vol_ir_src := io.irel().client_id
      xact_vol_ir_client_xact_id := io.irel().client_xact_id
      pending_irel_data := Mux(io.irel().hasMultibeatData(),
                              dropPendingBitWhenBeatHasData(io.inner.release),
                              UInt(0))
      pending_writes := addPendingBitWhenBeatHasData(io.inner.release)
      state := next_state
    }
  }

   // Write the voluntarily written back data
  def writeData[T <: Data](port: DecoupledIO[T], packet: T, drop: DecoupledIO[T] => UInt) {
    pending_writes := (pending_writes & drop(port)) |
                        addPendingBitWhenBeatHasData(io.inner.release)
    port.valid := state === s_busy && pending_writes.orR
    port.bits := packet
  }

  // Send an acknowledgement
  def acknowledge(committed: Bool, inner_ready: Bool) {
    io.inner.grant.valid := state === s_busy && pending_vol_ignt && !pending_irel_data.orR && committed
    io.inner.grant.bits := inner_coh.makeGrant(xact_vol_irel)
    io.outer.grant.ready := state === s_busy && inner_ready
  }
}

abstract class AcquireTracker(val trackerId: Int)(implicit p: Parameters) extends XactTracker()(p)
    with HasAcquireMetadataBuffer
    with AcceptsVoluntaryReleases
    with HasByteWriteMaskBuffer
    with TriggersInnerProbes
    with DoesDataReads 
    with DoesDataWrites
    with RoutesInParent {
  def io: HierarchicalXactTrackerIO
  def nSecondaryMisses: Int
  def alwaysWriteFullBeat: Boolean
  def inner_coh: ManagerMetadata

  // Miss queue holds transaction metadata used to make grants
  lazy val ignt_q = Module(new Queue(
        new SecondaryMissInfo()(p.alterPartial({ case TLId => p(InnerTLId) })),
        1 + nSecondaryMisses))
  lazy val xact_iacq = ignt_q.io.deq.bits
  lazy val xact_addr_beat = ignt_q.io.deq.bits.addr_beat
  lazy val pending_ignt = ignt_q.io.count > UInt(0)

  // Counters and scoreboard tracking progress made on processing this transaction

  lazy val (pending_ognt,
        oacq_data_idx,
        oacq_data_done,
        ognt_data_idx,
        ognt_data_done) =
    connectTwoWayBeatCounter(
      up = io.outer.acquire,
      down = io.outer.grant,
      beat = xact_addr_beat)

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

  val pending_put_data = Reg(init=Bits(0, width = innerDataBeats))
  val ignt_data_ready = Reg(init=Bits(0, width = innerDataBeats))

  // Used to decide when to escape from s_busy
  lazy val all_pending_done =
    !(pending_reads.orR ||
      pending_writes.orR ||
      pending_resps.orR ||
      pending_put_data.orR ||
      pending_irel_data.orR ||
      pending_ognt ||
      pending_ignt ||
      pending_vol_ignt ||
      pending_ifins)
      //pending_meta_write has own state: s_meta_write


  def accept(iacq_can_merge: Bool, can_alloc: Bool, next_state: UInt) {
    val iacq_same_xact = 
      xact_iacq.client_xact_id === io.iacq().client_xact_id &&
      xact_iacq.hasMultibeatData() &&
      ignt_q.io.deq.valid && // i.e. state =/= s_idle
      pending_put_data(io.iacq().addr_beat)

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
    pending_put_data := (pending_put_data &
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
      pending_put_data := Mux(
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

  // Handle misses or coherence permission upgrades by initiating a new transaction in the outer memory:
  //
  // If we're allocating in this cache, we can use the current metadata
  // to make an appropriate custom Acquire, otherwise we copy over the
  // built-in Acquire from the inner TL to the outer TL

  def outerAcquire(acq: Acquire, next_state: UInt) {
    io.outer.acquire.valid := state === s_outer_acquire &&
                              (xact_allocate || !pending_put_data(oacq_data_idx))
    io.outer.acquire.bits := acq

    when(state === s_outer_acquire && oacq_data_done) { state := next_state }
  }

  // Going back to the original inner transaction:
  // We read from the the cache at this level if data wasn't written back or refilled.
  // We may still merge further Gets, requiring further beats to be read.
  // If ECC requires a full writemask, we'll read out data on partial writes as well.
  def readData[T <: Data](port: DecoupledIO[T], packet: T, drop: DecoupledIO[T] => UInt) {
    pending_reads := (pending_reads &
                         drop(port) &
                         dropPendingBitWhenBeatHasData(io.inner.release) &
                         dropPendingBitWhenBeatHasData(io.outer.grant)) |
                       addPendingBitWhenBeatNeedsRead(io.inner.acquire, Bool(alwaysWriteFullBeat))
    port.valid := state === s_busy && pending_reads.orR && !pending_ognt
    port.bits := packet
  }

  // We write data to the cache at this level if it was Put here with allocate flag,
  // written back dirty, or refilled from outer memory.
  def writeData[T <: Data](port: DecoupledIO[T], packet: T, drop: DecoupledIO[T] => UInt) {
    pending_writes := (pending_writes &
                        drop(port) &
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
                           pending_put_data.orR ||
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
