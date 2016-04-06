// See LICENSE for license details.

package uncore
import Chisel._
import cde.{Parameters, Field}

class L2BroadcastHub(implicit p: Parameters) extends HierarchicalCoherenceAgent()(p) {

  // Create TSHRs for outstanding transactions
  val trackerList =
    (0 until nReleaseTransactors).map(id =>
      Module(new BufferedBroadcastVoluntaryReleaseTracker(id))) ++
    (nReleaseTransactors until nTransactors).map(id =>
      Module(new BufferedBroadcastAcquireTracker(id)))

  // Propagate incoherence flags
  trackerList.map(_.io.incoherent) foreach { _ := io.incoherent }

  // Handle acquire transaction initiation
  val irel_vs_iacq_conflict =
    io.inner.acquire.valid &&
    io.inner.release.valid &&
    io.irel().conflicts(io.iacq())

  doInputRoutingWithAllocation(
    io.inner.acquire,
    trackerList.map(_.io.inner.acquire),
    trackerList.map(_.io.matches.iacq),
    trackerList.map(_.io.alloc.iacq),
    allocOverride = !irel_vs_iacq_conflict)


  // Handle releases, which might be voluntary and might have data
  doInputRoutingWithAllocation(
    io.inner.release,
    trackerList.map(_.io.inner.release),
    trackerList.map(_.io.matches.irel),
    trackerList.map(_.io.alloc.irel))

  // Wire probe requests and grant reply to clients, finish acks from clients
  doOutputArbitration(io.inner.probe, trackerList.map(_.io.inner.probe))

  doOutputArbitration(io.inner.grant, trackerList.map(_.io.inner.grant))
  // Note that we bypass the Grant data subbundles
  //io.inner.grant.bits.data := io.outer.grant.bits.data
  //io.inner.grant.bits.addr_beat := io.outer.grant.bits.addr_beat

  doInputRouting(io.inner.finish, trackerList.map(_.io.inner.finish))

  // Create an arbiter for the one memory port
  val outerList = trackerList.map(_.io.outer)
  val outer_arb = Module(new ClientTileLinkIOArbiter(outerList.size)
                                                    (p.alterPartial({ case TLId => p(OuterTLId) })))
  outer_arb.io.in <> outerList
  io.outer <> outer_arb.io.out
}

class BroadcastXactTracker(implicit p: Parameters) extends XactTracker()(p) {
  val io = new HierarchicalXactTrackerIO
  pinAllReadyValidLow(io)
}

class BufferedBroadcastVoluntaryReleaseTracker(trackerId: Int)(implicit p: Parameters)
    extends VoluntaryReleaseTracker(trackerId)(p)
    with HasDataBuffer {
  val io = new HierarchicalXactTrackerIO
  pinAllReadyValidLow(io)

  val inner_coh = ManagerMetadata.onReset

  route(io.iacq().conflicts(xact_vol_irel))

  // Initialize and accept pending Release beats
  accept(s_busy)

  //TODO: Use io.outer.release instead?
  write(io.outer.acquire, 
        PutBlock( 
             client_xact_id = UInt(0),
             addr_block = xact_addr_block,
             addr_beat = curr_write_beat,
             data = data_buffer(curr_write_beat))
           (p.alterPartial({ case TLId => outerTLId })),
        dropPendingBitWhenBeatHasData(io.outer.acquire))

  acknowledge(io.outer.grant.valid, io.inner.grant.ready)

  when(state === s_busy && all_pending_done) { state := s_idle }

  // Checks for illegal behavior
  assert(!(state === s_idle && io.inner.release.fire() && !io.irel().isVoluntary()),
    "VoluntaryReleaseTracker accepted Release that wasn't voluntary!")
}

class BufferedBroadcastAcquireTracker(trackerId: Int)(implicit p: Parameters)
    extends AcquireTracker(trackerId)(p)
    with HasByteWriteMaskBuffer {
  val io = new HierarchicalXactTrackerIO
  pinAllReadyValidLow(io)

  val inner_coh = ManagerMetadata.onReset
  val alwaysWriteFullBeat = false
  val nSecondaryMisses = 0

  // Setup IOs are used for routing in the parent
  route(io.iacq().conflicts(xact_addr_block), io.irel().conflicts(xact_addr_block))

  accept(Bool(false), Bool(false), s_inner_probe)
  when(state === s_idle && io.inner.acquire.valid && io.alloc.iacq) {
    initializeProbes(~inner_coh.full(), xact_iacq.client_id, xact_iacq.requiresSelfProbe())
  }

  innerProbe(
    inner_coh.makeProbe(curr_probe_dst, xact_iacq, xact_addr_block),
    s_outer_acquire)

  // Handle incoming releases from clients, which may reduce sharer counts
  // and/or write back dirty data, and may be unexpected voluntary releases
  val irel_can_merge = io.irel().conflicts(xact_addr_block) &&
                         io.irel().isVoluntary() &&
                         !Vec(s_idle, s_meta_write).contains(state) &&
                         !all_pending_done &&
                         !io.outer.grant.fire() &&
                         !io.inner.grant.fire() &&
                         !pending_vol_ignt

  innerRelease(irel_can_merge)
  mergeDataInner(io.inner.release)

  // Send outer request for miss
  //outerAcquire(s_busy)

  // Handle the response from outer memory
  io.outer.grant.ready := state === s_busy
  mergeDataOuter(io.outer.grant)

  // Acknowledge or respond with data
  innerGrant(
    inner_coh.makeGrant(
      sec = ignt_q.io.deq.bits,
      manager_xact_id = UInt(trackerId), 
      data = data_buffer(ignt_data_idx)),
    UInt(0))

  // We must wait for as many Finishes as we sent Grants
  io.inner.finish.ready := state === s_busy

  // Wait for everything to quiesce
  when(state === s_busy && all_pending_done) {
    wmask_buffer.foreach { w => w := UInt(0) } // This is the only reg that must be clear in s_idle
    state := s_idle
  }

  assert(!(state =/= s_idle && io.matches.iacq && io.inner.acquire.fire() &&
    io.iacq().client_id =/= xact_iacq.client_id),
    "AcquireTracker accepted data beat from different network source than initial request.")

  assert(!(state =/= s_idle && io.matches.iacq && io.inner.acquire.fire() &&
    io.iacq().client_xact_id =/= xact_iacq.client_xact_id),
    "AcquireTracker accepted data beat from different client transaction than initial request.")

  assert(!(state === s_idle && io.inner.acquire.fire() && io.alloc.iacq &&
    io.iacq().hasMultibeatData() && io.iacq().addr_beat =/= UInt(0)),
    "AcquireTracker initialized with a tail data beat.")

  assert(!(state =/= s_idle && xact_iacq.isBuiltInType() && (xact_iacq.isPrefetch() || xact_iacq.isAtomic())),
    "Broadcast Hub does not support PutAtomics or prefetches") // TODO add support?
}
