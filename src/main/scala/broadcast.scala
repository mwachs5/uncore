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

trait BroadcastsToAllClients extends HasCoherenceAgentParameters {
  val inner_coh = ManagerMetadata.onReset
  val outer_coh = ClientMetadata.onReset
  def full_representation = ~UInt(0, width = innerNCachingClients)
}

abstract class BroadcastVoluntaryReleaseTracker(trackerId: Int)(implicit p: Parameters)
    extends VoluntaryReleaseTracker(trackerId)(p) 
    with EmitsVoluntaryReleases
    with BroadcastsToAllClients {
  val io = new HierarchicalXactTrackerIO
  pinAllReadyValidLow(io)

  // Checks for illegal behavior
  assert(!(state === s_idle && io.inner.release.fire() && io.alloc.irel && !io.irel().isVoluntary()),
    "VoluntaryReleaseTracker accepted Release that wasn't voluntary!")
}

abstract class BroadcastAcquireTracker(trackerId: Int)(implicit p: Parameters)
    extends AcquireTracker(trackerId)(p) 
    with EmitsVoluntaryReleases
    with BroadcastsToAllClients {
  val io = new HierarchicalXactTrackerIO
  pinAllReadyValidLow(io)

  val alwaysWriteFullBeat = false
  val nSecondaryMisses = 1
  def iacq_can_merge = Bool(false)

  // Checks for illegal behavior
  // TODO: this could be allowed, but is a useful check against allocation gone wild
  assert(!(state === s_idle && io.inner.acquire.fire() && io.alloc.iacq &&
    io.iacq().hasMultibeatData() && !io.iacq().first()),
    "AcquireTracker initialized with a tail data beat.")

  assert(!(state =/= s_idle && pending_ignt && xact_iacq.isPrefetch()),
    "Broadcast Hub does not support Prefetches.")

  assert(!(state =/= s_idle && pending_ignt && xact_iacq.isAtomic()),
    "Broadcast Hub does not support PutAtomics.")
}

class BufferedBroadcastVoluntaryReleaseTracker(trackerId: Int)(implicit p: Parameters)
    extends BroadcastVoluntaryReleaseTracker(trackerId)(p)
    with HasDataBuffer {

  routeInParent()

  // Start transaction by accepting inner release
  innerRelease(block_vol_ignt = pending_orel || pending_vol_ognt)

  io.inner.release.ready := state === s_idle || irel_can_merge || irel_same_xact

  when(irel_is_allocating) { pending_orel := io.irel().hasData() }

  when(io.inner.release.fire()) { data_buffer(io.irel().addr_beat) := io.irel().data }

  // Dispatch outer release
  outerRelease(outer_coh.onHit(M_XWR), data_buffer(orel_data_idx))

  quiesce()
}

class BufferedBroadcastAcquireTracker(trackerId: Int)(implicit p: Parameters)
    extends BroadcastAcquireTracker(trackerId)(p)
    with HasByteWriteMaskBuffer {

  // Setup IOs used for routing in the parent
  routeInParent()

  // First, take care of accpeting new acquires or secondary misses
  // Handling of primary and secondary misses' data and write mask merging
  innerAcquire(
    can_alloc = Bool(false),
    next = s_inner_probe)

  io.inner.acquire.ready := state === s_idle || iacq_can_merge || iacq_same_xact 

  // Track which clients yet need to be probed and make Probe message
  // If a writeback occurs, we can forward its data via the buffer,
  // and skip having to go outwards
  val skip_outer_acquire = pending_ignt_data.andR

  innerProbe(
    inner_coh.makeProbe(curr_probe_dst, xact_iacq, xact_addr_block),
    Mux(!skip_outer_acquire, s_outer_acquire, s_busy))

  // Handle incoming releases from clients, which may reduce sharer counts
  // and/or write back dirty data, and may be unexpected voluntary releases
  def irel_can_merge = io.irel().conflicts(xact_addr_block) &&
                         io.irel().isVoluntary() &&
                         !Vec(s_idle, s_meta_write).contains(state) &&
                         !all_pending_done &&
                         !io.outer.grant.fire() &&
                         !io.inner.grant.fire() &&
                         !pending_vol_ignt

  innerRelease(block_vol_ignt = pending_vol_ognt) 

  io.inner.release.ready := irel_can_merge || irel_same_xact

  mergeDataInner(io.inner.release)

  // If there was a writeback, forward it outwards
  outerRelease(outer_coh.onHit(M_XWR), data_buffer(orel_data_idx))

  // Send outer request for miss
  outerAcquire(
    caching = !xact_iacq.isBuiltInType(),
    coh = outer_coh,
    data = data_buffer(oacq_data_idx),
    wmask = wmask_buffer(oacq_data_idx),
    next = s_busy)
    
  // Handle the response from outer memory
  mergeDataOuter(io.outer.grant)

  // Acknowledge or respond with data
  innerGrant(
    ignt_data = data_buffer(ignt_data_idx),
    ignt_pending = pending_orel || pending_ognt || pending_vol_ognt)

  when(iacq_is_allocating) {
    wmask_buffer.foreach { w => w := UInt(0) } // This is the only reg that must be clear in s_idle
    initializeProbes()
  }

  initDataInner(io.inner.acquire, iacq_is_accepted)

  // Wait for everything to quiesce
  quiesce()

}
