// See LICENSE for license details.

package uncore
import Chisel._
import cde.{Parameters, Field}

trait HasDataBuffer extends HasCoherenceAgentParameters {
  val data_buffer = Reg(init=Vec.fill(innerDataBeats)(UInt(0, width = innerDataBits)))
  // TODO: provide func for accessing when innerDataBeats =/= outerDataBeats or internalDataBeats
}

trait HasByteWriteMaskBuffer extends HasCoherenceAgentParameters { 
  val wmask_buffer = Reg(init=Vec.fill(innerDataBeats)(UInt(0, width = innerWriteMaskBits)))
}

trait HasBlockAddressBuffer extends HasCoherenceAgentParameters {
  val s_idle :: s_meta_read :: s_meta_resp :: s_wb_req :: s_wb_resp :: s_inner_probe :: s_outer_acquire :: s_busy :: s_meta_write :: Nil = Enum(UInt(), 9)
  val state = Reg(init=s_idle)

  val xact_addr_block = Reg(init = UInt(0, width = blockAddrBits))
}


trait HasAcquireMetadataBuffer extends HasBlockAddressBuffer {
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
