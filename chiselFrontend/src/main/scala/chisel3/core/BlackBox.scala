// See LICENSE for license details.

package chisel3.core

import chisel3.internal.Builder.pushCommand
import chisel3.internal.firrtl._
import chisel3.internal.throwException
import chisel3.internal.sourceinfo.{SourceInfo, UnlocatableSourceInfo}
// TODO: remove this once we have CompileOptions threaded through the macro system.
import chisel3.core.ExplicitCompileOptions.NotStrict

/** Parameters for BlackBoxes */
sealed abstract class Param
case class IntParam(value: BigInt) extends Param
case class DoubleParam(value: Double) extends Param
case class StringParam(value: String) extends Param
/** Unquoted String */
case class RawParam(value: String) extends Param

/** Defines a black box, which is a module that can be referenced from within
  * Chisel, but is not defined in the emitted Verilog. Useful for connecting
  * to RTL modules defined outside Chisel.
  *
  * @example
  * {{{
  * ... to be written once a spec is finalized ...
  * }}}
  * @note The parameters API is experimental and may change
  */
abstract class BlackBox(val params: Map[String, Param] = Map.empty[String, Param]) extends BaseModule {
  def io: Record

  private[core] override def generateComponent(): Component = {
    require(!_closed, "Can't generate module more than once")
    _closed = true

    val namedPorts = io.elements.toSeq
    // setRef is not called on the actual io.
    // There is a risk of user improperly attempting to connect directly with io
    // Long term solution will be to define BlackBox IO differently as part of
    //   it not descending from the (current) Module
    for ((name, port) <- namedPorts) {
      port.setRef(ModuleIO(this, _namespace.name(name)))
    }

    // We need to call forceName and onModuleClose on all of the sub-elements
    // of the io bundle, but NOT on the io bundle itself.
    // Doing so would cause the wrong names to be assigned, since their parent
    // is now the module itself instead of the io bundle.
    for (id <- _ids; if id ne io) {
      id.forceName(default="_T", _namespace)
      id._onModuleClose
    }

    val firrtlPorts = for ((_, port) <- namedPorts) yield {
      // Port definitions need to know input or output at top-level.
      // By FIRRTL semantics, 'flipped' becomes an Input
      val direction = if(Data.isFirrtlFlipped(port)) Direction.Input else Direction.Output
      Port(port, direction)
    }

    val component = DefBlackBox(this, name, firrtlPorts, params)
    _component = Some(component)
    component
  }

  private[core] def initializeInParent(externalClock: Option[Clock], externalReset: Option[Bool]) {
    implicit val sourceInfo = UnlocatableSourceInfo

    val namedPorts = io.elements.toSeq
    for ((_, port) <- namedPorts) {
      pushCommand(DefInvalid(sourceInfo, port.ref))
    }
  }
}
