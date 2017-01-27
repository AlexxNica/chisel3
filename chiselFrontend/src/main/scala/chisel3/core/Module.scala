// See LICENSE for license details.

package chisel3.core

import scala.collection.mutable.ArrayBuffer
import scala.language.experimental.macros
import chisel3.internal._
import chisel3.internal.Builder._
import chisel3.internal.firrtl._
import chisel3.internal.firrtl.{Command => _, _}
import chisel3.internal.sourceinfo.{InstTransform, SourceInfo, UnlocatableSourceInfo}

object Module {
  /** A wrapper method that all Module instantiations must be wrapped in
    * (necessary to help Chisel track internal state).
    *
    * @param bc the Module being created
    *
    * @return the input module `m` with Chisel metadata properly set
    */
  def apply[T <: BaseModule](bc: => T): T = macro InstTransform.apply[T]

  def do_apply[T <: BaseModule](bc: => T)(implicit sourceInfo: SourceInfo): T = {
    // Don't generate source info referencing parents inside a module, sincce this interferes with
    // module de-duplication in FIRRTL emission.
    val childSourceInfo = UnlocatableSourceInfo

    if (Builder.readyForModuleConstr) {
      throwException("Error: Called Module() twice without instantiating a Module." +
                     sourceInfo.makeMessage(" See " + _))
    }
    Builder.readyForModuleConstr = true
    val parent: Option[BaseModule] = Builder.currentModule

    val m = bc.close() // This will set currentModule and unset readyForModuleConstr!!!

    if (Builder.readyForModuleConstr) {
      throwException("Error: attempted to instantiate a Module, but nothing happened. " +
                     "This is probably due to rewrapping a Module instance with Module()." +
                     sourceInfo.makeMessage(" See " + _))
    }
    Builder.currentModule = parent // Back to parent!

    val ports = m.firrtlPorts
    // Blackbox inherits from Module so we have to match on it first TODO fix
    val component = m match {
      case bb: BlackBox =>
        DefBlackBox(bb, bb.name, ports, bb.params)
      case mod: UserModule =>
        DefModule(mod, mod.name, ports, mod.getCommands)
    }
    m._component = Some(component)
    Builder.components += component
    // Avoid referencing 'parent' in top module
    if(!Builder.currentModule.isEmpty) {
      pushCommand(DefInstance(sourceInfo, m, ports))
      m match {
        case m: ImplicitModule => {
          pushCommand(DefInvalid(sourceInfo, m.io.ref)) // init instance inputs
          m.connectClocks(Builder.getClock, Builder.getReset)
        }
        case m => {
          for((_,port) <- m.ports) pushCommand(DefInvalid(sourceInfo, port.ref))
        }
      }
    }
    m
  }
}

/** Abstract base class for Modules, an instantiable organizational unit for RTL.
  */
// TODO: seal this?
abstract class BaseModule extends HasId {
  //
  // Builder Internals - this tracks which Module RTL construction belongs to.
  //
  Builder.currentModule = Some(this)
  if (!Builder.readyForModuleConstr) {
    throwException("Error: attempted to instantiate a Module without wrapping it in Module().")
  }
  readyForModuleConstr = false

  //
  // Module Construction Internals
  //
  protected val _namespace = Builder.globalNamespace.child
  protected val _ids = ArrayBuffer[HasId]()
  private[chisel3] def addId(d: HasId) { _ids += d }
  private[core] def close(): this.type

  //
  // Chisel Internals
  //
  /** Desired name of this module. Override this to give this module a custom, perhaps parametric,
    * name.
    */
  def desiredName = this.getClass.getName.split('.').last

  /** Legalized name of this module. */
  final val name = Builder.globalNamespace.name(desiredName)

  /** Keep component for signal names */
  private[chisel3] var _component: Option[Component] = None

  /** Signal name (for simulation). */
  override def instanceName =
    if (_parent == None) name else _component match {
      case None => getRef.name
      case Some(c) => getRef fullName c
    }

  /** Compatibility function. Allows Chisel2 code which had ports without the IO wrapper to
    * compile under Bindings checks. Does nothing in non-compatibility mode.
    *
    * Should NOT be used elsewhere. This API will NOT last.
    *
    * TODO: remove this, perhaps by removing Bindings checks in compatibility mode.
    */
  def _autoWrapPorts {}

  /** Returns ports, as a list of port name to the actual node. Ports should be Port-bound.
    *
    * Exception for Chisel compatibility: calling this will Port-bind io.
    */
  private[core] def ports: Seq[(String, Data)]

  // TODO: dedup with ports
  private[core] def firrtlPorts: Seq[firrtl.Port] = {
    for ((name, port) <- ports) yield {
      // Port definitions need to know input or output at top-level.
      // By FIRRTL semantics, 'flipped' becomes an Input
      val direction = if(Data.isFirrtlFlipped(port)) Direction.Input else Direction.Output
      firrtl.Port(port, direction)
    }
  }

  //
  // BaseModule User API functions
  //
  def annotate(annotation: ChiselAnnotation): Unit = {
    Builder.annotations += annotation
  }

  /**
   * This must wrap the datatype used to set the io field of any Module.
   * i.e. All concrete modules must have defined io in this form:
   * [lazy] val io[: io type] = IO(...[: io type])
   *
   * Items in [] are optional.
   *
   * The granted iodef WILL NOT be cloned (to allow for more seamless use of
   * anonymous Bundles in the IO) and thus CANNOT have been bound to any logic.
   * This will error if any node is bound (e.g. due to logic in a Bundle
   * constructor, which is considered improper).
   *
   * TODO(twigg): Specifically walk the Data definition to call out which nodes
   * are problematic.
   */
  def IO[T<:Data](iodef: T): iodef.type = {
    // Bind each element of the iodef to being a Port
    Binding.bind(iodef, PortBinder(this), "Error: iodef")
  }
}

/** Abstract base class for Modules that contain Chisel RTL.
  */
abstract class UserModule(implicit moduleCompileOptions: CompileOptions)
    extends BaseModule {
  //
  // RTL construction internals
  //
  protected val _commands = ArrayBuffer[Command]()
  private[chisel3] def addCommand(c: Command) { _commands += c }
  private[core] def getCommands = _commands

  /** Called at the Module.apply(...) level after this Module has finished elaborating.
    */
  private[core] def close(): this.type = {
    for ((name, port) <- ports) {
      port.setRef(ModuleIO(this, _namespace.name(name)))
      // Initialize output as unused
      _commands.prepend(DefInvalid(UnlocatableSourceInfo, port.ref))
    }

    /** Recursively suggests names to supported "container" classes
      * Arbitrary nestings of supported classes are allowed so long as the
      * innermost element is of type HasId
      * Currently supported:
      *   - Iterable
      *   - Option
      * (Note that Map is Iterable[Tuple2[_,_]] and thus excluded)
      */
    def nameRecursively(prefix: String, nameMe: Any): Unit =
      nameMe match {
        case (id: HasId) => id.suggestName(prefix)
        case Some(elt) => nameRecursively(prefix, elt)
        case (iter: Iterable[_]) if iter.hasDefiniteSize =>
          for ((elt, i) <- iter.zipWithIndex) {
            nameRecursively(s"${prefix}_${i}", elt)
          }
        case _ => // Do nothing
      }

    /** Scala generates names like chisel3$util$Queue$$ram for private vals
      * This extracts the part after $$ for names like this and leaves names
      * without $$ unchanged
      */
    def cleanName(name: String): String = name.split("""\$\$""").lastOption.getOrElse(name)
    for (m <- getPublicFields(classOf[UserModule])) {
      nameRecursively(cleanName(m.getName), m.invoke(this))
    }

    // For Module instances we haven't named, suggest the name of the Module
    _ids foreach {
      case m: BaseModule => m.suggestName(m.name)
      case _ =>
    }

    // All suggestions are in, force names to every node.
    _ids.foreach(_.forceName(default="_T", _namespace))
    _ids.foreach(_._onModuleClose)
    this
  }

  //
  // Other Internal Functions
  //
  // For debuggers/testers, TODO: refactor out into proper public API
  lazy val getPorts = firrtlPorts
  val compileOptions = moduleCompileOptions
}

/** Abstract base class for Modules, which behave much like Verilog modules.
  * These may contain both logic and state which are written in the Module
  * body (constructor).
  *
  * @note Module instantiations must be wrapped in a Module() call.
  */
abstract class ImplicitModule(
  override_clock: Option[Clock]=None, override_reset: Option[Bool]=None)
  (implicit moduleCompileOptions: CompileOptions)
extends UserModule {
  // Allow access to bindings from the compatibility package
  protected def _ioPortBound() = io.flatten.map(x => x.binding match {
    case _: chisel3.core.PortBinding => true
    case _: chisel3.core.SynthesizableBinding => throw new AssertionError("Internal error: bad IO binding")
    case _ => false
  }).reduce(_ && _)

    // _clock and _reset can be clock and reset in these 2ary constructors
    // once chisel2 compatibility issues are resolved
    def this(_clock: Clock)(implicit moduleCompileOptions: CompileOptions) = this(Option(_clock), None)(moduleCompileOptions)
    def this(_reset: Bool)(implicit moduleCompileOptions: CompileOptions)  = this(None, Option(_reset))(moduleCompileOptions)
    def this(_clock: Clock, _reset: Bool)(implicit moduleCompileOptions: CompileOptions) = this(Option(_clock), Option(_reset))(moduleCompileOptions)


  /** IO for this Module. At the Scala level (pre-FIRRTL transformations),
    * connections in and out of a Module may only go through `io` elements.
    */
  def io: Record
  val clock = IO(Input(Clock()))
  val reset = IO(Input(Bool()))

  private[core] override def ports: Seq[(String,Data)] = Seq(
    ("clock", clock), ("reset", reset), ("io", io)
  )

  def connectClocks(externalClock: Option[Clock], externalReset: Option[Bool]) {
    implicit val sourceInfo = UnlocatableSourceInfo
    externalClock map {clock := _}
    externalReset map {reset := _}
  }
}
