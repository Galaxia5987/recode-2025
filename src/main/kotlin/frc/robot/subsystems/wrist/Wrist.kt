package frc.robot.subsystems.wrist

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class Wrist(private val io: WristIO) : SubsystemBase() {
    var inputs = io.inputs

    @AutoLogOutput private val mechanism = LoggedMechanism2d(2.0, 3.0)
    private val root = mechanism.getRoot("Wrist", 1.0, 1.0)
    private val ligament2d =
        root.append(LoggedMechanismLigament2d("Wrist ligament", 1.2, 0.0))

    @AutoLogOutput private var SetpointName: String = Angles.ZERO.name
    private var SetpointValue: Angle = Angles.ZERO.angle

    @AutoLogOutput
    var atSetPoint: Trigger = Trigger {
        SetpointValue.isNear(io.inputs.angle, AT_SET_POINT)
    }

    @AutoLogOutput
    var nearPoint: Trigger = Trigger {
        SetpointValue.isNear(io.inputs.angle, NEAR_SET_POINT)
    }

    private val tuningAngleDegrees = LoggedNetworkNumber("tuning/WristAngle")

    private fun setVoltage(voltage: Voltage): Command =
        StartEndCommand(
                { io.setVoltage(voltage) },
                { io.setVoltage(Units.Volts.zero()) }
            )
            .withName("Wrist/setVoltage")

    private fun setAngle(angle: Angles): Command =
        runOnce {
                io.setAngle(angle.angle)
                SetpointName = angle.name
                SetpointValue = angle.angle
            }
            .withName("Wrist/${angle.name}")

    private fun reset(resetTrigger: Trigger): Command =
        Commands.sequence(
                runOnce { io.setSoftLimits(false) }
                    .andThen(setVoltage(RESET_VOLTAGE))
                    .until(resetTrigger),
                runOnce { io.resetAbsoluteEncoder(Units.Degree.zero()) },
                runOnce { io.setSoftLimits(true) }
            )
            .withName("Wrist/reset")

    fun l1(): Command = setAngle(Angles.L1)
    fun l2(): Command = setAngle(Angles.L2)
    fun l3(): Command = setAngle(Angles.L3)
    fun l3Manual(): Command = setAngle(Angles.L3_MANUAL)
    fun l4(): Command = setAngle(Angles.L4)
    fun alignL2(): Command = setAngle(Angles.ALIGN_L2)
    fun alignL4(): Command = setAngle(Angles.ALIGN_L4)
    fun l2algae(): Command = setAngle(Angles.L2_ALGAE)
    fun l2algaePickup(): Command = setAngle(Angles.L2_ALGAE_PICKUP)
    fun l2algaePickupEnd(): Command = setAngle(Angles.L2_ALGAE_PICKUP_END)
    fun l3algae(): Command = setAngle(Angles.L3_ALGAE)
    fun l3algaePickup(): Command = setAngle(Angles.L3_ALGAE_PICKUP)
    fun l3algaePickupEnd(): Command = setAngle(Angles.L3_ALGAE_PICKUP_END)
    fun floorAlgae(): Command = setAngle(Angles.FLOOR_ALGAE)
    fun net(): Command = setAngle(Angles.NET)
    fun feeder(): Command = setAngle(Angles.FEEDER)
    fun blockedFeeder(): Command = setAngle(Angles.BLOCKED_FEEDER)
    fun retract(): Command = setAngle(Angles.ZERO)
    fun skyward(): Command = setAngle(Angles.SKYWARD)
    fun max(): Command = setAngle(Angles.MAX)

    fun tunningAngle(): Command =
        run {
                io.setAngle(Units.Degrees.of(tuningAngleDegrees.get()))
                SetpointName = "Tuning Angle"
                SetpointValue = Units.Degrees.of(tuningAngleDegrees.get())
            }
            .withName("wrist/tuning")

    fun characterize(): Command {
        val routineForWards =
            SysIdRoutine(
                SysIdRoutine.Config(
                    Units.Volt.per(Units.Second).of(5.0),
                    Units.Volt.of(6.0),
                    Units.Second.of(1.5),
                    { Logger.recordOutput("Wrist/state", it) }
                ),
                SysIdRoutine.Mechanism(
                    { io.setVoltage(it) },
                    null,
                    this,
                )
            )
        val routineBeckWards =
            SysIdRoutine(
                SysIdRoutine.Config(
                    Units.Volt.per(Units.Second).of(5.0),
                    Units.Volt.of(6.0),
                    Units.Second.of(1.5),
                    { Logger.recordOutput("Wrist/state", it) }
                ),
                SysIdRoutine.Mechanism({ io.setVoltage(it) }, null, this)
            )
        return Commands.sequence(
                routineForWards.dynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1.0),
                routineBeckWards.dynamic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(1.0),
                routineForWards.quasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1.0),
                routineBeckWards.quasistatic(SysIdRoutine.Direction.kReverse)
            )
            .withName("Wrist/characterize")
    }

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)
        ligament2d.setAngle(io.inputs.angle.`in`(Units.Degree))
    }
}
