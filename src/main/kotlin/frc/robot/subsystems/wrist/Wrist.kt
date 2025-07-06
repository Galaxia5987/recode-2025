package frc.robot.subsystems.wrist

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class Wrist(private val io: WristIO) : SubsystemBase() {
    val inputs = io.inputs

    @AutoLogOutput private val mechanism = LoggedMechanism2d(2.0, 3.0)
    private val root = mechanism.getRoot("Wrist", 1.0, 1.0)
    private val ligament2d =
        root.append(LoggedMechanismLigament2d("Wrist ligament", 1.2, 0.0))

    @AutoLogOutput private var setpointName: String = Angles.ZERO.name
    private var setpointValue: Angle = Angles.ZERO.angle

    @AutoLogOutput
    var atSetpoint: Trigger = Trigger {
        setpointValue.isNear(inputs.angle, AT_SET_POINT_THERSHOLD)
    }

    private val tuningAngleDegrees = LoggedNetworkNumber("tuning/WristAngle")

    private fun setVoltage(voltage: Voltage): Command =
        this.startEnd(
                { io.setVoltage(voltage) },
                { io.setVoltage(Units.Volts.zero()) },
            )
            .withName("Wrist/setVoltage")

    private fun setAngle(angle: Angles): Command =
        runOnce {
                io.setAngle(angle.angle)
                setpointName = angle.name
                setpointValue = angle.angle
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

    fun l1() = setAngle(Angles.L1)
    fun l2() = setAngle(Angles.L2)
    fun l3() = setAngle(Angles.L3)
    fun l3Manual() = setAngle(Angles.L3_MANUAL)
    fun l4() = setAngle(Angles.L4)
    fun alignL2() = setAngle(Angles.ALIGN_L2)
    fun alignL4() = setAngle(Angles.ALIGN_L4)
    fun l2algae() = setAngle(Angles.L2_ALGAE)
    fun l2algaePickup() = setAngle(Angles.L2_ALGAE_PICKUP)
    fun l2algaePickupEnd() = setAngle(Angles.L2_ALGAE_PICKUP_END)
    fun l3algae() = setAngle(Angles.L3_ALGAE)
    fun l3algaePickup() = setAngle(Angles.L3_ALGAE_PICKUP)
    fun l3algaePickupEnd() = setAngle(Angles.L3_ALGAE_PICKUP_END)
    fun floorAlgae() = setAngle(Angles.FLOOR_ALGAE)
    fun net() = setAngle(Angles.NET)
    fun feeder() = setAngle(Angles.FEEDER)
    fun blockedFeeder() = setAngle(Angles.BLOCKED_FEEDER)
    fun retract() = setAngle(Angles.ZERO)
    fun skyward() = setAngle(Angles.SKYWARD)
    fun max() = setAngle(Angles.MAX)

    fun tunningAngle(): Command =
        run {
                io.setAngle(Units.Degrees.of(tuningAngleDegrees.get()))
                setpointName = "Tuning Angle"
                setpointValue = Units.Degrees.of(tuningAngleDegrees.get())
            }
            .withName("wrist/tuning")

    fun characterize(): Command {
        val routineForwards =
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
        val routineBackwards =
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
                routineForwards.dynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1.0),
                routineBackwards.dynamic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(1.0),
                routineForwards.quasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1.0),
                routineBackwards.quasistatic(SysIdRoutine.Direction.kReverse)
            )
            .withName("Wrist/characterize")
    }

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, inputs)
        ligament2d.setAngle(inputs.angle.`in`(Units.Degree))
    }
}
