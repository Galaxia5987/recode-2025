package frc.robot.subsystems.arm.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import kotlin.math.pow
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

class Elevator(private val io: ElevatorIO) : SubsystemBase() {
    private val mechanism = LoggedMechanism2d(3.0, 3.0)
    private val root = mechanism.getRoot("Elevator", 2.0, 0.0)
    private val elevatorLigament =
        root.append(LoggedMechanismLigament2d("ElevatorLigament", 5.0, 90.0))

    private var setpoint = Units.Meters.zero()

    @AutoLogOutput
    val atSetpoint = Trigger { io.inputs.height.isNear(setpoint, TOLERANCE) }

    fun isDangerousVelocityUpwards(velocity: LinearVelocity) =
        (Heights.MAX.height.`in`(Units.Meters) -
            io.inputs.height.`in`(Units.Meters)) <
            -velocity.`in`(Units.MetersPerSecond).pow(2) / 2 *
                SAFETY_ACCELERATION_UP

    fun isDangerousVelocityDownwards(velocity: LinearVelocity) =
        (Heights.MIN.height.`in`(Units.Meters) -
            io.inputs.height.`in`(Units.Meters)) <
            -velocity.`in`(Units.MetersPerSecond).pow(2) / 2 *
                SAFETY_ACCELERATION_DOWN

    fun setVelocity(velocity: LinearVelocity) =
        runOnce { io.setVelocity(velocity) }
            .andThen( // Keeps the command running unless interrupted or reaches the top.
                ConditionalCommand(
                        run {}
                            .until {
                                isDangerousVelocityUpwards(
                                    io.inputs.mainVelocity
                                )
                            }
                            .andThen(setHeight(Heights.MAX.height)),
                        run {}
                            .until {
                                isDangerousVelocityDownwards(
                                    io.inputs.mainVelocity
                                )
                            }
                            .andThen(setHeight(Heights.MIN.height))
                    ) {
                        velocity >= Units.MetersPerSecond.zero()
                    }
                    .until { atSetpoint.asBoolean }
            )

    fun setHeight(height: Distance) =
        runOnce {
                io.setHeight(height)
                setpoint = height
            }
            .withName("Elevator/setHeight")

    fun l1() = setHeight(Heights.L1.height).withName("Elevator/L1")
    fun l2() = setHeight(Heights.L2.height).withName("Elevator/L2")
    fun l3() = setHeight(Heights.L3.height).withName("Elevator/L3")
    fun l4() = setHeight(Heights.L4.height).withName("Elevator/L4")

    override fun periodic() {
        io.updateInputs()
        Logger.recordOutput("Elevator/Mechanism2d", mechanism)
        Logger.recordOutput("Elevator/Setpoint", setpoint)
        Logger.processInputs("Elevator", io.inputs)
        elevatorLigament.length = io.inputs.height.`in`(Units.Meters)
    }
}
