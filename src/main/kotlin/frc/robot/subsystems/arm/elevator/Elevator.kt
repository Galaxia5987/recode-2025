package frc.robot.subsystems.arm.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
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
    val atSetpoint: Trigger = Trigger {
        io.inputs.height.isNear(setpoint, TOLERANCE)
    }

    fun setVelocity(velocity: LinearVelocity) =
        runOnce { io.setVelocity(velocity) }.withName("Elevator/setVelocity")

    fun setHeight(height: Distance) =
        runOnce {
                io.setHeight(height)
                setpoint = height
            }
            .withName("Elevator/setHeight")

    fun l1() = setHeight(ElevatorSet.L1.height).withName("Elevator/L1")
    fun l2() = setHeight(ElevatorSet.L2.height).withName("Elevator/L2")
    fun l3() = setHeight(ElevatorSet.L3.height).withName("Elevator/L3")
    fun l4() = setHeight(ElevatorSet.L4.height).withName("Elevator/L4")

    override fun periodic() {
        io.updateInputs()
        Logger.recordOutput("Mechanism2d", mechanism)
        Logger.recordOutput("Elevator/Setpoint", setpoint)
        Logger.processInputs("Elevator", io.inputs)
        elevatorLigament.length = io.inputs.height.`in`(Units.Meters)
    }
}
