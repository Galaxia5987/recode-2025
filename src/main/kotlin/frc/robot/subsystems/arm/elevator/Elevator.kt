package frc.robot.subsystems.arm.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
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

    fun setVelocity(velocity: LinearVelocity): Command =
        runOnce { io.setVelocity(velocity) }.withName("Elevator/setVelocity")

    fun setHeight(height: Distance): Command =
        runOnce {
                io.setHeight(height)
                setpoint = height
            }
            .withName("Elevator/setHeight")

    fun l1(): Command =
        setHeight(ElevatorHeight.L1.height).withName("Elevator/setL1")
    fun l2(): Command =
        setHeight(ElevatorHeight.L2.height).withName("Elevator/setL2")
    fun l3(): Command =
        setHeight(ElevatorHeight.L3.height).withName("Elevator/setL3")
    fun l4(): Command =
        setHeight(ElevatorHeight.L4.height).withName("Elevator/setL4")

    override fun periodic() {
        io.updateInputs()
        Logger.recordOutput("Mechanism2d", mechanism)
        Logger.recordOutput("SetPoint", setpoint)
        Logger.processInputs("Elevator", io.inputs)
        elevatorLigament.length = io.inputs.height.`in`(Units.Meters)
    }
}
