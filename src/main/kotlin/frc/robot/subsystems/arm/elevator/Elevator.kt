package frc.robot.subsystems.arm.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput

class Elevator(private val io: ElevatorIO) : SubsystemBase() {
    private var setPoint: Distance = Units.Meters.zero()

    @AutoLogOutput
    val atSetpoint: Trigger = Trigger {
        io.getHeight().isNear(setPoint, TOLERANCE)
    }

    @AutoLogOutput
    val floored: Trigger = Trigger { io.getFlooredSensor() }.onTrue(reset())

    fun setVelocity(velocity: LinearVelocity): Command = runOnce {
        io.setVelocity(velocity)
    }.withName("Elevator/setVelocity")

    fun setHeight(height: Distance): Command = runOnce {
        io.setHeight(height)
        setPoint = height
    }.withName("Elevator/setHeight")

    fun l1(): Command = setHeight(l1Height).withName("Elevator/setL1")
    fun l2(): Command = setHeight(l2Height).withName("Elevator/setL2")
    fun l3(): Command = setHeight(l3Height).withName("Elevator/setL3")
    fun l4(): Command = setHeight(l4Height).withName("Elevator/setL4")
    fun reset(): Command = runOnce { reset() }.withName("Elevator/reset")

    override fun periodic() {
        io.updateInputs()
    }
}
