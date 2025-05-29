package frc.robot.subsystems.arm.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput

class Elevator(private val io: ElevatorIO) : SubsystemBase() {
    @AutoLogOutput var setPoint: Distance = Units.Meters.zero()

    val atSetpoint: Trigger = Trigger {
        io.getHeight().isNear(setPoint, TOLERANCE)
    }

    val floored: Trigger = Trigger { io.isFloored() }.onTrue(reset())

    fun setVelocity(velocity: LinearVelocity): Command = run {
        io.setVelocity(velocity)
    }

    fun setHeight(height: Distance): Command = run {
        io.setHeight(height)
        setPoint = height
    }

    fun l1(): Command = setHeight(l1Height)
    fun l2(): Command = setHeight(l2Height)
    fun l3(): Command = setHeight(l3Height)
    fun l4(): Command = setHeight(l4Height)
    fun reset(): Command = run { reset() }

    override fun periodic() {
        io.updateInputs()
    }
}
