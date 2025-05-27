package frc.robot.subsystems.arm

import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger

class Elevator(private val io: ElevatorIO) : SubsystemBase() {

//    val atSetpoint : Trigger = Trigger
//    val flooredSensor: Trigger = Trigger{}

    fun setVelocity(velocity: LinearVelocity): Command = run {
        io.setVelocity(velocity)
    }
    fun setHeight(height: Distance): Command = run { io.setHeight(height) }
    fun l1(): Command = run { io.setHeight(l1Height) }
    fun l2(): Command = run { io.setHeight(l2Height) }
    fun l3(): Command = run { io.setHeight(l3Height) }
    fun l4(): Command = run { io.setHeight(l4Height) }

    override fun periodic() {
        io.updateInputs()
    }
}
