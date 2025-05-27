package frc.robot.subsystems.arm

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ElevatorIO {
    fun setHeight(length: Distance) {}

    fun setVelocity(velocity: LinearVelocity) {}
    fun updateInputs() {}

    @Logged
    open class Inputs {
        var velocity: LinearVelocity = Units.MetersPerSecond.zero()
        var voltage: Voltage = Units.Volt.zero()
        var current: Current = Units.Amp.zero()
        var flooredSensor: Boolean = false
    }

}