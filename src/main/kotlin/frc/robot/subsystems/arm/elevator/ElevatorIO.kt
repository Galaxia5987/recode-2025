package frc.robot.subsystems.arm.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import org.team9432.annotation.Logged

interface ElevatorIO {

    val inputs: LoggedElevatorInputs

    fun setHeight(height: Distance) {}
    fun setVelocity(velocity: LinearVelocity) {}
    fun updateInputs() {}

    @Logged
    open class ElevatorInputs {
        var mainVelocity = Units.MetersPerSecond.zero()
        var auxVelocity = Units.MetersPerSecond.zero()
        var mainVoltage = Units.Volt.zero()
        var auxVoltage = Units.Volt.zero()
        var mainCurrent = Units.Amp.zero()
        var auxCurrent = Units.Amp.zero()
        var isFloored = false
        var height = Units.Meters.zero()
    }
}
