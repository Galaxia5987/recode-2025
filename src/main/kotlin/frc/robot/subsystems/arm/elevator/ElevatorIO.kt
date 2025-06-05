package frc.robot.subsystems.arm.elevator

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ElevatorIO {

    val inputs: LoggedElevatorInputs

    fun setHeight(height: Distance) {}
    fun setVelocity(velocity: LinearVelocity) {}
    fun updateInputs() {}
    fun reset() {}

    @Logged
    open class ElevatorInputs {
        var mainVelocity: LinearVelocity = Units.MetersPerSecond.zero()
        var auxVelocity: LinearVelocity = Units.MetersPerSecond.zero()
        var mainVoltage: Voltage = Units.Volt.zero()
        var auxVoltage: Voltage = Units.Volt.zero()
        var mainCurrent: Current = Units.Amp.zero()
        var auxCurrent: Current = Units.Amp.zero()
        var isFloored: Boolean = false
        var height: Distance = Units.Meters.zero()
    }
}
