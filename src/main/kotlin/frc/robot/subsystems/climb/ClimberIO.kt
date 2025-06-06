package frc.robot.subsystems.climb

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage

interface ClimberIO {
    val inputs: LoggedClimberInputs

    fun setVoltage(voltage: Voltage) {}
    fun updateInputs() {}

    @Logged
    open class ClimberInputs {
        var appliedVoltage: Voltage = Units.Volts.zero()
        var angle: Angle = Units.Degree.zero()
    }
}
