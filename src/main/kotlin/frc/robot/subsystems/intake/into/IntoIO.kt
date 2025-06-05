package frc.robot.subsystems.intake.into

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface IntoIO {
    val inputs: LoggedIntoInputs
    fun setVoltage(voltage: Voltage) {}
    fun updateInputs() {}

    @Logged
    open class IntoInputs {
        var voltage: Voltage = Units.Volt.zero()
        var current: Current = Units.Amp.zero()
    }
}