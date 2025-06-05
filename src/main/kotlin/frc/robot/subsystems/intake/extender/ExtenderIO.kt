package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ExtenderIO {
    val inputs: LoggedExtenderInputs

    fun setLength(length: Angle) {}
    fun updateInputs() {}

    @Logged
    open class ExtenderInputs {
        var length = Units.Meters.zero()
        var current = Units.Amp.zero()
        var voltage = Units.Volt.zero()
    }
}
