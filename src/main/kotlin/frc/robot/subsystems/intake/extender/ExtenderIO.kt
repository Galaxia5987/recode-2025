package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface ExtenderIO {
    val inputs: LoggedExtenderInputs

    fun setLength(length: Distance)
    fun updateInputs()

    @Logged
    open class ExtenderInputs {
        var length: Distance = Units.Meters.zero()
        var current: Current = Units.Amp.zero()
        var voltage: Voltage = Units.Volt.zero()
    }
}
