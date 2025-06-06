package frc.robot.subsystems.intake.into

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import org.team9432.annotation.Logged

interface RollerIO {
    val inputs: LoggedRollerInputs
    fun setVoltage(voltage: Voltage) {}
    fun updateInputs() {}

    @Logged
    open class RollerInputs {
        var voltage = Units.Volt.zero()
        var current = Units.Amp.zero()
    }
}
