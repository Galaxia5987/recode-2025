package frc.robot.subsystems.climb

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import javax.management.timer.Timer

class ClimberIOSim : ClimberIO {
    override val inputs: LoggedClimberInputs = LoggedClimberInputs()
    private val motor = MOTOR_CONFIG_SIM
    private var voltageControl = VoltageOut(0.0)

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageControl.withOutput(voltage))
    }

    override fun updateInputs() {
        motor.update(edu.wpi.first.wpilibj.Timer.getFPGATimestamp() * 100)
        inputs.angle = Units.Rotations.of(motor.position)
        inputs.appliedVoltage = motor.appliedVoltage
    }
}
