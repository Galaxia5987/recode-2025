package frc.robot.subsystems.climb

import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Voltage

class ClimberIOReal : ClimberIO {
    override val inputs: LoggedClimberInputs = LoggedClimberInputs()

    private val motor = TalonFX(17)
    private val voltageControl = VoltageOut(0.0)

    init {
        motor.configurator.apply(MOTOR_CONFIG.MotorOutput)
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageControl.withOutput(voltage))
    }

    override fun updateInputs() {
        inputs.angle = motor.position.value
        inputs.appliedVoltage = motor.motorVoltage.value
        inputs.angularVelocity = motor.velocity.value
    }
}
