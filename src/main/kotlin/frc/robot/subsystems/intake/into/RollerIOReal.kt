package frc.robot.subsystems.intake.into

import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Voltage

class RollerIOReal() : RollerIO {
    override val inputs = LoggedRollerInputs()
    private val motor = TalonFX(MOTOR_ID)
    private val voltageRequest = VoltageOut(0.0)

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    override fun updateInputs() {
        inputs.voltage = motor.supplyVoltage.value
        inputs.current = motor.supplyCurrent.value
        inputs.angle = motor.position.value
    }
}
