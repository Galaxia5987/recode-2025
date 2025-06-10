package frc.robot.subsystems.wrist

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage

class WristIOSim : WristIO {
    override var inputs: LoggedWristInputs = LoggedWristInputs()
    private val motor = MOTOR_CONFIG_SIM
    private val angleController = PIDController(3.0, 0.0, 0.0)
    private val positionControl = PositionVoltage(0.0)
    private val voltageOut = VoltageOut(0.0)

    init {
        motor.setController(angleController)
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageOut.withOutput(voltage))
    }

    override fun setAngle(angle: Angle) {
        motor.setControl(positionControl.withPosition(angle))
    }

    override fun updateInputs() {
        motor.update(edu.wpi.first.wpilibj.Timer.getFPGATimestamp() * 100)
        inputs.angle = Units.Rotations.of(motor.position)
        inputs.appliedVoltage = motor.appliedVoltage
        inputs.velocity = motor.velocity
    }
}
