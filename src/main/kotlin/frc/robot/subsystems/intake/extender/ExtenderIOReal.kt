package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.toDistance

class ExtenderIOReal() : ExtenderIO {
    override val inputs: LoggedExtenderInputs = LoggedExtenderInputs()
    private val motor: TalonFX = TalonFX(MOTOR_ID)
    private val positionTorqueRequest = PositionTorqueCurrentFOC(0.0)
    init {
        motor.configurator.apply(MOTOR_CONFIG)
    }
    override fun setLength(length: Angle) {
        motor.setControl(positionTorqueRequest.withPosition(length))
    }

    override fun updateInputs() {
        inputs.length = motor.position.value.toDistance(RADIUS, GEAR_RATIO)
        inputs.current = motor.supplyCurrent.value
        inputs.voltage = motor.supplyVoltage.value
    }
}
