package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.toAngle
import frc.robot.lib.toDistance

class ExtenderIOReal() : ExtenderIO {
    override val inputs: LoggedExtenderInputs = LoggedExtenderInputs()
    private val motor: TalonFX = TalonFX(MOTOR_ID)
    private val positionTorqueRequest = MotionMagicTorqueCurrentFOC(0.0)
    init {
        motor.configurator.apply(MOTOR_CONFIG)
    }
    override fun setLength(length: Distance) {
        motor.setControl(
            positionTorqueRequest.withPosition(
                length.toAngle(RADIUS, GEAR_RATIO)
            )
        )
    }

    override fun updateInputs() {
        inputs.length = motor.position.value.toDistance(RADIUS, GEAR_RATIO)
        inputs.current = motor.supplyCurrent.value
        inputs.voltage = motor.supplyVoltage.value
    }
}
