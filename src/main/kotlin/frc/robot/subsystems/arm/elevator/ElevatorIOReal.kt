package frc.robot.subsystems.arm.elevator

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import frc.robot.lib.toAngle
import frc.robot.lib.toAngular

class ElevatorRealIO() : ElevatorIO {
    override val inputs: LoggedElevatorInputs = LoggedElevatorInputs()
    val velocityVoltageController: VelocityVoltage = VelocityVoltage(0.0)
    val positionVoltageController: PositionVoltage = PositionVoltage(0.0)
    val mainMotor = TalonFX(MAIN_MOTOR_ID)
    val auxMotor = TalonFX(AUX_MOTOR_ID)

    init {
        mainMotor.configurator.apply(
            MOTOR_CONFIG
        )

        auxMotor.setControl(Follower(MAIN_MOTOR_ID, false))
    }


    override fun setHeight(height: Distance) {
        mainMotor.setControl(
            positionVoltageController.withPosition(
                height.toAngle(RADIUS, GEAR_RATIO)
            )
        )
    }

    override fun getHeight(): Distance =
        Units.Meters.of(RADIUS.`in`(Units.Meters) * mainMotor.position.value.`in`(Units.Radian))

    override fun setVelocity(velocity: LinearVelocity) {
        mainMotor.setControl(
            velocityVoltageController.withVelocity(
                velocity.toAngular(RADIUS, GEAR_RATIO)
            )
        )
    }

    override fun getFlooredSensor(): Boolean = {mainMotor.sens}
    override fun updateInputs() {}
    override fun reset() {}
}