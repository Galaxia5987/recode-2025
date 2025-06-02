package frc.robot.subsystems.arm.elevator

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import frc.robot.lib.toAngle
import frc.robot.lib.toAngular
import frc.robot.lib.toLinear

class ElevatorIOReal() : ElevatorIO {
    override val inputs: LoggedElevatorInputs = LoggedElevatorInputs()
    private val velocityVoltageController: VelocityVoltage = VelocityVoltage(0.0)
    private val positionVoltageController: PositionVoltage = PositionVoltage(0.0)
    private val mainMotor = TalonFX(MAIN_MOTOR_ID)
    private val auxMotor = TalonFX(AUX_MOTOR_ID)

    init {
        mainMotor.configurator.apply(MOTOR_CONFIG)

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
        Units.Meters.of(
            RADIUS.`in`(Units.Meters) *
                mainMotor.position.value.`in`(Units.Radian)
        )

    override fun setVelocity(velocity: LinearVelocity) {
        mainMotor.setControl(
            velocityVoltageController.withVelocity(
                velocity.toAngular(RADIUS, GEAR_RATIO)
            )
        )
    }

    override fun isFloored(): Boolean = mainMotor.reverseLimit.value.value == 1
    override fun updateInputs() {
        inputs.isFloored = mainMotor.reverseLimit.value.value == 1
        inputs.mainVoltage = mainMotor.supplyVoltage.value
        inputs.auxVoltage = auxMotor.supplyVoltage.value
        inputs.mainCurrent = mainMotor.supplyCurrent.value
        inputs.auxCurrent = auxMotor.supplyCurrent.value
        inputs.mainVelocity =
            mainMotor.velocity.value.toLinear(RADIUS, GEAR_RATIO)
        inputs.auxVelocity =
            auxMotor.velocity.value.toLinear(RADIUS, GEAR_RATIO)
    }
    override fun reset() {
        mainMotor.setPosition(0.0)
        auxMotor.setPosition(0.0)
    }
}
