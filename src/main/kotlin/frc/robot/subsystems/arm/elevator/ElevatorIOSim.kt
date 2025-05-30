package frc.robot.subsystems.arm.elevator

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType
import frc.robot.lib.toAngle
import frc.robot.lib.toAngular
import frc.robot.lib.toLinear
import kotlin.math.PI

class ElevatorIOSim: ElevatorIO {
    override val inputs: LoggedElevatorInputs = LoggedElevatorInputs()

    val velocityVoltageController: VelocityVoltage = VelocityVoltage(0.0)
    val positionVoltageController: PositionVoltage = PositionVoltage(0.0)

    val mainMotor = TalonFXSim(MAIN_MOTOR_ID,GEAR_RATIO,jKgMetersSquared.`in`(Units.KilogramSquareMeters),CONVERSION_FACTOR, TalonType.KRAKEN_FOC)

    init {
        mainMotor.setController(PIDController)
    }

    override fun setHeight(height: Distance) {
        mainMotor.setControl(positionVoltageController.withPosition(height.toAngle(RADIUS,GEAR_RATIO)))
    }

    override fun getHeight(): Distance =
        Units.Meters.of( RADIUS.`in`(Units.Meters) * mainMotor.position*2*PI)

    override fun setVelocity(velocity: LinearVelocity) {
        mainMotor.setControl(velocityVoltageController.withVelocity(velocity.toAngular(RADIUS,GEAR_RATIO)))
    }

    override fun isFloored(): Boolean = false

    override fun updateInputs() {
        inputs.isFloored = false
        inputs.mainVoltage = mainMotor.appliedVoltage
        inputs.auxVoltage = mainMotor.appliedVoltage
        inputs.mainCurrent = mainMotor.appliedCurrent
        inputs.auxCurrent = mainMotor.appliedCurrent
        inputs.mainVelocity =
            mainMotor.velocity.toLinear(RADIUS, GEAR_RATIO)
        inputs.auxVelocity =
            mainMotor.velocity.toLinear(RADIUS, GEAR_RATIO)
    }
}