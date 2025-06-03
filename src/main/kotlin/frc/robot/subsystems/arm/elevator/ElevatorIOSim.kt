package frc.robot.subsystems.arm.elevator

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType
import frc.robot.lib.toAngle
import frc.robot.lib.toAngular
import frc.robot.lib.toDistance
import frc.robot.lib.toLinear
import kotlin.math.PI
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean

class ElevatorIOSim : ElevatorIO {
    override val inputs: LoggedElevatorInputs = LoggedElevatorInputs()

    private val isFloored = LoggedNetworkBoolean("isFloored", false)

    private val velocityVoltageRequest: VelocityVoltage = VelocityVoltage(0.0)
    private val positionVoltageRequest: PositionVoltage = PositionVoltage(0.0)

    private val mainMotor = TalonFXSim(2, GEAR_RATIO, 0.003, 1.0, TalonType.KRAKEN_FOC)

    init {
        mainMotor.setController(PIDController)
    }

    override fun setHeight(height: Distance) {
        mainMotor.setControl(
            positionVoltageRequest.withPosition(
                height.toAngle(RADIUS, GEAR_RATIO)
            )
        )
    }


    override fun setVelocity(velocity: LinearVelocity) {
        mainMotor.setControl(
            velocityVoltageRequest.withVelocity(
                velocity.toAngular(RADIUS, GEAR_RATIO)
            )
        )
    }

    override fun isFloored(): Boolean = isFloored.get()

    override fun updateInputs() {
        mainMotor.update(Timer.getFPGATimestamp())
        inputs.isFloored = isFloored.get()
        inputs.mainVoltage = mainMotor.appliedVoltage
        inputs.auxVoltage = mainMotor.appliedVoltage
        inputs.mainCurrent = mainMotor.appliedCurrent
        inputs.auxCurrent = mainMotor.appliedCurrent
        inputs.mainVelocity = mainMotor.velocity.toLinear(RADIUS, GEAR_RATIO)
        inputs.auxVelocity = mainMotor.velocity.toLinear(RADIUS, GEAR_RATIO)
        inputs.height = Units.Rotations.of(mainMotor.position).toDistance(RADIUS,GEAR_RATIO)
    }
}
