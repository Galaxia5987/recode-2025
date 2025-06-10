package frc.robot.subsystems.arm.elevator

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.gainsPIDController
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType
import frc.robot.lib.toAngle
import frc.robot.lib.toAngular
import frc.robot.lib.toDistance
import frc.robot.lib.toLinear
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean

class ElevatorIOSim : ElevatorIO {
    override val inputs = LoggedElevatorInputs()

    private val isFloored = LoggedNetworkBoolean("isFloored", false)

    private val velocityVoltageRequest = VelocityVoltage(0.0)
    private val positionVoltageRequest = PositionVoltage(0.0)

    private val motors =
        TalonFXSim(
            2,
            GEAR_RATIO,
            MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters),
            1.0,
            TalonType.KRAKEN_FOC
        )

    private val PIDController = gainsPIDController(GAINS)
    init {
        motors.setController(PIDController)
    }

    override fun setHeight(height: Distance) {
        motors.setControl(
            positionVoltageRequest.withPosition(
                height.toAngle(RADIUS, GEAR_RATIO)
            )
        )
    }

    override fun setVelocity(velocity: LinearVelocity) {
        motors.setControl(
            velocityVoltageRequest.withVelocity(
                velocity.toAngular(RADIUS, GEAR_RATIO)
            )
        )
    }

    override fun updateInputs() {
        motors.update(Timer.getFPGATimestamp())
        inputs.isFloored = isFloored.get()
        inputs.mainVoltage = motors.appliedVoltage
        inputs.auxVoltage = motors.appliedVoltage
        inputs.mainCurrent = motors.appliedCurrent
        inputs.auxCurrent = motors.appliedCurrent
        inputs.mainVelocity = motors.velocity.toLinear(RADIUS, GEAR_RATIO)
        inputs.auxVelocity = motors.velocity.toLinear(RADIUS, GEAR_RATIO)
        inputs.height =
            Units.Rotations.of(motors.position).toDistance(RADIUS, GEAR_RATIO)
    }
}
