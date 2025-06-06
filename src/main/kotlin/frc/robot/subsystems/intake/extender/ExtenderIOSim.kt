package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.gainsPIDController
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType
import frc.robot.lib.toDistance

class ExtenderIOSim : ExtenderIO {
    override val inputs: LoggedExtenderInputs = LoggedExtenderInputs()
    private val motor =
        TalonFXSim(
            MOTOR_ID,
            GEAR_RATIO,
            MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters),
            CONVERSION_FACTOR,
            TalonType.KRAKEN_FOC
        )
    private val voltagePositionRequest = PositionVoltage(0.0)

    private val PIDController = gainsPIDController(Gains)

    init {
        motor.setController(PIDController)
    }

    override fun setLength(length: Angle) {
        motor.setControl(voltagePositionRequest.withPosition(length))
    }

    override fun updateInputs() {
        inputs.length =
            Units.Rotations.of(motor.position).toDistance(RADIUS, GEAR_RATIO)
        inputs.current = motor.appliedCurrent
        inputs.voltage = motor.appliedVoltage
        motor.update(Timer.getTimestamp())
    }
}
