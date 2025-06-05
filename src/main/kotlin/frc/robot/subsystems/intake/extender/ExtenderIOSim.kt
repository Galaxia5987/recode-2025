package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType
import frc.robot.lib.toAngle
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

    init {
        motor.setController(PIDController)
    }

    override fun setLength(length: Distance) {
        motor.setControl(
            voltagePositionRequest.withPosition(
                length.toAngle(RADIUS, GEAR_RATIO)
            )
        )
    }

    override fun updateInputs() {
        inputs.length =
            Units.Rotations.of(motor.position).toDistance(RADIUS, GEAR_RATIO)
        inputs.current = motor.appliedCurrent
        inputs.voltage = motor.appliedVoltage
        motor.update(Timer.getFPGATimestamp())
    }
}
