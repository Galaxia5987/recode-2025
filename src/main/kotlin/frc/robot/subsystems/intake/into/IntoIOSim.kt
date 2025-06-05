package frc.robot.subsystems.intake.into

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Timer
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

class IntoIOSim() : IntoIO {
    override val inputs: LoggedIntoInputs = LoggedIntoInputs()
    private val voltageRequest = VoltageOut(0.0)
    private val motor = TalonFXSim(MOTOR_ID,GEAR_RATIO,MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters),CONVERSION_FACTOR, TalonType.KRAKEN_FOC)
    init {
        motor.setController(PIDController)
    }
    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    override fun updateInputs() {
        motor.update(Timer.getFPGATimestamp())
        inputs.voltage = motor.appliedVoltage
        inputs.current = motor.appliedCurrent
    }
}