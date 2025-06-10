package frc.robot.subsystems.gripper

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.AnalogInput
import edu.wpi.first.math.filter.MedianFilter
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage

class GripperIOReal : GripperIO {
    override val inputs: LoggedGripperInputs = LoggedGripperInputs()

    private val motor = TalonFX(16)
    private val voltageControl = VoltageOut(0.0).withEnableFOC(true)

    private val sensor = edu.wpi.first.wpilibj.AnalogInput(3)
    private val distanceFilter = MedianFilter(3)

    init {
        motor.configurator.apply(
            TalonFXConfiguration().apply {
                MotorOutput =
                    MotorOutputConfigs().apply {
                        NeutralMode = NeutralModeValue.Brake
                        Inverted = InvertedValue.CounterClockwise_Positive
                    }
                CurrentLimits =
                    CurrentLimitsConfigs().apply {
                        StatorCurrentLimitEnable = true
                        SupplyCurrentLimitEnable = true
                        StatorCurrentLimit = 80.0
                        SupplyCurrentLimit = 40.0
                    }
            }
        )
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(VoltageOut(voltage).withEnableFOC(true))
    }

    override fun updateInputs() {
        var calculatedDistance =
            distanceFilter.calculate(4800 / (200 * sensor.voltage - 20.0))
        inputs.sensorDistance = Units.Centimeters.of(calculatedDistance)
        inputs.current = motor.statorCurrent.value
        inputs.appliedVoltage = motor.motorVoltage.value
    }
}
