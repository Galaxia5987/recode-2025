package frc.robot.subsystems.intake.into

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import frc.robot.lib.Gains

val MOMENT_OF_INERTIA = Units.KilogramSquareMeters.of(0.003)
val intakeVoltage = Units.Volt.one()
val outtakeVoltage = Units.Volt.of(-1.0)

const val CONVERSION_FACTOR = 1.0
const val GEAR_RATIO = 1.0

val Gains = Gains(1.0)

const val MOTOR_ID = 1

val MOTOR_CONFIG: TalonFXConfiguration =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        Feedback = FeedbackConfigs().apply { RotorToSensorRatio = 1.0 }
        Slot0 =
            Slot0Configs().apply {
                kP = Gains.kP
                kI = Gains.kI
                kD = Gains.kD
            }
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimit = 40.0
            }
    }
