package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import frc.robot.lib.Gains

const val MECHANISM_HEIGHT = 3.0
const val MECHANISM_WIDTH = 3.0

const val MECHANISM_ROOT_X = 0.0
const val MECHANISM_ROOT_Y = 2.0

const val MECHANISM_LIGAMENT_LENGTH = 5.0
const val MECHANISM_LIGAMENT_ANGLE = 0.0

val OPEN_LENGTH = Units.Meters.one()
val CLOSE_LENGTH = Units.Meters.zero()

val RADIUS = Units.Millimeters.of(36.4 / 2)

const val MOTOR_ID = 1

val MOMENT_OF_INERTIA = Units.KilogramSquareMeters.of(0.003)
val TOLERANCE = Units.Centimeter.of(2.0)

const val CONVERSION_FACTOR = 1.0
const val GEAR_RATIO = 1.0 / 3.0

val Gains = Gains(1.0)

val PIDController = PIDController(Gains.kP, Gains.kI, Gains.kD)

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
