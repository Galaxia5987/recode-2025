package frc.robot.subsystems.arm.elevator

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.ReverseLimitSourceValue
import com.ctre.phoenix6.signals.ReverseLimitTypeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.Gains
import frc.robot.lib.gainsPIDSlot0
import frc.robot.lib.toAngle

enum class Heights(val height: Distance) {
    L1(Units.Centimeter.of(0.0)),
    L2(Units.Centimeter.of(0.0)),
    L3(Units.Centimeter.of(15.0)),
    L4(Units.Centimeter.of(105.0)),
    MAX(Units.Centimeter.of(140.0)),
    MIN(Units.Centimeter.of(0.0))
}

const val SAFETY_ACCELERATION_UP = -5
const val SAFETY_ACCELERATION_DOWN = -5
val TOLERANCE = Units.Centimeter.of(2.0)
val RADIUS = Units.Millimeters.of(36.4 / 2)

val MOMENT_OF_INERTIA = Units.KilogramSquareMeters.of(0.003)

const val CONVERSION_FACTOR = 1.0
const val GEAR_RATIO = 1.0
val GAINS = Gains(0.5)

const val MAIN_MOTOR_ID = 1
const val AUX_MOTOR_ID = 2
const val LIMIT_SWITCH_ID = 3

val MOTOR_CONFIG: TalonFXConfiguration =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        Feedback = FeedbackConfigs().apply { RotorToSensorRatio = 1.0 }
        Slot0 = gainsPIDSlot0(GAINS)
        HardwareLimitSwitch =
            HardwareLimitSwitchConfigs().apply {
                ReverseLimitType = ReverseLimitTypeValue.NormallyOpen
                ReverseLimitEnable = true
                ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin
                ReverseLimitAutosetPositionEnable = true
                ReverseLimitAutosetPositionValue = 0.0
                ReverseLimitRemoteSensorID = LIMIT_SWITCH_ID
            }
        SoftwareLimitSwitch =
            SoftwareLimitSwitchConfigs().apply {
                ForwardSoftLimitEnable = true
                ForwardSoftLimitThreshold = Heights.MAX.height.toAngle(RADIUS,GEAR_RATIO).`in`(Units.Rotations)
            }
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimit = 40.0
            }
    }
