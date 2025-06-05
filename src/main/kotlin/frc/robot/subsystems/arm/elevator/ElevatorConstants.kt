package frc.robot.subsystems.arm.elevator

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.ReverseLimitSourceValue
import com.ctre.phoenix6.signals.ReverseLimitTypeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.MomentOfInertia
import frc.robot.lib.Gains
import frc.robot.lib.selectGainsBasedOnMode

enum class ElevatorHeight(val height: Distance) {
    L1(Units.Meters.of(0.0)),
    L2(Units.Meters.of(0.0)),
    L3(Units.Meters.of(15.0)),
    L4(Units.Meters.of(105.0))
}

val TOLERANCE = Units.Centimeter.of(2.0)
val RADIUS = Units.Millimeters.of(36.4 / 2)

val MOMENT_OF_INERTIA = Units.KilogramSquareMeters.of(0.003)

const val CONVERSION_FACTOR = 1.0
const val GEAR_RATIO = 1.0
val GAINS = selectGainsBasedOnMode(Gains(1.0), Gains(1.0))

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
        Slot0Configs().apply {
            kP = GAINS.kP
            kI = GAINS.kI
            kD = GAINS.kP
        }
        HardwareLimitSwitch =
            HardwareLimitSwitchConfigs().apply {
                ReverseLimitType = ReverseLimitTypeValue.NormallyOpen
                ReverseLimitEnable = true
                ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin
                ReverseLimitAutosetPositionEnable = true
                ReverseLimitAutosetPositionValue = 0.0
                ReverseLimitRemoteSensorID = LIMIT_SWITCH_ID
            }
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimit = 40.0
            }
    }
