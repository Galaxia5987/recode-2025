package frc.robot.subsystems.climb

import com.ctre.phoenix6.configs.TalonFXSConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.MomentOfInertia
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

val GRER_RATIO = 1 / ((1.0 / 9.0) * (1.0 / 3.0) * (1.0 / 3.0) * (30.0 / 60.0))
val MOMENT_OF_INERTIA: MomentOfInertia =
    edu.wpi.first.units.Units.KilogramSquareMeters.of(0.003)
val MOTOR_CONFIG =
    TalonFXSConfiguration().apply {
        MotorOutput.apply {
            NeutralMode = NeutralModeValue.Brake
            Inverted = InvertedValue.CounterClockwise_Positive
        }
    }
var MOTOR_CONFIG_SIM =
    TalonFXSim(
        1,
        GRER_RATIO,
        MOMENT_OF_INERTIA.`in`(edu.wpi.first.units.Units.KilogramSquareMeters),
        1.0,
        TalonType.KRAKEN_FOC
    )
