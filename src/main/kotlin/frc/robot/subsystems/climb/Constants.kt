package frc.robot.subsystems.climb

import com.ctre.phoenix6.configs.TalonFXSConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

var GRER_RATIO = 0.0
var MOMENT_OF_INERTIA = 0.0
var MOTOR_CONFIG =
    TalonFXSConfiguration().apply {
        MotorOutput.apply {
            NeutralMode = NeutralModeValue.Brake
            Inverted = InvertedValue.CounterClockwise_Positive
        }
    }
var MOTOR_CONFIG_SIM =
    TalonFXSim(1, GRER_RATIO, MOMENT_OF_INERTIA, 1.0, TalonType.KRAKEN_FOC)
