package frc.robot.subsystems.example

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.math.controller.PIDController
import frc.robot.lib.Gains
import frc.robot.lib.gainsPIDController
import frc.robot.lib.gainsPIDSlot0
import frc.robot.lib.selectGainsBasedOnMode

val gains = selectGainsBasedOnMode(Gains(1.0), Gains(1.0))

val pidController0 = PIDController(gains.kP,gains.kI,gains.kD)
// לאומת
val pidController1 = gainsPIDController(gains)

// also 

val MOTOR_CONFIG = TalonFXConfiguration().apply { 
    Slot0 = Slot0Configs().apply { 
        kP = gains.kP
        kD = gains.kD
        kI = gains.kI
        kA = gains.kA
        kS = gains.kS
        kG = gains.kG
        kV = gains.kV
    }
    // לאומת
    Slot0 = gainsPIDSlot0(gains)
}