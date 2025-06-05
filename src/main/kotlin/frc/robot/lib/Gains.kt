package frc.robot.lib

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.Slot2Configs
import edu.wpi.first.math.controller.PIDController
import frc.robot.CURRENT_MODE
import frc.robot.Mode

data class Gains(
    val kP: Double = 0.0,
    val kI: Double = 0.0,
    val kD: Double = 0.0,
    val kS: Double = 0.0,
    val kV: Double = 0.0,
    val kA: Double = 0.0,
    val kG: Double = 0.0
)

fun selectGainsBasedOnMode(realGains: Gains, simGains: Gains): Gains {
    return if (CURRENT_MODE == Mode.SIM) simGains else realGains
}

fun gainsPIDController(gains: Gains): PIDController = PIDController(gains.kP, gains.kI, gains.kD)
fun gainsPIDSlot0(gains: Gains): Slot0Configs =
    Slot0Configs().apply {
        kP = gains.kP
        kI = gains.kI
        kD = gains.kD
        kA = gains.kA
        kG = gains.kG
        kS = gains.kS
        kV = gains.kV
    }
fun gainsPIDSlot1(gains: Gains): Slot1Configs =
    Slot1Configs().apply {
        kP = gains.kP
        kI = gains.kI
        kD = gains.kD
        kA = gains.kA
        kG = gains.kG
        kS = gains.kS
        kV = gains.kV
    }
fun gainsPIDSlot2(gains: Gains): Slot2Configs =
    Slot2Configs().apply {
        kP = gains.kP
        kI = gains.kI
        kD = gains.kD
        kA = gains.kA
        kG = gains.kG
        kS = gains.kS
        kV = gains.kV
    }