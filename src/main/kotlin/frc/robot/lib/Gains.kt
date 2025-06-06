package frc.robot.lib

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.Slot1Configs
import com.ctre.phoenix6.configs.Slot2Configs
import edu.wpi.first.math.controller.PIDController

data class Gains(
    val kP: Double = 0.0,
    val kI: Double = 0.0,
    val kD: Double = 0.0,
    val kS: Double = 0.0,
    val kV: Double = 0.0,
    val kA: Double = 0.0,
    val kG: Double = 0.0
)

/**
 * Creates a WPILib [PIDController] instance using the proportional, integral,
 * and derivative gains from the given [Gains].
 *
 * @param gains The [Gains] object containing PID parameters.
 * @return A configured [PIDController] instance.
 */
fun gainsPIDController(gains: Gains): PIDController =
    PIDController(gains.kP, gains.kI, gains.kD)

/**
 * Converts the [Gains] into a [Slot0Configs] object for CTRE motor controllers.
 *
 * @param gains The [Gains] object containing all relevant control parameters.
 * @return A configured [Slot0Configs] instance.
 */
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

/**
 * Converts the [Gains] into a [Slot1Configs] object for CTRE motor controllers.
 *
 * @param gains The [Gains] object containing all relevant control parameters.
 * @return A configured [Slot1Configs] instance.
 */
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

/**
 * Converts the [Gains] into a [Slot2Configs] object for CTRE motor controllers.
 *
 * @param gains The [Gains] object containing all relevant control parameters.
 * @return A configured [Slot2Configs] instance.
 */
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
