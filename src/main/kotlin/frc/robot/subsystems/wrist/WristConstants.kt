package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.TalonFXSConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.motors.TalonFXSim
import frc.robot.lib.motors.TalonType

const val ROTOR_TO_SENSOR = 1 / ((1.0 / 9.0) * (20.0 / 66.0))
const val SENSOR_TO_MECHANISM = 1 / (16.0 / 42.0)
const val GEAR_RATIO = 1 / ((1.0 / 9.0) * (20.0 / 66.0) * (16.0 / 42.0))

val FORWARD_SOFT_LIMIT = Rotation2d.fromRotations(1.8)
val REVERSE_SOFT_LIMIT = Rotation2d.fromRotations(0.0)
val GRER_RATIO = 1 / ((1.0 / 9.0) * (20.0 / 66.0) * (16.0 / 42.0))
val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.001)
var AT_SET_POINT: Angle = Units.Degrees.of(1.5)
var NEAR_SET_POINT: Angle = Units.Degrees.of(10.0)
val RESET_VOLTAGE: Voltage = Units.Volt.of(-10.0)
var MOTOR_CONFIG =
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
        MOMENT_OF_INERTIA.`in`(Units.KilogramSquareMeters),
        1.0,
        TalonType.KRAKEN_FOC
    )

val Double.deg: Angle
    get() = Units.Degrees.of(this)

enum class Angles(val angle: Angle) {
    L1(20.0.deg),
    L2(0.0.deg),
    L3(153.0.deg),
    L3_MANUAL(170.0.deg),
    L4(90.0.deg),
    ALIGN_L2(120.0.deg),
    ALIGN_L4(128.8.deg),
    L2_ALGAE(80.0.deg),
    L2_ALGAE_PICKUP(120.0.deg),
    L2_ALGAE_PICKUP_END(140.0.deg),
    L3_ALGAE(130.0.deg),
    L3_ALGAE_PICKUP(120.0.deg),
    L3_ALGAE_PICKUP_END(140.0.deg),
    FLOOR_ALGAE(55.0.deg),
    NET(210.0.deg),
    FEEDER(17.0.deg),
    BLOCKED_FEEDER(25.0.deg),
    MAX(245.0.deg),
    SKYWARD(200.0.deg),
    ZERO(0.0.deg)
}
