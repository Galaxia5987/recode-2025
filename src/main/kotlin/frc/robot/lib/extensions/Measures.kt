package frc.robot.lib.extensions

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.*
import edu.wpi.first.units.measure.*
import kotlin.math.PI

// Length
val m get() = Units.Meters
val cm get() = Units.Centimeters
val mm get() = Units.Millimeters

// Angle
val deg get() = Units.Degrees
val rad get() = Units.Radians
val rot get() = Units.Rotations

// Time
val sec get() = Units.Seconds

// Other
val percent get() = Units.Percent
val amps get() = Units.Amps
val volts get() = Units.Volts

fun LinearVelocity.toAngular(
    diameter: Distance,
    gearRatio: Double,
): AngularVelocity = timesConversionFactor(
    Units.RotationsPerSecond.per(Units.MetersPerSecond)
        .of(1.0 / (diameter[m] * gearRatio * PI))
)

fun Distance.toAngle(diameter: Distance, gearRatio: Double): Angle = timesConversionFactor(
    Units.Rotations.per(Units.Meters)
        .of(1.0 / (diameter[m] * gearRatio * PI))
)

fun Angle.toDistance(diameter: Distance, gearRatio: Double): Distance =
    timesConversionFactor(
        Units.Meters.per(Units.Rotations)
            .of(diameter[m] * gearRatio * PI)
    )

fun AngularVelocity.toLinear(
    diameter: Distance,
    gearRatio: Double,
): LinearVelocity =
    timesConversionFactor(
        Units.MetersPerSecond.per(Units.RotationsPerSecond)
            .of(diameter[m] * gearRatio * PI)
    )


operator fun Distance.div(time: TimeUnit): LinearVelocity = this / time.one()
operator fun Distance.div(divisor: Number): Distance = this / divisor.toDouble()
operator fun Voltage.div(time: TimeUnit): Velocity<VoltageUnit> = this / time.one()


// Factories

// Helper function for conversion
inline fun <N : Number, R> N.toUnit(converter: (Double) -> R) = converter(toDouble())

// Distance
val Number.m: Distance get() = toUnit(Units.Meters::of)
val Number.cm: Distance get() = toUnit(Units.Centimeters::of)
val Number.mm: Distance get() = toUnit(Units.Millimeters::of)
operator fun Distance.get(unit: DistanceUnit): Double = this.`in`(unit)

// Linear velocity
val Number.mps: LinearVelocity get() = toUnit(Units.MetersPerSecond::of)

// Angle
val Number.deg: Angle get() = toUnit(Units.Degrees::of)
val Number.rot: Angle get() = toUnit(Units.Rotations::of)
val Number.rad: Angle get() = toUnit(Units.Radians::of)
fun Angle.toRotation2d(): Rotation2d = Rotation2d(`in`(Units.Radians))
operator fun Angle.get(unit: AngleUnit): Double = this.`in`(unit)

// Angular velocity
val Number.deg_ps: AngularVelocity get() = toUnit(Units.DegreesPerSecond::of)
val Number.rot_ps: AngularVelocity get() = toUnit(Units.RotationsPerSecond::of)
val Number.rad_ps: AngularVelocity get() = toUnit(Units.RadiansPerSecond::of)

// Other
val Number.sec: Time get() = toUnit(Units.Seconds::of)
operator fun Time.get(unit: TimeUnit): Double = this.`in`(unit)

val Number.percent: Dimensionless get() = toUnit(Units.Percent::of)
operator fun Dimensionless.get(unit: DimensionlessUnit): Double = this.`in`(unit)

val Number.amps: Current get() = toUnit(Units.Amps::of)
operator fun Current.get(unit: CurrentUnit): Double = this.`in`(unit)


operator fun Voltage.get(unit: VoltageUnit): Double = this.`in`(unit)
val Number.volts: Voltage get() = toUnit(Units.Volts::of)

val Number.kg2m: MomentOfInertia get() = toUnit(Units.KilogramSquareMeters::of)
operator fun MomentOfInertia.get(unit: MomentOfInertiaUnit): Double = this.`in`(unit)
