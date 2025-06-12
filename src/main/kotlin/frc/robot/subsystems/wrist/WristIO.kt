package frc.robot.subsystems.wrist

import edu.wpi.first.epilogue.Logged
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage

interface WristIO {
    val inputs: LoggedWristInputs

    fun setAngle(angle: Angle) {}
    fun setVoltage(voltage: Voltage) {}
    fun setSoftLimits(value: Boolean) {}
    fun resetAbsoluteEncoder(angle: Angle) {}
    fun updateInputs() {}

    @org.team9432.annotation.Logged
    open class WristInputs {
        var appliedVoltage: Voltage = Units.Volts.zero()
        var angle: Angle = Units.Degree.zero()
        var velocity: AngularVelocity = Units.RotationsPerSecond.zero()
        var absoluteEncoderAngle: Angle = Units.Degree.zero()
        var noOffsetAbsoluteEncoderPosition: Angle = Units.Degree.zero()
    }
}
