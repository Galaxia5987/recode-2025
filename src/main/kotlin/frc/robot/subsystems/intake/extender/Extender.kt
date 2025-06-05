package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.toAngle
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

class Extender(private val io: ExtenderIO) : SubsystemBase() {
    private val mechanism = LoggedMechanism2d(3.0, 3.0)
    private val root = mechanism.getRoot("Extender", 0.0, 2.0)
    private val elevatorLigament =
        root.append(LoggedMechanismLigament2d("ExtenderLigament", 5.0, 0.0))

    private var setpoint = Units.Meters.zero()

    @AutoLogOutput
    val atSetpoint = Trigger { io.inputs.length.isNear(setpoint, TOLERANCE) }

    fun setLength(length: Distance) = runOnce {
        io.setLength(length.toAngle(RADIUS, GEAR_RATIO))
        setpoint = length
    }

    fun open() = setLength(OPEN_LENGTH)
    fun close() = setLength(CLOSE_LENGTH)

    override fun periodic() {
        io.updateInputs()
        Logger.recordOutput("mechanism2d", mechanism)
        Logger.recordOutput("SetPoint", setpoint)
        Logger.processInputs("Extender", io.inputs)
        elevatorLigament.length = io.inputs.length.`in`(Units.Meters)
    }
}
