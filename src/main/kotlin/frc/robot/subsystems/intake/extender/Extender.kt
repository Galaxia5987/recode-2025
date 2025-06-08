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
    private val mechanism = LoggedMechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT)
    private val root =
        mechanism.getRoot("Extender", MECHANISM_ROOT_X, MECHANISM_ROOT_Y)
    private val extenderLigament =
        root.append(
            LoggedMechanismLigament2d(
                "ExtenderLigament",
                MECHANISM_LIGAMENT_LENGTH,
                MECHANISM_LIGAMENT_ANGLE
            )
        )

    private var setpoint = Units.Meters.zero()

    @AutoLogOutput
    val atSetpoint = Trigger { io.inputs.length.isNear(setpoint, TOLERANCE) }

    @AutoLogOutput
    val isOpen = Trigger { io.inputs.length.isNear(OPEN_LENGTH, TOLERANCE) }
    @AutoLogOutput
    val isClosed = Trigger { io.inputs.length.isNear(CLOSE_LENGTH, TOLERANCE) }

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
        extenderLigament.length = io.inputs.length.`in`(Units.Meters)
    }
}
