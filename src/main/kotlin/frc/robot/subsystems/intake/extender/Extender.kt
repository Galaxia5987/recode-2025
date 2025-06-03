package frc.robot.subsystems.intake.extender

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

class Extender(private val io: ExtenderIO) : SubsystemBase() {
    private val mechanism = LoggedMechanism2d(3.0, 3.0)
    private val root = mechanism.getRoot("Extender", 0.0, 2.0)
    private val elevatorLigament =
        root.append(LoggedMechanismLigament2d("ExtenderLigament", 5.0, 0.0))

    private var setPoint: Distance = Units.Meters.zero()

    @AutoLogOutput
    val atSetpoint: Trigger = Trigger {
        io.inputs.length.isNear(setPoint, TOLERANCE)
    }

    fun setLength(length: Distance): Command = runOnce {
        io.setLength(length)
        setPoint = length
    }

    fun open(): Command = setLength(OPEN_LENGTH)
    fun close(): Command = setLength(CLOSE_LENGTH)

    override fun periodic() {
        io.updateInputs()
        Logger.recordOutput("mechanism2d", mechanism)
        Logger.recordOutput("SetPoint", setPoint)
        Logger.processInputs("Extender", io.inputs)
        elevatorLigament.length = io.inputs.length.`in`(Units.Meters)
    }
}
