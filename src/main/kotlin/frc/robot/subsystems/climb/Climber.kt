package frc.robot.subsystems.climb

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

class Climber(private val io: ClimberIO) : SubsystemBase() {
    var inputs = io.inputs

    @AutoLogOutput private val mechaism = LoggedMechanism2d(3.0, 2.0)
    private val root = mechaism.getRoot("climber", 1.0, 1.0)
    private val ligament =
        root.append(
            LoggedMechanismLigament2d("climber ligament", 0.27003, 90.0)
        )

    val angle: () -> Angle = { io.inputs.angle }

    private fun setVoltage(voltage: Voltage): Command =
        StartEndCommand(
                { io.setVoltage(voltage) },
                { io.setVoltage(Units.Volts.zero()) }
            )
            .withName("Climber/setVoltage")

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)
        ligament.angle = inputs.angle.`in`(Units.Degree)
    }
}
