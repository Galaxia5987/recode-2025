package frc.robot.subsystems.intake.into

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

class Roller(private val io: RollerIO) : SubsystemBase() {

    fun setVoltage(voltage: Voltage) =
        runOnce {
                io.setVoltage(voltage)
            }
            .withName("Roller/setVoltage")

    fun intake() = setVoltage(INTAKE_VOLTAGE).withName("Roller/Intake")
    fun outtake() = setVoltage(OUTTAKE_VOLTAGE).withName("Roller/Outtake")
    fun stop() = setVoltage(Units.Volt.zero()).withName("Roller/Stop")

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("Roller", io.inputs)
    }
}
