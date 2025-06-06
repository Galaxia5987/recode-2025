package frc.robot.subsystems.intake.into

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Roller(private val io: RollerIO) : SubsystemBase() {
    private var setVoltage = Units.Volt.zero()

    @AutoLogOutput
    val onSetVoltage = Trigger { io.inputs.voltage == setVoltage }

    fun setVoltage(voltage: Voltage) = runOnce {
        io.setVoltage(voltage)
        setVoltage = voltage
    }.withName("Roller/setVoltage")

    fun inTake() = setVoltage(intakeVoltage).withName("Roller/InTake")
    fun outTake() = setVoltage(outtakeVoltage).withName("Roller/OutTake")

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("Into", io.inputs)
        Logger.recordOutput("IntoSetVoltage", setVoltage)
    }
}
