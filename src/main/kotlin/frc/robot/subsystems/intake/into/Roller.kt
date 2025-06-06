package frc.robot.subsystems.intake.into

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Roller(private val io: RollerIO) : SubsystemBase() {
    private var appliedVoltage = Units.Volt.zero()

    fun setVoltage(voltage: Voltage) =
        runOnce {
                io.setVoltage(voltage)
                appliedVoltage = voltage
            }
            .withName("Roller/setVoltage")

    fun inTake() = setVoltage(intakeVoltage).withName("Roller/InTake")
    fun outTake() = setVoltage(outtakeVoltage).withName("Roller/OutTake")

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("Roller", io.inputs)
        Logger.recordOutput("RollerSetVoltage", appliedVoltage)
    }
}
