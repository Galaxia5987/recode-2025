package frc.robot.subsystems.intake.into

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

class Roller(private val io: RollerIO) : SubsystemBase() {
    private val mechanism = LoggedMechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT)
    private val root =
        mechanism.getRoot("Roller", MECHANISM_ROOT_X, MECHANISM_ROOT_Y)
    private val elevatorLigament =
        root.append(
            LoggedMechanismLigament2d(
                "RollerLigament",
                MECHANISM_LIGAMENT_LENGTH,
                MECHANISM_LIGAMENT_ANGLE
            )
        )

    private var appliedVoltage = Units.Volt.zero()
    fun setVoltage(voltage: Voltage) =
        runOnce {
                io.setVoltage(voltage)
                appliedVoltage = voltage
            }
            .withName("Roller/setVoltage")

    fun inTake() = setVoltage(INTAKE_VOLTAGE).withName("Roller/InTake")
    fun outTake() = setVoltage(OUTTAKE_VOLTAGE).withName("Roller/OutTake")
    fun stop() = setVoltage(Units.Volt.zero()).withName("Roller/Stop")

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("Roller", io.inputs)
        Logger.recordOutput("RollerAppliedVoltage", appliedVoltage)
        Logger.recordOutput("Roller2dMechanism", mechanism)
        elevatorLigament.angle = io.inputs.angle.`in`(Units.Degree)
    }
}
