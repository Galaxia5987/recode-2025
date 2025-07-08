package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.extender
import frc.robot.roller

enum class IntakeState {
    Intaking,
    Idling,
    Outtaking,
    Resetting
}

private var intakeState = IntakeState.Resetting

private fun setState(state: IntakeState) =
    Commands.runOnce({ intakeState = state })

fun startIntaking() = setState(IntakeState.Intaking)

fun startOuttaking() = setState(IntakeState.Outtaking)

fun startIdling() = setState(IntakeState.Idling)

fun startResetting() = setState(IntakeState.Resetting)

val IsIntaking = Trigger { intakeState == IntakeState.Intaking }
val IsIdling = Trigger { intakeState == IntakeState.Idling }
val IsOuttaking = Trigger { intakeState == IntakeState.Outtaking }
val IsResetting = Trigger { intakeState == IntakeState.Resetting }

fun bindIntakeTriggers() {
    IsIntaking.onTrue(intake())
    IsIdling.onTrue(idle())
    IsOuttaking.onTrue(outtake())
    IsResetting.onTrue(reset())
}

fun intake() =
    sequence(
        extender.open(),
        WaitUntilCommand(extender.isOpen),
        roller.intake()
    )

fun outtake() =
    sequence(
        extender.open(),
        WaitUntilCommand(extender.isOpen),
        roller.outtake()
    )

fun idle() =
    sequence(extender.open(), WaitUntilCommand(extender.isOpen), roller.stop())

fun reset() =
    sequence(
        roller.stop(),
        extender.close(),
        WaitUntilCommand(extender.isClosed)
    )
