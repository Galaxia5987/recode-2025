package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.pathfindToPose

private val FeederRight = Pose2d(1.5, 0.9, Rotation2d.fromDegrees(-125.0))
private val FeederLeft = Pose2d(1.5, 7.1, Rotation2d.fromDegrees(125.0))

enum class RobotStates {
    Idle,
    Feeding,
    PlaceL1,
    PlaceL2,
    PlaceL3,
    PlaceL4,
    Net,
    AlgaePickUpReef,
    IdleHasCoral,
    IdleHasAlgae
}

private var robotState = RobotStates.Idle

private fun setRobotState(state: RobotStates) =
    Commands.runOnce({ robotState = state })

fun startIdle() = setRobotState(RobotStates.Idle)
fun startFeeding() = setRobotState(RobotStates.Feeding)
fun startPlaceL1() = setRobotState(RobotStates.PlaceL1)
fun startPlaceL2() = setRobotState(RobotStates.PlaceL2)
fun startPlaceL3() = setRobotState(RobotStates.PlaceL3)
fun startPlaceL4() = setRobotState(RobotStates.PlaceL4)
fun startNet() = setRobotState(RobotStates.Net)
fun startAlgaePickUpReef() = setRobotState(RobotStates.AlgaePickUpReef)
fun startIdleHasCoral() = setRobotState(RobotStates.IdleHasCoral)
fun startIdleHasAlgae() = setRobotState(RobotStates.IdleHasAlgae)

// Triggers for each state
val IsIdle = Trigger { robotState == RobotStates.Idle }
val IsFeeding = Trigger { robotState == RobotStates.Feeding }
val IsPlaceL1 = Trigger { robotState == RobotStates.PlaceL1 }
val IsPlaceL2 = Trigger { robotState == RobotStates.PlaceL2 }
val IsPlaceL3 = Trigger { robotState == RobotStates.PlaceL3 }
val IsPlaceL4 = Trigger { robotState == RobotStates.PlaceL4 }
val IsNet = Trigger { robotState == RobotStates.Net }
val IsAlgaePickUpReef = Trigger { robotState == RobotStates.AlgaePickUpReef }
val IsIdleHasCoral = Trigger { robotState == RobotStates.IdleHasCoral }
val IsIdleHasAlgae = Trigger { robotState == RobotStates.IdleHasAlgae }

fun bindRobotStateTriggers() {
    IsIdle.onTrue(idle())                        // Define this function
    IsFeeding.onTrue(feeding())                     // Define this function
    IsPlaceL1.onTrue(placeL1())                  // Define this function
    IsPlaceL2.onTrue(placeL2())
    IsPlaceL3.onTrue(placeL3())
    IsPlaceL4.onTrue(placeL4())
    IsNet.onTrue(net())
    IsAlgaePickUpReef.onTrue(algaePickUpReef())
    IsIdleHasCoral.onTrue(idleHasCoral())
    IsIdleHasAlgae.onTrue(idleHasAlgae())
}

fun idleHasAlgae(): Command {
    TODO("Not yet implemented")
}

fun idleHasCoral(): Command =
    elevator.min().alongWith(wrist.skyward())

fun algaePickUpReef(): Command {
    TODO("Not yet implemented")
}

fun net(): Command {
    TODO("Not yet implemented")
}

fun placeL4(): Command {
    TODO("Not yet implemented")
}

fun placeL3(): Command {
    TODO("Not yet implemented")
}

fun placeL2(): Command {
    TODO("Not yet implemented")
}

fun placeL1(): Command {
    TODO("Not yet implemented")
}

fun idle(): Command {
    TODO("Not yet implemented")
}

fun idle(): Command =
    elevator.min().alongWith(wrist.feeder())

fun feeding() =
    sequence(
        Commands.parallel(
            Commands.either(
                pathfindToPose(FeederLeft),
                pathfindToPose(FeederRight)
            ) {
                (FeederRight.distanceFromPoint(drive.pose.translation) >
                        FeederLeft.distanceFromPoint(drive.pose.translation))
            },
            elevator.feeder().alongWith(
                wrist.feeder()
            ),
            gripper.intakeBySensor(),
            Commands.runOnce({ robotState = RobotStates.IdleHasCoral })
        )
    )
