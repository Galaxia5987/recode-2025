package frc.robot

import com.pathplanner.lib.util.FlippingUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.units.Units.Centimeters
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.flip
import frc.robot.lib.extensions.m
import frc.robot.lib.getTranslation2d
import frc.robot.lib.pathfindToPose
import frc.robot.subsystems.drive.alignToPose
import org.littletonrobotics.junction.Logger

private val FeederRight = Pose2d(1.5, 0.9, Rotation2d.fromDegrees(-125.0))
private val FeederLeft = Pose2d(1.5, 7.1, Rotation2d.fromDegrees(125.0))
val reefLocation: ReefLocation = ReefLocation.Reef1Left
val REEF_RADIUS = 0.8317.m
val nearReefTolerance = 0.4.m
val ReefFaceLeft
    get() = Pose2d(14.32, 3.86, Rotation2d.k180deg).flip()
val ReefFaceRight
    get() = Pose2d(14.32, 4.20, Rotation2d.k180deg).flip()

enum class ReefLocation(val pose2d: Pose2d) {
    Reef4Left(
        ReefFaceLeft.plus(
            Transform2d((-4.0).cm, Centimeters.zero(), Rotation2d.kZero)
        )
    ),
    Reef4Right(
        ReefFaceRight.plus(
            Transform2d((-4.0).cm, Centimeters.zero(), Rotation2d.kZero)
        )
    ),
    Reef5Left(
        Reef4Left.pose2d.rotateAround(ReefCenter, Rotation2d.fromDegrees(60.0))
    ),
    Reef5Right(
        Reef4Right.pose2d.rotateAround(ReefCenter, Rotation2d.fromDegrees(60.0))
    ),
    Reef6Left(
        Reef4Left.pose2d.rotateAround(ReefCenter, Rotation2d.fromDegrees(120.0))
    ),
    Reef6Right(
        Reef4Right.pose2d.rotateAround(
            ReefCenter,
            Rotation2d.fromDegrees(120.0)
        )
    ),
    Reef1Left(
        Reef4Left.pose2d.rotateAround(ReefCenter, Rotation2d.fromDegrees(180.0))
    ),
    Reef1Right(
        Reef4Right.pose2d.rotateAround(
            ReefCenter,
            Rotation2d.fromDegrees(180.0)
        )
    ),
    Reef2Left(
        Reef4Left.pose2d.rotateAround(ReefCenter, Rotation2d.fromDegrees(240.0))
    ),
    Reef2Right(
        Reef4Right.pose2d.rotateAround(
            ReefCenter,
            Rotation2d.fromDegrees(240.0)
        )
    ),
    Reef3Left(
        Reef4Left.pose2d.rotateAround(ReefCenter, Rotation2d.fromDegrees(300.0))
    ),
    Reef3Right(
        Reef4Right.pose2d.rotateAround(
            ReefCenter,
            Rotation2d.fromDegrees(300.0)
        )
    )
}

val ReefCenter
    get() = getTranslation2d(4.48945, FlippingUtil.fieldSizeY / 2)

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

private fun setReefLocation(reefPlacementLocation: ReefLocation) =
    Commands.runOnce({ reefLocation == reefPlacementLocation })

fun setReef4Left() = setReefLocation(ReefLocation.Reef4Left)

fun setReef4Right() = setReefLocation(ReefLocation.Reef4Right)

fun setReef1Left() = setReefLocation(ReefLocation.Reef1Left)

fun setReef1Right() = setReefLocation(ReefLocation.Reef1Right)

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
    IsIdle.onTrue(idle()) // Define this function
    IsFeeding.onTrue(feeding()) // Define this function
    IsPlaceL1.onTrue(placeL1()) // Define this function
    IsPlaceL2.onTrue(placeL2())
    IsPlaceL3.onTrue(placeL3())
    IsPlaceL4.onTrue(placeL4())
    //    IsNet.onTrue(net())
    //    IsAlgaePickUpReef.onTrue(algaePickUpReef())
    IsIdleHasCoral.onTrue(idleHasCoral())
    //    IsIdleHasAlgae.onTrue(idleHasAlgae())
}

fun log() {
    Logger.recordOutput("robot state", robotState)
}

fun idleHasAlgae(): Command {
    TODO("Not yet implemented")
}

fun idleHasCoral(): Command = elevator.min().alongWith(wrist.skyward())

fun algaePickUpReef(): Command {
    TODO("Not yet implemented")
}

fun net(): Command {
    TODO("Not yet implemented")
}

val isNearTargetPose
    get() =
        (drive.pose.distanceFromPoint(reefLocation.pose2d.translation) <
            nearReefTolerance)

fun placeL4(): Command =
    sequence(
        pathfindToPose(reefLocation.pose2d),
        Commands.parallel(
            alignToPose(reefLocation.pose2d),
            elevator.l4().alongWith(wrist.l4()).onlyIf { isNearTargetPose },
        ),
        gripper.outtakeBySensor(),
        Commands.runOnce({ robotState = RobotStates.Idle })
    )

fun placeL3(): Command =
    sequence(
        pathfindToPose(reefLocation.pose2d),
        Commands.parallel(
            alignToPose(reefLocation.pose2d),
            elevator.l3().alongWith(wrist.l3()).onlyIf { isNearTargetPose }
        ),
        gripper.outtakeBySensor(),
        Commands.runOnce({ robotState = RobotStates.Idle })
    )

fun placeL2(): Command =
    sequence(
        pathfindToPose(reefLocation.pose2d),
        Commands.parallel(
            alignToPose(reefLocation.pose2d),
            elevator.l2().alongWith(wrist.l2()).onlyIf { isNearTargetPose }
        ),
        gripper.outtakeBySensor(),
        Commands.runOnce({ robotState = RobotStates.Idle })
    )

fun placeL1(): Command =
    sequence(
            //            pathfindToPose(reefLocation.pose2d),
            Commands.parallel(
                alignToPose(reefLocation.pose2d),
                elevator.l1().alongWith(wrist.l1()).onlyIf { isNearTargetPose }
            ),
            gripper.outtakeBySensor(),
            Commands.runOnce({ robotState = RobotStates.Idle })
        )
        .handleInterrupt { robotState = RobotStates.IdleHasCoral }

fun idle(): Command = elevator.min().alongWith(wrist.feeder())

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
            elevator.feeder().alongWith(wrist.feeder()),
            gripper.intakeBySensor(),
            Commands.runOnce({ robotState = RobotStates.IdleHasCoral })
        )
    )
