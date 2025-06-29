package frc.robot.subsystems

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
import frc.robot.drive
import frc.robot.elevator
import frc.robot.extender
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.drive.Drive
import frc.robot.wrist

private val swerveModulePose: Array<Translation2d> =
    Drive.getModuleTranslations()
private val kWheelRadius = Meters.of(0.0508)
private val secondElevatorStageHeight = Meters.of(0.7)

class Visualizer {
    val defaultPose = getPose3d()
    private fun getSwerveModulePoseTurn(
        moduleX: Double,
        moduleY: Double,
        moduleYaw: Angle
    ): Pose3d {
        return Pose3d(
            Translation3d(moduleX, moduleY, kWheelRadius.`in`(Meters)),
            getRotation3d(yaw = moduleYaw)
        )
    }

    private fun getSwerveModulePoseDrive(
        moduleX: Double,
        moduleY: Double,
        moduleYaw: Angle,
        modulePitch: Angle
    ): Pose3d {

        return Pose3d(
            Translation3d(moduleX, moduleY, kWheelRadius.`in`(Meters)),
            getRotation3d(yaw = moduleYaw, pitch = modulePitch)
        )
    }

    private fun getAllSwerveModulePoseTurn(): Array<Pose3d> {
        val swervePosesTurn: Array<Pose3d> =
            arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())
        for (i in 0..3) {
            swervePosesTurn[i] =
                getSwerveModulePoseTurn(
                    swerveModulePose[i].x,
                    swerveModulePose[i].y,
                    drive.SwerveTurnAngle[i]
                )
        }
        return swervePosesTurn
    }

    private fun getAllSwerveModulePoseDrive(): Array<Pose3d> {
        val swervePosesDrive: Array<Pose3d> =
            arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())

        for (i in 0..3) {
            swervePosesDrive[i] =
                getSwerveModulePoseDrive(
                    swerveModulePose[i].x,
                    swerveModulePose[i].y,
                    drive.SwerveTurnAngle[i],
                    drive.SwerveDriveAngle[i]
                )
        }
        return swervePosesDrive
    }

    val EXTENDER_POSITION = getTranslation3d(0.0)
    val extenderPose
        get() = getPose3d(EXTENDER_POSITION + getTranslation3d(extender.length))

    val ELEVATOR_FIRST_STAGE_POSITION = getTranslation3d(0.0)
    val ELEVATOR_SECOND_STAGE_POSITION = getTranslation3d(0.0)

    val elevatorPoseFirstStage
        get() =
            getPose3d(
                ELEVATOR_FIRST_STAGE_POSITION +
                    getTranslation3d(
                        z =
                            if (
                                elevator.height - secondElevatorStageHeight >
                                    Meters.zero()
                            ) {
                                (secondElevatorStageHeight)
                            } else {
                                elevator.height
                            }
                    )
            )

    val elevatorPoseSecondStage
        get() =
            getPose3d(
                ELEVATOR_SECOND_STAGE_POSITION +
                    getTranslation3d(z = elevator.height)
            )

    val WRIST_POSITION = getTranslation3d(z = 0.9)
    val WRIST_ROTATION = getRotation3d(0.0)
    val wristPos
        get() =
            getPose3d(
                WRIST_POSITION + getTranslation3d(z = elevator.height),
                getRotation3d(pitch = wrist.inputs.angle)
            )

    fun ROBOT_POSE(): Array<Pose3d> {
        val swervePosesTurn = getAllSwerveModulePoseTurn()
        val swervePosesDrive = getAllSwerveModulePoseDrive()
        return arrayOf(
            swervePosesTurn[0], // 1
            swervePosesDrive[0], // 2
            swervePosesTurn[1], // 3
            swervePosesDrive[1], // 4
            swervePosesTurn[2], // 5
            swervePosesDrive[2], // 6
            swervePosesTurn[3], // 7
            swervePosesDrive[3], // 8
            extenderPose, // 9
            extenderPose, // 10
            elevatorPoseFirstStage, // 11
            elevatorPoseSecondStage, // 12
            wristPos,
            defaultPose,
            defaultPose,
            defaultPose
        )
    }
}
