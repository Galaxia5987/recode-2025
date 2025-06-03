package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.arm.elevator.Elevator
import frc.robot.subsystems.arm.elevator.ElevatorIO
import frc.robot.subsystems.arm.elevator.ElevatorIOReal
import frc.robot.subsystems.arm.elevator.ElevatorIOSim
import frc.robot.subsystems.arm.elevator.LoggedElevatorInputs
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.drive.ModuleIOs.ModuleIO
import frc.robot.subsystems.drive.ModuleIOs.ModuleIOSim
import frc.robot.subsystems.drive.ModuleIOs.ModuleIOTalonFX
import frc.robot.subsystems.drive.gyroIOs.GyroIO
import frc.robot.subsystems.drive.gyroIOs.GyroIONavX
import frc.robot.subsystems.drive.gyroIOs.GyroIOSim
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.subsystems.vision.VisionIOPhotonVision
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation

val driveSimulation: SwerveDriveSimulation? =
    if (CURRENT_MODE != Mode.REPLAY)
        SwerveDriveSimulation(
            Drive.mapleSimConfig,
            Pose2d(3.0, 3.0, Rotation2d())
        )
            .apply {
                SimulatedArena.getInstance().addDriveTrainSimulation(this)
            }
    else null

private val driveModuleIOs =
    arrayOf(
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight
    )
        .mapIndexed { index, module ->
            when (CURRENT_MODE) {
                Mode.REAL -> ModuleIOTalonFX(module)
                Mode.SIM -> ModuleIOSim(driveSimulation!!.modules[index])
                Mode.REPLAY -> object : ModuleIO {}
            }
        }
        .toTypedArray()

private val gyroIO =
    when (CURRENT_MODE) {
        Mode.REAL -> GyroIONavX()
        Mode.SIM ->
            GyroIOSim(
                driveSimulation?.gyroSimulation
                    ?: throw Exception("Gyro simulation is null")
            )

        else -> object : GyroIO {}
    }

val drive =
    Drive(
        gyroIO,
        driveModuleIOs,
        driveSimulation?.let { it::setSimulationWorldPose } ?: { _: Pose2d -> }
    )

private val visionIOs =
    when (CURRENT_MODE) {
        Mode.REAL ->
            VisionConstants.OVNameToTransform.map {
                VisionIOPhotonVision(it.key, it.value)
            }

        Mode.SIM ->
            VisionConstants.OVNameToTransform.map {
                VisionIOPhotonVisionSim(
                    it.key,
                    it.value,
                    driveSimulation!!::getSimulatedDriveTrainPose
                )
            }

        Mode.REPLAY -> emptyList()
    }.toTypedArray()

val vision = Vision(drive, *visionIOs)

val elevator: Elevator =
    when (CURRENT_MODE) {
        Mode.REAL -> Elevator(ElevatorIOReal())
        Mode.SIM -> Elevator(ElevatorIOSim())
        Mode.REPLAY -> Elevator(object : ElevatorIO {
            override val inputs = LoggedElevatorInputs()
            override fun isFloored(): Boolean = false
        })
    }
