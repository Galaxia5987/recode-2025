package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.arm.elevator.Elevator
import frc.robot.subsystems.arm.elevator.ElevatorIO
import frc.robot.subsystems.arm.elevator.ElevatorIOReal
import frc.robot.subsystems.arm.elevator.ElevatorIOSim
import frc.robot.subsystems.arm.elevator.LoggedElevatorInputs
import frc.robot.subsystems.climb.Climber
import frc.robot.subsystems.climb.ClimberIO
import frc.robot.subsystems.climb.ClimberIOReal
import frc.robot.subsystems.climb.ClimberIOSim
import frc.robot.subsystems.climb.LoggedClimberInputs
import frc.robot.subsystems.drive.*
import frc.robot.subsystems.drive.ModuleIOs.ModuleIO
import frc.robot.subsystems.drive.ModuleIOs.ModuleIOSim
import frc.robot.subsystems.drive.ModuleIOs.ModuleIOTalonFX
import frc.robot.subsystems.drive.gyroIOs.GyroIO
import frc.robot.subsystems.drive.gyroIOs.GyroIONavX
import frc.robot.subsystems.drive.gyroIOs.GyroIOSim
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.gripper.GripperIO
import frc.robot.subsystems.gripper.GripperIOReal
import frc.robot.subsystems.gripper.GripperIOSim
import frc.robot.subsystems.gripper.LoggedGripperInputs
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.extender.ExtenderIO
import frc.robot.subsystems.intake.extender.ExtenderIOReal
import frc.robot.subsystems.intake.extender.ExtenderIOSim
import frc.robot.subsystems.intake.extender.LoggedExtenderInputs
import frc.robot.subsystems.intake.into.LoggedRollerInputs
import frc.robot.subsystems.intake.into.Roller
import frc.robot.subsystems.intake.into.RollerIO
import frc.robot.subsystems.intake.into.RollerIOReal
import frc.robot.subsystems.intake.into.RollerIOSim
import frc.robot.subsystems.vision.Vision
import frc.robot.subsystems.vision.VisionConstants
import frc.robot.subsystems.vision.VisionIOPhotonVision
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim
import frc.robot.subsystems.wrist.LoggedWristInputs
import frc.robot.subsystems.wrist.Wrist
import frc.robot.subsystems.wrist.WristIO
import frc.robot.subsystems.wrist.WristIOReal
import frc.robot.subsystems.wrist.WristIOSim
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

val roller =
    Roller(
        when (CURRENT_MODE) {
            Mode.REAL -> RollerIOReal()
            Mode.SIM -> RollerIOSim()
            Mode.REPLAY ->
                object : RollerIO {
                    override val inputs: LoggedRollerInputs =
                        LoggedRollerInputs()
                }
        }
    )

val climber =
    Climber(
        when (CURRENT_MODE) {
            Mode.REAL -> ClimberIOReal()
            Mode.SIM -> ClimberIOSim()
            Mode.REPLAY ->
                object : ClimberIO {
                    override var inputs = LoggedClimberInputs()
                }
        }
    )

val extender: Extender =
    when (CURRENT_MODE) {
        Mode.REAL -> Extender(ExtenderIOReal())
        Mode.SIM -> Extender(ExtenderIOSim())
        else ->
            Extender(
                object : ExtenderIO {
                    override val inputs = LoggedExtenderInputs()
                }
            )
    }
val elevator: Elevator =
    Elevator(
        when (CURRENT_MODE) {
            Mode.REAL -> ElevatorIOReal()
            Mode.SIM -> ElevatorIOSim()
            Mode.REPLAY ->
                object : ElevatorIO {
                    override val inputs = LoggedElevatorInputs()
                }
        }
    )

val wrist: Wrist =
    Wrist(
        when (CURRENT_MODE) {
            Mode.REAL -> WristIOReal()
            Mode.SIM -> WristIOSim()
            else ->
                object : WristIO {
                    override val inputs = LoggedWristInputs()
                }
        }
    )
val gripper: Gripper =
    Gripper(
        when (CURRENT_MODE) {
            Mode.REAL -> GripperIOReal()
            Mode.SIM -> GripperIOSim()
            else ->
                object : GripperIO {
                    override val inputs = LoggedGripperInputs()
                }
        }
    )
