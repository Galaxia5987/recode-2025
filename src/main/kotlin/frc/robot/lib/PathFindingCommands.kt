package frc.robot.lib

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RadiansPerSecondPerSecond
import frc.robot.subsystems.drive.TunerConstants.kPathFindAcceleration
import frc.robot.subsystems.drive.TunerConstants.kPathFindAngularAcceleration
import frc.robot.subsystems.drive.TunerConstants.kPathFindOmegaVelocity
import frc.robot.subsystems.drive.TunerConstants.kSpeedAt12Volts


var PATH_CONSTRAINTS: PathConstraints? = PathConstraints(
    kSpeedAt12Volts,
    kPathFindAcceleration,
    kPathFindOmegaVelocity,
    kPathFindAngularAcceleration)

fun pathfindToPose(pose: Pose2d) = AutoBuilder.pathfindToPoseFlipped(pose,PATH_CONSTRAINTS)