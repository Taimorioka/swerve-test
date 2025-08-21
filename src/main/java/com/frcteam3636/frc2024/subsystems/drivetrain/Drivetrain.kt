package com.frcteam3636.frc2024.subsystems.drivetrain

import com.frcteam3636.frc2024.CTREMotorControllerId
import com.frcteam3636.frc2024.REVMotorControllerId
import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain.Constants.BRAKE_POSITION
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain.Constants.DEFAULT_PATHING_CONSTRAINTS
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain.Constants.FREE_SPEED
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain.Constants.JOYSTICK_DEADBAND
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain.Constants.ROTATION_SENSITIVITY
import com.frcteam3636.frc2024.subsystems.drivetrain.Drivetrain.Constants.TRANSLATION_SENSITIVITY
import com.frcteam3636.frc2024.utils.ElasticWidgets
import com.frcteam3636.frc2024.utils.math.*
import com.frcteam3636.frc2024.utils.swerve.PerCorner
import com.frcteam3636.frc2024.utils.swerve.cornerStatesToChassisSpeeds
import com.frcteam3636.frc2024.utils.swerve.toCornerSwerveModuleStates
import com.frcteam3636.frc2024.utils.translation2d
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.littletonrobotics.junction.Logger
import java.util.*
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI
import kotlin.math.abs

object Drivetrain: Subsystem {

    //start inputs
    private val gyro = GyroPigeon(Pigeon2(CTREDeviceId.PigeonGyro))
    private val fr = SwerveModule(0,0,0)
} 