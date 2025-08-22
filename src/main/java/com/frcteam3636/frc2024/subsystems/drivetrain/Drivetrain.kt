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
import kotlin.math.pow
import kotlin.math.withSign

object Drivetrain: Subsystem {

    //start inputs
    private val gyro = GyroPigeon(Pigeon2(CTREDeviceId.PigeonGyro))

    private val fl = SwerveModule(
        driveMotorID = CTREDeviceId(3, "FrontLeft/Drive"),
        turnMotorID = REVMotorControllerId(4, "FrontLeft/Turn"),
        chassisAngle = Rotation2d.fromDegrees(45.0)
    )

    private val fr = SwerveModule(
        driveMotorID = CTREDeviceId(1, "FrontRight/Drive"),
        turnMotorID = REVMotorControllerId(2, "FrontRight/Turn"),
        chassisAngle = Rotation2d.fromDegrees(-45.0)
    )

    private val bl = SwerveModule(
        driveMotorID = CTREDeviceId(7, "BackLeft/Drive"),
        turnMotorID = REVMotorControllerId(8, "BackLeft/Turn"),
        chassisAngle = Rotation2d.fromDegrees(135.0)
    )

    private val br = SwerveModule(
        driveMotorID = CTREDeviceId(5, "BackRight/Drive"),
        turnMotorID = REVMotorControllerId(6, "BAckRight/Turn"),
        chassisAngle = Rotation2d.fromDegrees(-135.0)
    )

    val modules: PerCorner<SwerveModule> = PerCorner(fl, fr, bl, br)

    private val kinematics = SwerveDriveKinematics (
        *Constants.MODULE_POSITIONS.map { it.translation }.toTypedArray()
    )

    private val poseEstimator = SwerveDrivePoseEstimator(
        kinematics,
        gyro.rotation2d,
        modules.map { it.position }.toTypedArray(),
        Pose2d()
    )

    var estimatedPose: Pose2d
        get() = poseEstimator.estimatedPosition
        set(value) {
            poseEstimator.resetPosition(
                gyro.rotation2d,
                modules.map { it.position }.toTypedArray(),
                value
            )
        }

    override fun periodic() {

        modules.forEach { it.periodic() }

        estimatedPose = poseEstimator.update(
            gyro.rotation2d,
            modules.map { it.position }.toTypedArray()
        )

        Logger.recordOutput("Drivetrain/Pose", estimatedPose)
        modules.forEachIndexed { index, module ->
            Logger.recordOutput("Drivetrain/Module$index/State", module.state)
            Logger.recordOutput("Drivetrain/Module$index/DesiredState", module.desiredState)
        }
    }

    private var desiredModuleStates: PerCorner<SwerveModuleState>
        get() = modules.map{ it.desiredState }
        set(value) {
            synchronized(this) {
                val stateArr = value.toTypedArray()
                SwerveDriveKinematics.desaturateWheelSpeeds(stateArr, FREE_SPEED)

                modules.zip(PerCorner.fromConventionalArray(stateArr)).forEach {
                    (module, state) -> module.desiredState = state
                }
            }
        }

    private var desiredChassisSpeeds
        get() = kinematics.cornerStatesToChassisSpeeds(desiredModuleStates)
        set(value) {
            val discretized = ChassisSpeeds.discretize(value, Robot.period)
            desiredModuleStates = kinematics.toCornerSwerveModuleStates(discretized)
        }

    private fun isInDeadband(translation: Translation2d) =
        abs(translation.x) < JOYSTICK_DEADBAND && abs(translation.y) < JOYSTICK_DEADBAND

    private fun calculateInputCurve(input: Double): Double {
        val exponent = 1.7
        return input.absoluteValue.pow(exponent).withSign(input)
    }

    private fun drive(translationInput: Translation2d, rotationInput: Double) {
        if ( isInDeadband(translationInput) && isInDeadband(rotationInput)) {
            desiredModuleStates = BRAKE_POSITION
        } else {
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                calculateInputCurve(translationInput.x) * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                calculateInputCurve(translationInput.y) * FREE_SPEED.baseUnitMagnitude() * TRANSLATION_SENSITIVITY,
                rotationInput * PI * ROTATION_SENSITIVITY,
                estimatedPose.rotation
            )
        }
    }

    fun driveWithJoysticks(translationJoystick: Joystick, rotationJoystick: Joystick): Command =
        run { 
            drive(translationJoystick.fieldRelativeTranslation2d, rotationJoystick.y)
        }
    
    fun driveWithController(controller: CommandXboxController): Command =
        run {
            val translationInput = Translation2d(controller.leftX, controller.leftY)
            val rotationInput = Translation2d(controller.rightX)

            drive(translationInput, rotationInput)
        }

    fun zeroGyro(isReversed: Boolean = false, offset: Rotation2d = Rotation2d.kZero()) {
        // Tell the gyro that the robot is facing the other alliance.
        var zeroPos = when (DriverStation.getAlliance().getOrNull()) {
            DriverStation.Alliance.Red -> Rotation2d.fromDegrees(180.0)
            else -> Rotation2d()
        }

        if (isReversed) {
            zeroPos += Rotation2d.fromDegrees(180.0)
        }

        estimatedPose = Pose2d(estimatedPose.translation, zeroPos + offset)
    }

} 