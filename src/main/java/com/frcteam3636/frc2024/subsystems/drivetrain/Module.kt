package com.frcteam3636.frc2024.subsystems.drivetrain

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.frcteam3636.frc2024.*
import com.frcteam3636.frc2024.utils.math.*
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkBase.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.Logger
import kotlin.math.roundToInt

interface SwerveModule {
    val state: SwerveModuleState
    val desiredState: SwerveModuleState
    val position: SwerveModulePosition

    fun periodic() {}
    fun characterize(voltage: Voltage)
}

internal val WHEEL_RADIUS = 1.5.inches
internal val WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * TAU

internal val NEO_FREE_SPEED = 5676.rpm 

private const val DRIVING_MOTOR_PINION_TEETH = 14
internal const val DRIVING_GEAR_RATIO_TALON = 1.0 / 3.56
const val DRIVING_GEAR_RATIO = (45.0 * 22.0) / (DRIVING_MOTOR_PINION_TEETH * 15.0)

internal val NEO_DRIVING_FREE_SPEED = NEO_FREE_SPEED.toLinear(WHEEL_CIRCUMFERENCE) / DRIVING_GEAR_RATIO

internal val DRIVING_PID_GAINS_TALON: PIDGains = PIDGains(.19426, 0.0)
internal val DRIVING_PID_GAINS_NEO: PIDGains = PIDGains(0.04, 0.0, 0.0)
internal val DRIVING_FF_GAINS_TALON: MotorFFGains = MotorFFGains(0.22852, 0.1256, 0.022584)
internal val DRIVING_FF_GAINS_NEO: MotorFFGains =
    MotorFFGains(0.0, 1 / NEO_DRIVING_FREE_SPEED.inMetersPerSecond(), 0.0)

internal val TURNING_PID_GAINS: PIDGains = PIDGains(1.7, 0.0, 0.125)
internal val DRIVING_CURRENT_LIMIT = 37.amps
internal val TURNING_CURRENT_LIMIT = 20.amps 

class SwerveModule (driveMotorID: CTREDeviceId, turnMotorID: REVMotorControllerId, chassisAngle: Rotation2d): SwerveModule {

    private val moduleName = "Module/${driveMotorID.name.split('/').last()}"

    private val turnMotor = SparkMax(turnMotorID, SparkLowLevel.MotorType.kBrushless).apply {
        configure (
            SparkMaxConfig().apply {
                idleMode(IdleMode.kBrake)
                smartCurrentLimit(TURNING_CURRENT_LIMIT.inAmps().roundToInt())

                absoluteEncoder.apply {
                    inverted(true)
                    positionConversionFactor(TAU)
                    velocityConversionFactor(TAU / 60)
                }

                closedLoop.apply {
                    pid(TURNING_PID_GAINS.p, TURNING_PID_GAINS.i, TURNING_PID_GAINS.d)
                    feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
                    positionWrappingEnabled(true)
                    positionWrappingMinInput(0.0)
                    positionWrappingMaxInput(TAU)
                }
            }, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        )
    }

    private val turningEncoder = turnMotor.getAbsoluteEncoder()
    private val turningPIDController = turnMotor.closedLoopController 

    private val driveMotor = TalonFX(driveMotorID).apply {
        configurator.apply(
            TalonFXConfiguration().apply {
                Slot0.apply {
                    kP = DRIVING_PID_GAINS_TALON.p
                    kI = DRIVING_PID_GAINS_TALON.i
                    kD = DRIVING_PID_GAINS_TALON.d
                    kV = DRIVING_FF_GAINS_TALON.v
                    kS = DRIVING_FF_GAINS_TALON.s
                }

                CurrentLimits.apply {
                    SupplyCurrentLimit = DRIVING_CURRENT_LIMIT.inAmps()
                    SupplyCurrentLimitEnable = true
                }
            }
        )
    }

    private val velocityControl = VelocityVoltage(0.0).apply {
        EnableFOC = true
    }
    
    private val voltageControl = VoltageOut(0.0).apply {
        EnableFOC = true
    }

    override val state: SwerveModuleState
        get() = SwerveModuleState(
                driveMotor.velocity.value * WHEEL_RADIUS.inMeters() * DRIVING_GEAR_RATIO_TALON,
                Rotation2d.fromRadians(turningEncoder.position) + chassisAngle
            ).also {
                Logger.recordOutput("$moduleName/ActualSpeed", it.speedMetersPerSecond)
                Logger.recordOutput("$moduleName/ActualAngle", it.angle.degrees)
            }

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
                driveMotor.position.value * WHEEL_RADIUS.inMeters() * DRIVING_GEAR_RATIO_TALON, 
                Rotation2d.fromRadians(turningEncoder.position) + chassisAngle
            )

    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, -chassisAngle)
        set(value) {

            val correctedState = SwerveModuleState(value.speedMetersPerSecond, value.angle - chassisAngle)
            val optimizedState = SwerveModuleState.optimize(
                correctedState,
                Rotation2d.fromRadians(turningEncoder.position)
            )
            val wheelAngularVelocityRPS = optimizedState.speedMetersPerSecond / (WHEEL_RADIUS.inMeters() * TAU * DRIVING_GEAR_RATIO_TALON)

            Logger.recordOutput("$moduleName/DesiredSpeed", value.speedMetersPerSecond)
            Logger.recordOutput("$moduleName/DesiredAngle", value.angle.degrees)
            Logger.recordOutput("$moduleName/OptimizedSpeed", optimizedState.speedMetersPerSecond)
            Logger.recordOutput("$moduleName/OptimizedAngle", optimizedState.angle.degrees)
            Logger.recordOutput("$moduleName/WheelRPS", wheelAngularVelocityRPS)

            driveMotor.setControl(velocityControl.withVelocity(wheelAngularVelocityRPS))
            turningPIDController.setReference(
                optimizedState.angle.radians, SparkBase.ControlType.kPosition 
            )

            field = value
        }

    override fun characterize(voltage: Voltage) {
        Logger.recordOutput("$moduleName/CharacterizeVoltage", voltage.inVolts())

        driveMotor.setControl(voltageControl.withOutput(voltage.inVolts()))
        turningPIDController.setReference(-chassisAngle.radians, SparkBase.ControlType.kPosition)
    }

    override fun periodic() {
        Logger.recordOutput("$moduleName/DriveCurrent", driveMotor.supplyCurrent.value)
        Logger.recordOutput("$moduleName/TurnCurrent", turnMotor.outputCurrent)
        Logger.recordOutput("$moduleName/TurnPosition", turningEncoder.position)
        Logger.recordOutput("$moduleName/DrivePosition", driveMotor.position.value)
        Logger.recordOutput("$moduleName/DriveVelocity", driveMotor.velocity.value)
    }
}