package com.frcteam3636.frc2024.subsystems.drivetrain

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TorqueCurrentConfigs
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.frcteam3636.frc2024.*
import com.frcteam3636.frc2024.utils.math.*
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.SparkAbsoluteEncoder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import kotlin.math.roundToInt

interface SwerveModule {
    val state: SwerveModuleState
    val desiredState: SwerveModuleState
    val position: SwerveModulePosition

    fun periodic() {}
    fun characterize(voltage: Voltage)
}

class SwerveModule (driveMotorID: CTREDeviceId, turnMotorID: REVMotorControllerId, chassisAngle: Rotation2d): SwerveModule {

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
    private val turningPIDController = turnMotor.closedloopController

    private val driveMotor = TalonFX(driveMotorID).apply {
        configurator.apply(
            TalonFXConfiguration().apply {
                Slot0.apply {
                    pidGains = DRIVING_PID_GAINS_TALON
                    motorFFGains = DRIVING_FF_GAINS_TALON
                }

                CurrentLimits.apply {
                    SupplyCurrentLimit = DRIVING_CURRENT_LIMIT.inAmps()
                    SupplyCurrentLimitEnable = true
                }
            }
        )
    }

    override val state: SwerveModuleState
        get() = SwerveModuleState(
                (driveMotor.velocity.value.toLinear(WHEEL_RADIUS) * DRIVING_GEAR_RATIO_TALON).inMetersPerSecond(), 
                Rotation2d.fromRadians(turningEncoder.position) + chassisAngle
            )

    override val position: SwerveModulePosition
        get() = SwerveModulePosition(
                driveMotor.position.value.toLinear(WHEEL_RADIUS) * DRIVING_GEAR_RATIO_TALON, 
                Rotation2d.fromRadians(turningEncoder.position) + chassisAngle
            )

    override var desiredState: SwerveModuleState = SwerveModuleState(0.0, -chassisAngle)
        get() = SwerveModuleState(field.speedMetersPerSecond, field.angle + chassisAngle)
        set(value) {
            val correctedState = SwerveModuleState(value.speedMetersPerSecond, value.angle - chassisAngle)
            correctedState.optimize(
                Rotation2d.fromRadians(turningEncoder.position)
            )
            // driveMotor.velocity = correctedState.speed
            driveMotor.closedLoopController.setReference(correctedState.speed.inMetersPerSecond(), SparkBase.ControlType.kVelocity)
            turningPIDController.setReference(
                corrected.angle.radians, SparkBase.ControlType.kPosition
            )
            field = corrected
        }

    override fun characterize(voltage: Voltage) {
        driveMotor.setVoltage(voltage)
        turningPIDController.setReference(-chassisAngle.radians, SparkBase.ControlType.kPosition)
    }
    
}