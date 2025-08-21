package com.frcteam3636.frc2024.subsystems.drivetrain

import com.frcteam3636.frc2024.utils.swerve.PerCorner
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

abstract class DrivetrainIO {
    protected abstract val gyro: Gyro
    abstract val modules: PerCorner<out SwerveModule>

    class Inputs : LoggableInputs {
        var gyroRotation = Rotation3d()
        var measuredStates = PerCorner.generate { SwerveModuleState() }
        var measuredPositions = PerCorner.generate { SwerveModulePosition() }

        var gyroConnected = true

        override fun fromLog(table: LogTable?) {
            gyroRotation = table?.get("Gyro Rotation", gyroRotation)!![0]
            measuredStates =
                PerCorner.fromConventionalArray(
                    table.get("Measured States", *measuredStates.toTypedArray())
                )
            measuredPositions =
                PerCorner.fromConventionalArray(
                    table.get("Measured Positions", *measuredPositions.toTypedArray())
                )
            gyroConnected = table.get("Gyro Connected", false)
        }

        override fun toLog(table: LogTable?) {
            table?.put("Gyro Rotation", gyroRotation)
            table?.put("Measured States", *measuredStates.toTypedArray())
            table?.put("Measured Positions", *measuredPositions.toTypedArray())
            table?.put("Gyro Connected", gyroConnected)
        }
    }

    fun updateInputs(inputs: Inputs) {
        gyro.periodic()
        modules.forEach(SwerveModule::periodic)

        inputs.gyroRotation = gyro.rotation
        inputs.measuredStates = modules.map { it.state }
        inputs.measuredPositions = modules.map { it.position }
        inputs.gyroConnected = gyro.connected
    }

    fun setGyro(rotation: Rotation3d) {
        gyro.rotation = rotation
    }

    var desiredStates: PerCorner<SwerveModuleState>
        get() = modules.map { it.desiredState }
        set(value) {
            modules.zip(value).forEach { (module, state) -> module.desiredState = state }
        }
}

/** Drivetrain I/O layer that uses real swerve modules along with a NavX gyro. */
class DrivetrainIOReal(override val modules: PerCorner<out SwerveModule>) : DrivetrainIO() {
    override val gyro = GyroNavX(AHRS())

    companion object {
        fun fromKrakenSwerve() =
            DrivetrainIOReal(
                Drivetrain.Constants.MODULE_POSITIONS.zip(Drivetrain.Constants.KRAKEN_MODULE_CAN_IDS)
                    .map { (position, ids) ->
                        val (driveId, turnId) = ids
                        MAXSwerveModule(
                            DrivingTalon(driveId),
                            turnId,
                            position.rotation
                        )
                    })
    }
}

/** Drivetrain I/O layer that uses simulated swerve modules along with a simulated gyro with an angle based off their movement. */
class DrivetrainIOSim : DrivetrainIO() {
    override val modules = PerCorner.generate { SimSwerveModule() }
    override val gyro = GyroSim(modules.map { it })
}
