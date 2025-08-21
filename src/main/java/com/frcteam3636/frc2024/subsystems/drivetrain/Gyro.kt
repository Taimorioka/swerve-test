package com.frcteam3636.frc2024.subsystems.drivetrain

import com.frcteam3636.frc2024.Robot
import com.frcteam3636.frc2024.utils.swerve.PerCorner
import com.frcteam3636.frc2024.utils.swerve.translation2dPerSecond
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation2d
import org.littletonrobotics.junction.Logger
import kotlin.math.sign

interface Gyro {
    /**
     * The current rotation of the robot.
     * This can be set to a different value to change the gyro's offset.
     */
    var rotation: Rotation3d

    /** Whether the gyro is connected. */
    val connected: Boolean

    fun periodic() {}
}

class GyroNavX(private val ahrs: AHRS) : Gyro {

    private var offset: Rotation3d = Rotation3d()

    init {
        Logger.recordOutput("Gyro/Offset", offset)
    }

    override var rotation: Rotation3d
        get() = offset + ahrs.rotation3d
        set(goal) {
            offset = goal - ahrs.rotation3d
            Logger.recordOutput("Gyro/Offset", offset)
        }

    override val connected
        get() = ahrs.isConnected
}

class GyroSim(private val modules: PerCorner<SwerveModule>) : Gyro {
    override var rotation = Rotation3d()
    override val connected = true

    override fun periodic() {
        // Calculate the average translation velocity of each module
        val moduleVelocities = modules.map { it.state.translation2dPerSecond }
        val translationVelocity = moduleVelocities.reduce(Translation2d::plus) / moduleVelocities.size.toDouble()

        // Use the front left module's rotational velocity to calculate the yaw velocity
        val rotationalVelocities = moduleVelocities.map { it - translationVelocity }
        val yawVelocity =
            sign(rotationalVelocities.frontLeft.y) * rotationalVelocities.frontLeft.norm /
                    Drivetrain.Constants.MODULE_POSITIONS.frontLeft.translation.norm

        rotation += Rotation3d(0.0, 0.0, yawVelocity) * Robot.period
    }
}
