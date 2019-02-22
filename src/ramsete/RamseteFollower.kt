package ramsete

import jaci.pathfinder.Trajectory

/**
 * A trajectory tracking approach that actively corrects longitudinal, lateral,
 * and angular error by asymptotically stabilizing the error at zero.
 *
 * The link to the paper is here: https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
 *
 * For FRC implementation, I think the best approach is to use WPILib notifier on the same period
 * as the Robot period.
 *
 * Thanks to many members on the FRC Discord for constantly talking about this and making helpful
 * resources for it available.
 */
class RamseteFollower(
    val b: Double, // must be greater than 0. increases correction
    val zeta: Double, // must be between 0 and 1
    val wheelBase: Double, // size of wheel base (duh)
    val path: Trajectory // the Pathfinder path to follow
) {
    private var segmentIdx = 0 // the segmentIdx for tracking our progress in the path
    private var odo =
        Odometry(0.0, 0.0, 0.0) // the Odometry data class we will use for pose estimation

    init {
        // Logging to indicate that we have created a new RamseteFollower
        println("[[ Initializing Ramsete Follower! ]]")
    }

    /**
     * Determine what the next DriveSignal should be, based on a variety of calculations.
     */
    fun getNextDriveSignal(): DriveSignal {
        var left = 0.0
        var right = 0.0

        // If we are finished following the path, then DriveSignal should be "zeroed"
        if (isFinished()) {
            return DriveSignal(left, right)
        }

        // Some logging so that we know roughly how far along in the path we are
        System.out.println("Seg " + segmentIdx + " of " + (path.length() - 1))

        val current = path.get(segmentIdx) // look at segment of the path
        val w_d = getDesiredAngularVelocity() // to determine wanted rate of change of heading

        val linearVelocity = getVelocity(current.x, current.y, current.heading, current.velocity, w_d)
        val angularVelocity = getAngularVelocity(current.x, current.y, current.heading, current.velocity, w_d)

        // Some more logging to confirm that angular and linear velocity make sense, more or less
        println("Velocity: $linearVelocity Angular Velocity: $angularVelocity")

        // Calculate desired outputs
        left = -1 * wheelBase * angularVelocity / 2 +
                linearVelocity // do math to convert angular velocity + linear velocity
        right = wheelBase * angularVelocity / 2 + linearVelocity // into left and right wheel speeds (FperS)

        // Log the left and right outputs to confirm one last time that they make sense
        println("Left: $left Right: $right")

        // Increment segment index
        segmentIdx++
        return DriveSignal(left, right)
    }

    /**
     * Calculate the desired angular velocity on the current segment, looking ahead to the next segment.
     */
    private fun getDesiredAngularVelocity(): Double {
        if (segmentIdx < path.length() - 1) {
            val lastTheta = path.get(segmentIdx).heading
            val nextTheta = path.get(segmentIdx + 1).heading
            return (nextTheta - lastTheta) / path.get(segmentIdx).dt
        }
        return 0.0
    }

    /**
     * Calculates linear velocity.
     */
    private fun getVelocity(x_d: Double, y_d: Double, theta_d: Double, v_d: Double, w_d: Double): Double {
        val k = calcK(v_d, w_d)
        val v =
            v_d * Math.cos(theta_d - odo.theta) + k * (Math.cos(odo.theta) * (x_d - odo.x) + Math.sin(odo.theta) * (y_d - odo.y))
        return v
    }

    /**
     * Calculates angular velocity.
     */
    private fun getAngularVelocity(x_d: Double, y_d: Double, theta_d: Double, v_d: Double, w_d: Double): Double {
        val k = calcK(v_d, w_d)
        System.out.println("Theta: " + odo.theta)
        val err = theta_d - odo.theta
        val sinErrOverErr = when (err) {
            0.0, 1E-5 -> 1.0 // lim sin(x)/x as x->0
            else -> Math.sin(theta_d - odo.theta) / err
        }
        val v =
            w_d + b * v_d * sinErrOverErr * (Math.cos(odo.theta) * (y_d - odo.y) - Math.sin(odo.theta) * (x_d - odo.x)) + k * err // from eq. 5.12
        return v
    }

    /**
     * Calculate the "special constant". The exact meaning of this constant is hard to explain, so read the paper.
     */
    private fun calcK(v_d: Double, w_d: Double): Double {
        return 2 * zeta * Math.sqrt(Math.pow(w_d, 2.0) + b * Math.pow(v_d, 2.0)) // from eq. 5.12
    }

    /**
     * Set the follower odometry to be a given odometry.
     */
    fun setOdometryTo(desire: Odometry) {
        odo = desire
    }

    /**
     * Get the odometry of the path at its beginning.
     */
    fun getInitialOdometry() = Odometry(path.get(0).x, path.get(0).y, path.get(0).heading)

    /**
     * Determine whether we are done tracking the path.
     */
    fun isFinished() = segmentIdx == path.length()
}