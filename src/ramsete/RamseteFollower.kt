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
    private var index = 0 // the index for tracking our progress in the path
    private var odo =
        Odometry(0.0, 0.0, 0.0) // the Odometry data class we will use for pose estimation

    init {
        println("[[ Initializing Ramsete Follower! ]]")
    }

    private fun calcWsubd(): Double {
        if (index < path.length() - 1) {
            val lastTheta = path.get(index).heading
            val nextTheta = path.get(index + 1).heading
            return (nextTheta - lastTheta) / path.get(index).dt
        }
        return 0.0
    }

    /**
     * Determines what the next DriveSignal should be, based on a variety of calculations.
     */
    fun getNextDriveSignal(): DriveSignal {
        var left = 0.0
        var right = 0.0
        if (isFinished()) {
            return DriveSignal(left, right)
        }
        System.out.println("Seg " + index + " of " + (path.length() - 1))

        val current = path.get(index) // look at segment of the path
        val w_d = calcWsubd() // to determine wanted rate of change of heading

        val linearVelocity = calcVel(current.x, current.y, current.heading, current.velocity, w_d)
        val angularVelocity = calcAngleVel(current.x, current.y, current.heading, current.velocity, w_d)

        println("Velocity: $linearVelocity Angular Velocity: $angularVelocity")

        left = -wheelBase * angularVelocity / 2 + linearVelocity //do math to convert angular velocity + linear velocity
        right = +wheelBase * angularVelocity / 2 + linearVelocity //into left and right wheel speeds (FperS)


        println("Left: $left Right: $right")
        index += 1
        return DriveSignal(left, right)
    }

    fun setOdometryTo(desire: Odometry) {
        odo = desire
    }

    /**
     * Calculates linear velocity.
     */
    private fun calcVel(x_d: Double, y_d: Double, theta_d: Double, v_d: Double, w_d: Double): Double {
        val k = calcK(v_d, w_d)
        return v_d * Math.cos(theta_d - odo.theta) + k * (Math.cos(odo.theta) * (x_d - odo.x) + Math.sin(odo.theta) * (y_d - odo.y))
    }

    /**
     * Calculates angular velocity.
     */
    private fun calcAngleVel(x_d: Double, y_d: Double, theta_d: Double, v_d: Double, w_d: Double): Double {
        val k = calcK(v_d, w_d)
        System.out.println("Theta: " + odo.theta)
        val err = theta_d - odo.theta
        val sinErrOverErr = when (err) {
            0.0, 1E-5 -> 1.0 // lim sin(x)/x as x->0
            else -> Math.sin(theta_d - odo.theta) / err
        }
        return w_d + b * v_d * sinErrOverErr * (Math.cos(odo.theta) * (y_d - odo.y) - Math.sin(odo.theta) * (x_d - odo.x)) + k * err // from eq. 5.12
    }

    private fun calcK(v_d: Double, w_d: Double): Double {
        return 2 * zeta * Math.sqrt(Math.pow(w_d, 2.0) + b * Math.pow(v_d, 2.0)) // from eq. 5.12
    }

    fun getStartOdometry() = Odometry(path.get(0).x, path.get(0).y, path.get(0).heading)

    fun isFinished() = index == path.length()
}