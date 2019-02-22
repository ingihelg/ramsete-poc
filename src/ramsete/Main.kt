package ramsete

import jaci.pathfinder.Waypoint

import java.util.ArrayList
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory

object Main {
    lateinit var follower: RamseteFollower
    private var robotPos = ArrayList<Odometry>()

    /**
     * Simulates "snowball"-like error.
     */
    fun fakeError(min: Double, max: Double) = Math.random() * (max - min) + min

    @JvmStatic
    fun main(args: Array<String>) {
        // Create a set of Waypoints to generate out Pathfinder path.
        val waypoints =
            arrayOf(
                Waypoint(0.0, 0.0, 0.0),
                Waypoint(4.0, 0.0, 0.0),
                Waypoint(8.0, 4.0, Math.toRadians(90.0))
            )

        // Some Pathfinder configuration. This is relatively standard stuff, nothing fancy here
        val config =
            Trajectory.Config(
                Trajectory.FitMethod.HERMITE_QUINTIC, // fit method
                Trajectory.Config.SAMPLES_HIGH, // samples
                0.02, // dt
                Constants.kV, // max velocity
                Constants.kA, // max acceleration
                Constants.kJ // max jerk
            )

        // Generate the actual trajectory using our waypoints and Pathfinder configuration
        val traj = Pathfinder.generate(waypoints, config)

        // Create a new RamseteFollower
        follower = RamseteFollower(
            wheelBase = Constants.kWheelBase,
            path = traj,
            b = Constants.kAutoB,
            zeta = Constants.kAutoZeta
        )

        // Begin tracking robot position
        robotPos.add(follower.getStartOdometry())
        var odometryIdx = 0
        var driveSignal: DriveSignal
        val trajDt = traj.get(0).dt

        // Until we are finished following the path, do this
        while (!follower.isFinished()) {
            val current = robotPos[odometryIdx]
            follower.setOdometryTo(current)
            driveSignal = follower.getNextDriveSignal()
            val w = (-driveSignal.left + driveSignal.right) / Constants.kWheelBase
            val v = (driveSignal.left + driveSignal.right) / 2
            val dt = trajDt * fakeError(0.1, 2.0) // TODO have someone confirm this makes sense
            val heading = w * dt
            val pos = v * dt
            val x = pos * Math.cos(current.theta + heading)
            val y = pos * Math.sin(current.theta + heading)

            val newX = current.x + x
            val newY = current.y + y
            val newTheta = current.theta + heading

            println("New X:  " + newX + " New Y:  " + newY + " New Heading:  " + Math.toDegrees(newTheta))
            robotPos.add(Odometry(newX, newY, newTheta))
            odometryIdx++
        }

        // Log to indicate we are done following the path
        println("That's all folks!")
    }
}