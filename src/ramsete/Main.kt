package ramsete

import jaci.pathfinder.Waypoint

import java.util.ArrayList
import jaci.pathfinder.Pathfinder
import jaci.pathfinder.Trajectory


object Main {
    lateinit var follower: RamseteFollower
    private var robotPos = ArrayList<Odometry>()

    @JvmStatic
    fun main(args: Array<String>) {
        val waypoints =
            arrayOf(
                Waypoint(0.0, 0.0, 0.0),
                Waypoint(4.0, 0.0, 0.0),
                Waypoint(8.0, 4.0, Math.toRadians(90.0))
            )

        val config =
            Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0)
        val traj = Pathfinder.generate(waypoints, config)

        follower = RamseteFollower(
            wheelBase = Constants.kWheelBase,
            path = traj,
            b = Constants.kAutoB,
            zeta = Constants.kAutoZeta
        )
        robotPos.add(follower.getStartOdometry())
        var odometryIdx = 0
        var driveSignal: DriveSignal

        while (!follower.isFinished()) {
            val current = robotPos[odometryIdx]
            follower.setOdometryTo(current)
            driveSignal = follower.getNextDriveSignal()
            val w = (-driveSignal.left + driveSignal.right) / Constants.kWheelBase
            val v = (driveSignal.left + driveSignal.right) / 2
            val dt = .02
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

        println("That's all folks!")
    }
}