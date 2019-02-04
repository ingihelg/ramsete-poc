package ramsete

/**
 * A data class for pose estimation which contains the robot's x,
 * y, and heading on the field.
 *
 * The positive X-axis is defined as the robot going straight forward towards the center of the field.
 * The positive Y-axis is defined as the robot moving left from its starting point against the back wall.
 * Theta is within the range [-2pi, +2pi] and is defined to be counter-clockwise positive (like unit circle).
 */
data class Odometry(var x: Double, var y: Double, var theta: Double)