package ramsete

/**
 * A data class which contains the left motor setting,
 * right motor setting, and whether or not the brake mode is
 * enabled.
 */
data class DriveSignal(val left: Double, val right: Double, val brake: Boolean = false)