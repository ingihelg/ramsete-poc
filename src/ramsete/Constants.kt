package ramsete

/**
 * A simple location for storing all the "magic-number" like things used in the project.
 */
object Constants {
    const val kWheelBase = 2.5
    const val kAutoB: Double = 15.0 // must be greater than 0.0. increases the correction
    const val kAutoZeta: Double = 0.9 // must be between 0.0 and 1.0. increases the dampening, I think

    const val kV: Double = 2.0 // max velocity
    const val kA: Double = 2.3 // max acceleration
    const val kJ: Double = 60.0 // max jerk
}