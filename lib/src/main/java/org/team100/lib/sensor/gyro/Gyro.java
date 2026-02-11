package org.team100.lib.sensor.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Three-axis gyro, NWU.
 */
public interface Gyro {

    /**
     * White noise density of the gyro measurement, in rad/sqrt(dt),
     * used for fusing the gyro measurement.
     * 
     * Start with 1e-4. ADIS16448 is noisier, 4e-4. Can be higher, like 5e-3
     * 
     * The returned value must be a constant.
     * 
     * Multiply by sqrt(dt) to get the standard deviation for a single sample.
     * 
     * @see https://stechschulte.net/2023/10/11/imu-specs.html
     */
    double white_noise();

    /**
     * Noise density of the gyro bias (i.e. drift), in rad*sqrt(dt).
     * 
     * Start with 1e-5. ADIS16448 is less, 4e-6. Can be higher, like 5e-4.
     * 
     * The returned value must be a constant.
     * 
     * 
     * 
     * @see https://stechschulte.net/2023/10/11/imu-specs.html
     */
    double bias_noise();

    /**
     * Yaw in radians, NWU, counterclockwise positive.
     * Implementations should extrapolate using the yaw rate,
     * to get the yaw at the current Takt time.
     */
    Rotation2d getYawNWU();

    /**
     * Yaw rate in rad/s, NWU, counterclockwise positive.
     * Not cached, may be inconsistent with the yaw value, and not constant during
     * the cycle.
     */
    double getYawRateNWU();

    /** Pitch in radians, NWU, positive-down. */
    Rotation2d getPitchNWU();

    /** Roll in radians, NWU, positive-right. */
    Rotation2d getRollNWU();

    /** For computing rate. */
    void periodic();

}
