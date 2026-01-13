package org.team100.lib.trajectory.constraint;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.trajectory.path.PathSE2Point;
import org.team100.lib.tuning.Mutable;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Mecanum drive has a diamond-shaped velocity envelope. If the x and y
 * directions responded the same (they don't), it would be a square.
 * 
 * This ignores the interaction with rotation.
 */
public class DiamondConstraint implements TimingConstraint {
    /** Max velocity ahead */
    private final Mutable m_maxVelocityX;
    /** Max velocity to the side */
    private final Mutable m_maxVelocityY;
    private final Mutable m_maxAccel;

    /**
     * @param parent log
     * @param maxVX  max velocity straight ahead, typically higher
     * @param maxVY  max velocity sideways, typically lower
     * @param maxA   accel
     */
    public DiamondConstraint(LoggerFactory parent, double maxVX, double maxVY, double maxA) {
        LoggerFactory log = parent.type(this);
        m_maxVelocityX = new Mutable(log, "maxVX", maxVX);
        m_maxVelocityY = new Mutable(log, "maxVY", maxVY);
        m_maxAccel = new Mutable(log, "maxA", maxA);
    }

    @Override
    public double maxV(PathSE2Point point) {
        Rotation2d course = point.waypoint().course().toRotation();
        Rotation2d heading = point.waypoint().pose().getRotation();
        Rotation2d strafe = course.minus(heading);
        // a rhombus is a superellipse with exponent 1
        // https://en.wikipedia.org/wiki/Superellipse
        double a = m_maxVelocityX.getAsDouble();
        double b = m_maxVelocityY.getAsDouble();
        return 1 / (Math.abs(strafe.getCos() / a) + Math.abs(strafe.getSin() / b));
    }

    @Override
    public double maxAccel(PathSE2Point point, double velocityM_S) {
        return m_maxAccel.getAsDouble();
    }

    @Override
    public double maxDecel(PathSE2Point point, double velocity) {
        return -m_maxAccel.getAsDouble();
    }

}
