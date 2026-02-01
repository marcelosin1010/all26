package org.team100.lib.localization;

import java.util.Objects;

import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveDriveKinematics100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleDeltas;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;

class SwerveState implements Interpolatable<SwerveState> {
    private final SwerveDriveKinematics100 m_kinematics;
    private final ModelSE2 m_state;
    private final SwerveModulePositions m_wheelPositions;

    /**
     * @param kinematics Passed here for convenience.
     * @param state      Position and velocity.
     * @param positions  Current encoder readings. Makes a copy.
     */
    SwerveState(
            SwerveDriveKinematics100 kinematics,
            ModelSE2 state,
            SwerveModulePositions positions) {
        m_kinematics = kinematics;
        m_state = state;
        // this copy is important, don't keep the passed one.
        m_wheelPositions = new SwerveModulePositions(positions);
    }

    /**
     * Return the "interpolated" record. This object is assumed to be the starting
     * position, or lower bound.
     * 
     * Interpolates the wheel positions.
     * Integrates wheel positions to find the interpolated pose.
     * Interpolates the velocity.
     *
     * @param endValue The upper bound, or end.
     * @param t        How far between the lower and upper bound we are. This should
     *                 be bounded in [0, 1].
     * @return The interpolated value.
     */
    @Override
    public SwerveState interpolate(SwerveState endValue, double t) {
        if (t < 0) {
            return this;
        }
        if (t >= 1) {
            return endValue;
        }
        // Find the new wheel distances.
        SwerveModulePositions wheelLerp = new SwerveModulePositions(
                m_wheelPositions.frontLeft().interpolate(endValue.m_wheelPositions.frontLeft(), t),
                m_wheelPositions.frontRight().interpolate(endValue.m_wheelPositions.frontRight(), t),
                m_wheelPositions.rearLeft().interpolate(endValue.m_wheelPositions.rearLeft(), t),
                m_wheelPositions.rearRight().interpolate(endValue.m_wheelPositions.rearRight(), t));

        // Create a twist to represent the change based on the interpolated sensor
        // inputs.
        Twist2d twist = m_kinematics.toTwist2d(
                SwerveModuleDeltas.modulePositionDelta(m_wheelPositions, wheelLerp));
        Pose2d pose = m_state.pose().exp(twist);

        // these lerps are wrong but maybe close enough
        VelocitySE2 startVelocity = m_state.velocity();
        VelocitySE2 endVelocity = endValue.m_state.velocity();
        VelocitySE2 velocity = startVelocity.plus(endVelocity.minus(startVelocity).times(t));

        ModelSE2 newState = new ModelSE2(pose, velocity);
        return new SwerveState(m_kinematics, newState, wheelLerp);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof SwerveState)) {
            return false;
        }
        SwerveState rec = (SwerveState) obj;
        return Objects.equals(m_wheelPositions, rec.m_wheelPositions)
                && Objects.equals(m_state, rec.m_state);
    }

    public ModelSE2 state() {
        return m_state;
    }

    public SwerveModulePositions positions() {
        return m_wheelPositions;
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_wheelPositions, m_state);
    }

    @Override
    public String toString() {
        return "InterpolationRecord [m_state=" + m_state
                + ", m_wheelPositions=" + m_wheelPositions + "]";
    }

}