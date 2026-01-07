package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.WaypointSE3;
import org.team100.lib.trajectory.path.PathPointSE3;
import org.team100.lib.trajectory.timing.TimedStateSE3;
import org.team100.lib.trajectory.timing.TimingConstraintSE3;

import edu.wpi.first.math.geometry.Pose3d;

public class TrajectorySE3 {
    private final List<TimedStateSE3> m_points;
    public final List<TimingConstraintSE3> m_constraints;
    private final double m_duration;

    public TrajectorySE3() {
        m_points = new ArrayList<>();
        m_constraints = new ArrayList<>();
        m_duration = 0;
    }

    public TrajectorySE3(
            List<TimedStateSE3> points, List<TimingConstraintSE3> constraints) {
        m_points = points;
        m_constraints = constraints;
        m_duration = m_points.get(m_points.size() - 1).getTimeS();
    }

    public TimedStateSE3 sample(double timeS) {
        // This scans the whole trajectory for every sample, but most of the time
        // is the interpolation; I tried a TreeMap index and it only saved a few
        // nanoseconds per call.
        if (isEmpty())
            throw new IllegalStateException("can't sample an empty trajectory");
        if (timeS >= m_duration) {
            return getLastPoint();
        }
        if (timeS <= 0) {
            return getPoint(0);
        }

        for (int i = 1; i < length(); ++i) {
            final TimedStateSE3 ceil = getPoint(i);
            if (ceil.getTimeS() >= timeS) {
                final TimedStateSE3 floor = getPoint(i - 1);
                double span = ceil.getTimeS() - floor.getTimeS();
                if (Math.abs(span) <= 1e-12) {
                    return ceil;
                }
                double delta_t = timeS - floor.getTimeS();
                return floor.interpolate(ceil, delta_t);
            }
        }
        throw new IllegalStateException("impossible trajectory: " + toString());
    }

    public int length() {
        return m_points.size();
    }

    public boolean isEmpty() {
        return m_points.isEmpty();
    }

    public TimedStateSE3 getPoint(int index) {
        return m_points.get(index);
    }

    public TimedStateSE3 getLastPoint() {
        return m_points.get(length() - 1);
    }

    public double duration() {
        return m_duration;
    }

    public void dump() {
        System.out.println("i, s, t, v, a, k, x, y");
        for (int i = 0; i < length(); ++i) {
            TimedStateSE3 ts = getPoint(i);
            PathPointSE3 pwm = ts.point();
            WaypointSE3 w = pwm.waypoint();
            Pose3d p = w.pose();
            System.out.printf("%d, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
                    i, pwm.getS(), ts.getTimeS(), ts.velocityM_S(), ts.acceleration(),
                    p.getX(), p.getY());
        }
    }

}
