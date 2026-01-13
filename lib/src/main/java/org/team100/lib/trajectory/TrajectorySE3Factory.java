package org.team100.lib.trajectory;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.trajectory.constraint.TimingConstraintSE3;
import org.team100.lib.trajectory.path.PathSE3;
import org.team100.lib.trajectory.path.PathSE3Entry;
import org.team100.lib.trajectory.path.PathSE3Point;
import org.team100.lib.util.Math100;

/**
 * For now this is a copy of the SE2 version.
 */
public class TrajectorySE3Factory {
    public static final boolean DEBUG = false;
    private static final double EPSILON = 1e-6;
    private static final double HIGH_V = 100;
    private static final double HIGH_ACCEL = 1000;

    private final List<TimingConstraintSE3> m_constraints;

    public TrajectorySE3Factory(List<TimingConstraintSE3> constraints) {
        m_constraints = constraints;
    }

    public TrajectorySE3 fromPath(PathSE3 path, double start_vel, double end_vel) {
        double[] distances = distances(path);
        double[] velocities = velocities(path, start_vel, end_vel, distances);
        double[] accels = accels(distances, velocities);
        double[] runningTime = runningTime(distances, velocities, accels);
        List<TrajectorySE3Entry> timedStates = timedStates(path, velocities, accels, runningTime);
        return new TrajectorySE3(timedStates, m_constraints);
    }

    /////////////////////////////////////////////////////////////////////////////////////

    private double[] distances(PathSE3 path) {
        int n = path.length();
        double distances[] = new double[n];
        for (int i0 = 0; i0 < n - 1; ++i0) {
            int i1 = i0 + 1;
            double arclength = path.getEntry(i1).point().distanceCartesian(path.getEntry(i0).point());
            distances[i1] = arclength + distances[i0];
        }
        return distances;
    }

    private double[] velocities(
            PathSE3 path, double start_vel, double end_vel, double[] distances) {
        double velocities[] = new double[path.length()];
        forward(path, start_vel, distances, velocities);
        backward(path, end_vel, distances, velocities);
        if (start_vel > velocities[0]) {
            System.out.printf("WARNING: start velocity %f is higher than constrained velocity %f\n",
                    start_vel, velocities[0]);
        }
        return velocities;
    }

    private double[] accels(double[] distances, double[] velocities) {
        int n = distances.length;
        double[] accels = new double[n];
        for (int i = 0; i < n - 1; ++i) {
            double arcLength = distances[i + 1] - distances[i];
            accels[i] = Math100.accel(velocities[i], velocities[i + 1], arcLength);
        }
        return accels;
    }

    private double[] runningTime(double[] distances, double[] velocities, double[] accels) {
        int n = distances.length;
        double[] runningTime = new double[n];
        for (int i = 1; i < n; ++i) {
            double arcLength = distances[i] - distances[i - 1];
            double dt = dt(velocities[i - 1], velocities[i], arcLength, accels[i - 1]);
            runningTime[i] = runningTime[i - 1] + dt;
        }
        return runningTime;
    }

    private List<TrajectorySE3Entry> timedStates(
            PathSE3 path, double[] velocities, double[] accels, double[] runningTime) {
        int n = path.length();
        List<TrajectorySE3Entry> timedStates = new ArrayList<>(n);
        for (int i = 0; i < n; ++i) {
            PathSE3Entry pe = path.getEntry(i);
            TrajectorySE3Entry te = new TrajectorySE3Entry(pe.parameter(),
                    new TrajectorySE3Point(pe.point(), runningTime[i], velocities[i], accels[i]));
            timedStates.add(te);
        }
        return timedStates;
    }

    private void forward(
            PathSE3 path, double start_vel, double[] distances, double[] velocities) {
        int n = path.length();
        velocities[0] = start_vel;
        for (int i0 = 0; i0 < n - 1; ++i0) {
            int i1 = i0 + 1;
            if (DEBUG)
                System.out.printf("FWD i %d\n", i0);
            double arclength = distances[i1] - distances[i0];
            if (Math.abs(arclength) < EPSILON) {
                if (DEBUG)
                    System.out.printf("i %d zero arc\n", i0);
                // zero-length arcs have the same state at both ends
                velocities[i1] = velocities[i0];
                break;
            }
            // velocity constraint depends only on state
            double maxVelocity = maxVelocity(path.getEntry(i1).point());
            if (DEBUG)
                System.out.printf("maxV i %d %f\n", i1, maxVelocity);
            // start with the maximum velocity
            velocities[i1] = maxVelocity;
            // reduce velocity to fit under the acceleration constraint
            double impliedAccel = Math100.accel(velocities[i0], velocities[i1], arclength);
            double maxAccel = maxAccel(path.getEntry(i0).point(), velocities[i0]);
            if (impliedAccel > maxAccel) {
                velocities[i1] = Math100.v1(velocities[i0], maxAccel, arclength);
                if (DEBUG)
                    System.out.printf("adjust vi+1 %f\n", velocities[i1]);
            }
            if (DEBUG)
                System.out.printf("FWD i %d vi %f vi+1 %f maxA %f impliedA %f\n",
                        i0, velocities[i0], velocities[i1], maxAccel, impliedAccel);
        }
    }

    private void backward(
            PathSE3 path, double end_vel, double[] distances, double[] velocities) {
        int n = path.length();
        velocities[n - 1] = end_vel;
        for (int i0 = n - 2; i0 >= 0; --i0) {
            int i1 = i0 + 1;
            if (DEBUG)
                System.out.printf("BACK i %d\n", i0);
            double arclength = distances[i1] - distances[i0];
            if (Math.abs(arclength) < EPSILON) {
                // already handled this case
                break;
            }

            double maxVelocity = maxVelocity(path.getEntry(i0).point());
            if (DEBUG)
                System.out.printf("maxV i %d %f\n", i0, maxVelocity);

            double impliedAccel = Math100.accel(velocities[i0], velocities[i1], arclength);
            // Apply the decel constraint at the end of the segment since it is feasible.
            double maxDecelAtI1 = maxDecel(path.getEntry(i1).point(), velocities[i1]);
            if (impliedAccel < maxDecelAtI1) {
                velocities[i0] = Math100.v0(velocities[i1], maxDecelAtI1, arclength);
                if (DEBUG)
                    System.out.printf("1 adjust vi %f\n", velocities[i0]);
            }
            // This can produce an infeasible result at i0 so apply it again there.
            // This will be conservative, which is better than violating the constraint.
            impliedAccel = Math100.accel(velocities[i0], velocities[i1], arclength);
            double maxDecelAtI0 = maxDecel(path.getEntry(i0).point(), velocities[i0]);
            if (impliedAccel < maxDecelAtI0) {
                velocities[i0] = Math100.v0(velocities[i1], maxDecelAtI0, arclength);
                if (DEBUG)
                    System.out.printf("2 adjust vi %f impliedA %f\n", velocities[i0], impliedAccel);
            }

            if (Math.abs(maxVelocity) < velocities[i0]) {
                velocities[i0] = Math.signum(velocities[i0]) * maxVelocity;
                if (DEBUG)
                    System.out.println("fix v one more time");
            }

            if (DEBUG)
                System.out.printf("BACK i %d vi0 %f vi1 %f max %f implied %f\n",
                        i0, velocities[i0], velocities[i1], maxDecelAtI1, impliedAccel);
        }
    }

    private double maxVelocity(PathSE3Point point) {
        double minVelocity = HIGH_V;
        for (TimingConstraintSE3 constraint : m_constraints) {
            minVelocity = Math.min(minVelocity, constraint.maxV(point));
        }
        return minVelocity;
    }

    private double maxAccel(PathSE3Point point, double velocity) {
        double minAccel = HIGH_ACCEL;
        for (TimingConstraintSE3 constraint : m_constraints) {
            minAccel = Math.min(minAccel, constraint.maxAccel(point, velocity));
        }
        return minAccel;
    }

    private double maxDecel(PathSE3Point point, double velocity) {
        double maxDecel = -HIGH_ACCEL;
        for (TimingConstraintSE3 constraint : m_constraints) {
            maxDecel = Math.max(maxDecel, constraint.maxDecel(point, velocity));
        }
        return maxDecel;
    }

    private static double dt(
            double v0,
            double v1,
            double arcLength,
            double accel) {
        if (Math.abs(accel) > EPSILON) {
            // If accelerating, find the time to go from v0 to v1.
            return (v1 - v0) / accel;
        }
        if (Math.abs(v0) > EPSILON) {
            // If moving, find the time to go distance dq at speed v0.
            return arcLength / v0;
        }
        return 0;
    }

}
