package org.team100.lib.trajectory.timing;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.trajectory.TrajectorySE3;
import org.team100.lib.trajectory.path.PathPointSE3;
import org.team100.lib.trajectory.path.PathSE3;
import org.team100.lib.util.Math100;

/**
 * For now this is a copy of the SE2 version.
 * TODO: combine some aspects
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
        PathPointSE3[] samples = getSamples(path);
        return fromSamples(samples, start_vel, end_vel);
    }

    private PathPointSE3[] getSamples(PathSE3 path) {
        return path.resample();
    }

    public TrajectorySE3 fromSamples(
            PathPointSE3[] samples,
            double start_vel,
            double end_vel) {
        double[] distances = distances(samples);
        double[] velocities = velocities(samples, start_vel, end_vel, distances);
        double[] accels = accels(distances, velocities);
        double[] runningTime = runningTime(distances, velocities, accels);
        List<TimedStateSE3> timedStates = timedStates(samples, velocities, accels, runningTime);
        return new TrajectorySE3(timedStates, m_constraints);
    }

    private double[] distances(PathPointSE3[] samples) {
        int n = samples.length;
        double distances[] = new double[n];
        for (int i = 1; i < n; ++i) {
            double arclength = samples[i].distanceCartesian(samples[i - 1]);
            distances[i] = arclength + distances[i - 1];
        }
        return distances;
    }

    private double[] velocities(
            PathPointSE3[] samples, double start_vel, double end_vel, double[] distances) {
        double velocities[] = new double[samples.length];
        forward(samples, start_vel, distances, velocities);
        backward(samples, end_vel, distances, velocities);
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

    private List<TimedStateSE3> timedStates(
            PathPointSE3[] samples, double[] velocities, double[] accels, double[] runningTime) {
        int n = samples.length;
        List<TimedStateSE3> timedStates = new ArrayList<>(n);
        for (int i = 0; i < n; ++i) {
            timedStates.add(new TimedStateSE3(samples[i], runningTime[i], velocities[i], accels[i]));
        }
        return timedStates;
    }

    private void forward(
            PathPointSE3[] samples, double start_vel, double[] distances, double[] velocities) {
        int n = samples.length;
        velocities[0] = start_vel;
        for (int i = 0; i < n - 1; ++i) {
            if (DEBUG)
                System.out.printf("FWD i %d\n", i);
            double arclength = distances[i + 1] - distances[i];
            if (Math.abs(arclength) < EPSILON) {
                if (DEBUG)
                    System.out.printf("i %d zero arc\n", i);
                // zero-length arcs have the same state at both ends
                velocities[i + 1] = velocities[i];
                break;
            }
            // velocity constraint depends only on state
            double maxVelocity = maxVelocity(samples[i + 1]);
            if (DEBUG)
                System.out.printf("maxV i %d %f\n", i + 1, maxVelocity);
            // start with the maximum velocity
            velocities[i + 1] = maxVelocity;
            // reduce velocity to fit under the acceleration constraint
            double impliedAccel = Math100.accel(velocities[i], velocities[i + 1], arclength);
            double maxAccel = maxAccel(samples[i], velocities[i]);
            if (impliedAccel > maxAccel/* + EPSILON */) {
                velocities[i + 1] = Math100.v1(velocities[i], maxAccel, arclength);
                if (DEBUG)
                    System.out.printf("adjust vi+1 %f\n", velocities[i + 1]);
            }
            if (DEBUG)
                System.out.printf("FWD i %d vi %f vi+1 %f maxA %f impliedA %f\n",
                        i, velocities[i], velocities[i + 1], maxAccel, impliedAccel);
        }
    }

    private void backward(
            PathPointSE3[] samples, double end_vel, double[] distances, double[] velocities) {
        int n = samples.length;
        velocities[n - 1] = end_vel;
        for (int i = n - 2; i >= 0; --i) {
            if (DEBUG)
                System.out.printf("BACK i %d\n", i);
            double arclength = distances[i + 1] - distances[i];
            if (Math.abs(arclength) < EPSILON) {
                // already handled this case
                break;
            }

            double maxVelocity = maxVelocity(samples[i]);
            if (DEBUG)
                System.out.printf("maxV i %d %f\n", i, maxVelocity);

            double impliedAccel = Math100.accel(velocities[i], velocities[i + 1], arclength);
            // Apply the decel constraint at the end of the segment since it is feasible.
            double maxDecel = maxDecel(samples[i], velocities[i + 1]);
            if (impliedAccel < maxDecel/* - EPSILON */) {
                velocities[i] = Math100.v0(velocities[i + 1], maxDecel, arclength);
                if (DEBUG)
                    System.out.printf("adjust vi %f\n", velocities[i]);
            }
            if (Math.abs(maxVelocity) < velocities[i]) {
                velocities[i] = Math.signum(velocities[i]) * maxVelocity;
                if (DEBUG)
                    System.out.println("fix v one more time");
            }

            if (DEBUG)
                System.out.printf("BACK i %d vi %f vi+1 %f max %f implied %f\n",
                        i, velocities[i], velocities[i + 1], maxDecel, impliedAccel);
        }
    }

    private double maxVelocity(PathPointSE3 sample) {
        double minVelocity = HIGH_V;
        for (TimingConstraintSE3 constraint : m_constraints) {
            minVelocity = Math.min(minVelocity, constraint.maxV(sample));
        }
        return minVelocity;
    }

    private double maxAccel(PathPointSE3 sample, double velocity) {
        double minAccel = HIGH_ACCEL;
        for (TimingConstraintSE3 constraint : m_constraints) {
            minAccel = Math.min(minAccel, constraint.maxAccel(sample, velocity));
        }
        return minAccel;
    }

    private double maxDecel(PathPointSE3 sample, double velocity) {
        double maxDecel = -HIGH_ACCEL;
        for (TimingConstraintSE3 constraint : m_constraints) {
            maxDecel = Math.max(maxDecel, constraint.maxDecel(sample, velocity));
        }
        return maxDecel;
    }

    // TODO: move this to some util class?
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
