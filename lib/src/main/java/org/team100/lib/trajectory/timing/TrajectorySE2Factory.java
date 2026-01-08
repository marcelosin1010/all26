package org.team100.lib.trajectory.timing;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.trajectory.TrajectorySE2;
import org.team100.lib.trajectory.path.PathEntrySE2;
import org.team100.lib.trajectory.path.PathPointSE2;
import org.team100.lib.trajectory.path.PathSE2;
import org.team100.lib.util.Math100;

/**
 * Given a path, produces a trajectory, which includes the path and adds a
 * schedule.
 */
public class TrajectorySE2Factory {
    public static final boolean DEBUG = true;
    private static final double EPSILON = 1e-6;

    /** Defaults to make the constraints set the actual. */
    private static final double HIGH_V = 100;
    private static final double HIGH_ACCEL = 1000;

    private final List<TimingConstraint> m_constraints;

    public TrajectorySE2Factory(List<TimingConstraint> constraints) {
        m_constraints = constraints;
    }

    /**
     * Assigns a time to each point in the path.
     * 
     * Output is these same points with time.
     */
    public TrajectorySE2 fromPath(PathSE2 path, double start_vel, double end_vel) {
        double[] distances = distances(path);
        double[] velocities = velocities(path, start_vel, end_vel, distances);
        double[] accels = accels(distances, velocities);
        double[] runningTime = runningTime(distances, velocities, accels);
        List<TimedStateSE2> timedStates = timedStates(path, velocities, accels, runningTime);
        return new TrajectorySE2(timedStates, m_constraints);
    }

    /////////////////////////////////////////////////////////////////////////////////////

    /**
     * Computes the length of each segment, as if it were a straight line, and
     * accumulates.
     */
    private double[] distances(PathSE2 path) {
        int n = path.length();
        double distances[] = new double[n];
        for (int i = 1; i < n; ++i) {
            double segmentLength = path.getEntry(i).point().distanceCartesian(path.getEntry(i - 1).point());
            distances[i] = segmentLength + distances[i - 1];
        }
        return distances;
    }

    /**
     * Assigns a velocity to each sample, using velocity, accel, and decel
     * constraints.
     */
    private double[] velocities(
            PathSE2 path, double start_vel, double end_vel, double[] distances) {
        double velocities[] = new double[path.length()];
        forward(path, start_vel, distances, velocities);
        backward(path, end_vel, distances, velocities);
        if (start_vel > velocities[0]) {
            System.out.printf("WARNING: start velocity %f is higher than constrained velocity %f\n",
                    start_vel, velocities[0]);
        }
        return velocities;
    }

    /**
     * Computes average accel based on distance of each arc and velocity at each
     * point.
     * 
     * Accel is attached to the *start* of each arc ([i] not [i+1])
     * 
     * The very last accel is always zero, but it's never used since it describes
     * samples off the end of the trajectory.
     */
    private double[] accels(double[] distances, double[] velocities) {
        int n = distances.length;
        double[] accels = new double[n];
        for (int i = 0; i < n - 1; ++i) {
            double arcLength = distances[i + 1] - distances[i];
            accels[i] = Math100.accel(velocities[i], velocities[i + 1], arcLength);
        }
        return accels;
    }

    /**
     * Computes duration of each arc and accumulate. Assigns a time to each point.
     */
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

    /**
     * Creates a list of timed states.
     */
    private List<TimedStateSE2> timedStates(
            PathSE2 path, double[] velocities, double[] accels, double[] runningTime) {
        int n = path.length();
        List<TimedStateSE2> timedStates = new ArrayList<>(n);
        for (int i = 0; i < n; ++i) {
            timedStates.add(new TimedStateSE2(path.getEntry(i).point(), runningTime[i], velocities[i], accels[i]));
        }
        return timedStates;
    }

    /**
     * Computes velocities[i+1] using velocity and acceleration constraints
     * referencing the state at i.
     */
    private void forward(
            PathSE2 path, double start_vel, double[] distances, double[] velocities) {
        int n = path.length();
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
            double maxVelocity = maxVelocity(path.getEntry(i + 1).point());
            if (DEBUG)
                System.out.printf("maxV i %d %f\n", i + 1, maxVelocity);
            // start with the maximum velocity
            velocities[i + 1] = maxVelocity;
            // reduce velocity to fit under the acceleration constraint
            double impliedAccel = Math100.accel(velocities[i], velocities[i + 1], arclength);
            double maxAccel = maxAccel(path.getEntry(i), velocities[i]);
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

    /**
     * Adjusts velocities[i] for decel constraint referencing the state at i+1.
     * 
     * This isn't strictly correct since the decel constraint should operate at i,
     * but walking backwards through the path, only i+1 is available, and the
     * samples should be enough close together, and the velocity should change
     * smoothly smooth enough so it shouldn't matter much in practice.
     * 
     * TODO: gah, it does seem to have too-big an effect, so fix it.
     */
    private void backward(
            PathSE2 path, double end_vel, double[] distances, double[] velocities) {
        int n = path.length();
        velocities[n - 1] = end_vel;
        for (int i = n - 2; i >= 0; --i) {
            if (DEBUG)
                System.out.printf("BACK i %d\n", i);
            double arclength = distances[i + 1] - distances[i];
            if (Math.abs(arclength) < EPSILON) {
                // already handled this case
                break;
            }

            double maxVelocity = maxVelocity(path.getEntry(i).point());
            if (DEBUG)
                System.out.printf("maxV i %d %f\n", i, maxVelocity);

            double impliedAccel = Math100.accel(velocities[i], velocities[i + 1], arclength);
            // Apply the decel constraint at the end of the segment since it is feasible.
            double maxDecel = maxDecel(path.getEntry(i).point(), velocities[i + 1]);
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

    /**
     * Returns the lowest (i.e. closest to zero) velocity constraint from the list
     * of constraints. Always positive or zero.
     */
    private double maxVelocity(PathPointSE2 sample) {
        double minVelocity = HIGH_V;
        for (TimingConstraint constraint : m_constraints) {
            minVelocity = Math.min(minVelocity, constraint.maxV(sample));
        }
        return minVelocity;
    }

    /**
     * Returns the lowest (i.e. closest to zero) acceleration constraint from the
     * list of constraints. Always positive or zero.
     */
    private double maxAccel(PathEntrySE2 sample, double velocity) {
        double minAccel = HIGH_ACCEL;
        for (TimingConstraint constraint : m_constraints) {
            minAccel = Math.min(minAccel, constraint.maxAccel(sample.point(), velocity));
        }
        return minAccel;
    }

    /**
     * Returns the highest (i.e. closest to zero) deceleration constraint from the
     * list of constraints. Always negative or zero.
     */
    private double maxDecel(PathPointSE2 sample, double velocity) {
        double maxDecel = -HIGH_ACCEL;
        for (TimingConstraint constraint : m_constraints) {
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