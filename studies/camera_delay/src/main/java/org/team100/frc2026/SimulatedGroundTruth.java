package org.team100.frc2026;

import java.util.function.DoubleFunction;

import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

import edu.wpi.first.math.geometry.Rotation2d;

/** The actual position for simulation */
public class SimulatedGroundTruth implements Runnable, DoubleFunction<Rotation2d> {
    private static final double SPEED_RAD_S = 1.0;
    /** When the disc started moving, or null if stopped */

    private final DoubleLogger m_log_truth;
    private Double t0;
    /** Position at t0 */
    private Rotation2d x0;

    public SimulatedGroundTruth(LoggerFactory parent) {
        LoggerFactory log = parent.type(this);
        m_log_truth = log.doubleLogger(Level.TRACE, "lagged truth");
        t0 = null;
        x0 = Rotation2d.kZero;
    }

    /** Start spinning at the current position at time t. */
    public void start(double t) {
        if (t0 != null) {
            // already running
            return;
        }
        // current x0 is the starting x0
        t0 = t;
    }

    /** Stop spinning at the position at time t. */
    public void stop(double t) {
        if (t0 == null) {
            // already stopped
            return;
        }
        // x0 is where it is right now
        x0 = apply(t);
        t0 = null;
    }

    @Override
    public void run() {
        m_log_truth.log(() -> apply(Takt.actual() - 1.0).getRadians());
    }

    @Override
    public Rotation2d apply(double t) {
        if (t0 == null) {
            // disc is stopped
            return x0;
        }
        double dt = t - t0;
        double dx = SPEED_RAD_S * dt;
        Rotation2d dr = new Rotation2d(dx);
        return x0.plus(dr);
    }

}
