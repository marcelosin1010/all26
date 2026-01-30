package org.team100.lib.config;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.tuning.Mutable;

/**
 * Friction model with three constants, applicable for motor feedforward. These
 * values describe the entire mechanism; the motor friction itself is usually
 * negligible.
 * 
 * TODO: make a little library of friction configs based on experiment.
 * 
 * @param kS Static friction. Voltage to just barely get the mechanism moving
 *           from a stop.
 * @param kD Dynamic friction. Voltage to just barely keep the mechanism moving.
 * @param kV Viscous friction. Voltage to keep moving at a constant velocity.
 *           Volt-sec/rev.
 * @param vS Velocity threshold for static friction, rev/s.
 * 
 * @see https://mogi.bme.hu/TAMOP/robot_applications/ch07.html
 * @see https://en.wikipedia.org/wiki/Friction
 * @see https://en.wikipedia.org/wiki/Stribeck_curve
 * @see https://engee.com/helpcenter/stable/en/fmod-mechanical-translational-elements/translational-friction.html
 */
public class Friction {
    private final Mutable kS;
    private final Mutable kD;
    private final Mutable kV;
    private final double vS;

    public Friction(
            LoggerFactory log,
            double kS,
            double kD,
            double kV,
            double vS) {
        if (kS < kD)
            throw new IllegalArgumentException("static friction is always at least as high as dynamic friction");
        LoggerFactory fLog = log.type(this);
        this.kS = new Mutable(fLog, "kS", kS);
        this.kD = new Mutable(fLog, "kD", kD);
        this.kV = new Mutable(fLog, "kV", kV);
        this.vS = vS;
    }

    /**
     * Voltage to balance friction (i.e. this has the same sign as the supplied
     * speed).
     * 
     * @param motorRev_S setpoint speed
     */
    public double frictionFFVolts(double motorRev_S) {
        double viscous = kV.getAsDouble() * motorRev_S;
        double direction = Math.signum(motorRev_S);
        if (Math.abs(motorRev_S) < vS) {
            return viscous + kS.getAsDouble() * direction;
        }
        return viscous + kD.getAsDouble() * direction;
    }

}
