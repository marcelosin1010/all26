package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.sensor.position.incremental.IncrementalBareEncoder;

public class MockBareMotor implements BareMotor, IncrementalBareEncoder {
    public double output = 0;
    public double velocity = 0;
    public double position = 0;
    public double accel = 0;
    public double torque = 0;

    /**
     * this is for testing feedforwards.
     */
    public double ffVolts;
    public double frictionFFVolts;
    public double velocityFFVolts;
    public double torqueFFVolts;
    public double accelFFVolts;
    private final Feedforward100 m_ff;

    public MockBareMotor(Feedforward100 ff) {
        m_ff = ff;
    }

    @Override
    public void setDutyCycle(double output) {
        this.output = output;
    }

    @Override
    public void setVelocity(double velocityRad_S, double accel, double torque) {
        this.velocity = velocityRad_S;
        this.accel = accel;
        this.torque = torque;
    }

    @Override
    public void setUnwrappedPosition(
            double motorRad, double motorRad_S, double motorRad_S2, double torqueNm) {
        this.position = motorRad;
        this.velocity = motorRad_S;
        this.accel = motorRad_S2;
        this.torque = torqueNm;

        final double motorRev_S = motorRad_S / (2 * Math.PI);
        final double motorRev_S2 = motorRad_S2 / (2 * Math.PI);

        frictionFFVolts = m_ff.frictionFFVolts(motorRev_S);
        velocityFFVolts = m_ff.velocityFFVolts(motorRad_S);
        torqueFFVolts = getTorqueFFVolts(torqueNm);
        accelFFVolts = m_ff.accelFFVolts(motorRev_S, motorRev_S2);
        ffVolts = frictionFFVolts + velocityFFVolts + torqueFFVolts + accelFFVolts;
    }

    /** placeholder */
    @Override
    public double kROhms() {
        return 0.1;
    }

    /** placeholder */
    @Override
    public double kTNm_amp() {
        return 0.02;
    }

    @Override
    public IncrementalBareEncoder encoder() {
        return this;
    }

    @Override
    public void stop() {
        this.output = 0;
        this.velocity = 0;
    }

    @Override
    public void reset() {
        //
    }

    @Override
    public void close() {
        //
    }

    @Override
    public double getVelocityRad_S() {
        return this.velocity;
    }

    @Override
    public double getUnwrappedPositionRad() {
        return this.position;
    }

    @Override
    public double getCurrent() {
        return 0;
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        this.position = positionRad;
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
    }

    @Override
    public void periodic() {
    }

    @Override
    public void play(double freq) {
    }

}
