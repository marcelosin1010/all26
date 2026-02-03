package org.team100.lib.motor.rev;

import java.util.function.Supplier;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.DoubleCache;
import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.sensor.position.incremental.rev.CANSparkEncoder;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLimitSwitch;

/**
 * PID units for outboard positional control are duty cycle per revolution, so
 * if you want to control to 0.1 revolutions, P should be something like 2.
 * 
 * PID units for outboard velocity control are duty cycle per RPM, so if you
 * want to control to a few hundred RPM, P should be something like 0.0002.
 * 
 * Relies on Memo and Takt, so you must put Memo.resetAll() and Takt.update() in
 * Robot.robotPeriodic().
 * 
 * Current limit is stator current.
 * 
 * https://docs.revrobotics.com/brushless/spark-max/gs/make-it-spin#limiting-current
 * https://www.chiefdelphi.com/t/rev-robotics-2024-2025/471083/26
 */
public abstract class CANSparkMotor implements BareMotor {
    private final LoggerFactory m_log;
    protected final Feedforward100 m_ff;
    protected final SparkBase m_motor;
    private final RevConfigurator m_configurator;
    protected final SparkLimitSwitch m_forLimitSwitch;
    protected final SparkLimitSwitch m_revLimitSwitch;
    protected final RelativeEncoder m_encoder;
    protected final SparkClosedLoopController m_pidController;
    // CACHES
    private final DoubleCache m_encoder_position;
    private final DoubleCache m_encoder_velocity;
    private final DoubleCache m_current;
    private final DoubleCache m_supplyVoltage;
    private final DoubleCache m_output;
    // private final DoubleCache m_temp;
    // LOGGERS
    private final DoubleLogger m_log_desired_position;
    private final DoubleLogger m_log_desired_speed;
    private final DoubleLogger m_log_desired_accel;
    private final DoubleLogger m_log_friction_FF;
    private final DoubleLogger m_log_velocity_FF;
    private final DoubleLogger m_log_accel_FF;
    private final DoubleLogger m_log_torque_FF;
    private final DoubleLogger m_log_duty;
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_velocity;
    private final DoubleLogger m_log_rpm;
    private final DoubleLogger m_log_current;
    private final DoubleLogger m_log_supplyVoltage;
    // private final DoubleLogger m_log_torque;
    // private final DoubleLogger m_log_temp;

    protected CANSparkMotor(
            LoggerFactory parent,
            SparkBase motor,
            NeutralMode neutral,
            MotorPhase motorPhase,
            int statorCurrentLimit,
            Feedforward100 ff,
            PIDConstants pid) {
        m_motor = motor;
        m_log = parent.type(this);
        m_ff = ff;

        m_configurator = new RevConfigurator(
                m_log,
                m_motor,
                neutral,
                motorPhase,
                statorCurrentLimit,
                pid);
        m_configurator.longCANTimeout();
        m_configurator.baseConfig();
        m_configurator.motorConfig();
        m_configurator.currentConfig();
        m_configurator.pidConfig();
        m_configurator.zeroCANTimeout();

        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getClosedLoopController();

        // LIMIT SWITCHES
        m_forLimitSwitch = m_motor.getForwardLimitSwitch();
        m_revLimitSwitch = m_motor.getReverseLimitSwitch();

        // CACHES
        m_encoder_position = Cache.ofDouble(m_encoder::getPosition);
        m_encoder_velocity = Cache.ofDouble(m_encoder::getVelocity);
        m_current = Cache.ofDouble(m_motor::getOutputCurrent);
        m_supplyVoltage = Cache.ofDouble(m_motor::getBusVoltage);
        m_output = Cache.ofDouble(m_motor::getAppliedOutput);
        // m_temp = Memo.ofDouble(m_motor::getMotorTemperature);
        // LOGGERS
        m_log.intLogger(Level.TRACE, "Device ID").log(m_motor::getDeviceId);
        m_log_desired_position = m_log.doubleLogger(Level.DEBUG, "desired position (rev)");
        m_log_desired_speed = m_log.doubleLogger(Level.DEBUG, "desired speed (rev_s)");
        m_log_desired_accel = m_log.doubleLogger(Level.TRACE, "desired accel (rev_s2)");
        m_log_friction_FF = m_log.doubleLogger(Level.TRACE, "friction feedforward (v)");
        m_log_velocity_FF = m_log.doubleLogger(Level.TRACE, "velocity feedforward (v)");
        m_log_accel_FF = m_log.doubleLogger(Level.TRACE, "accel feedforward (v)");
        m_log_torque_FF = m_log.doubleLogger(Level.TRACE, "torque feedforward (v)");
        m_log_duty = m_log.doubleLogger(Level.DEBUG, "Duty Cycle");
        m_log_position = m_log.doubleLogger(Level.DEBUG, "position (rev)");
        m_log_velocity = m_log.doubleLogger(Level.DEBUG, "velocity (rev_s)");
        m_log_rpm = m_log.doubleLogger(Level.TRACE, "velocity (RPM)");
        m_log_current = m_log.doubleLogger(Level.DEBUG, "current (A)");
        m_log_supplyVoltage = m_log.doubleLogger(Level.DEBUG, "voltage (V)");
        // m_log_torque = m_log.doubleLogger(Level.TRACE, "torque (Nm)");
        // m_log_temp = m_log.doubleLogger(Level.TRACE, "temperature (C)");
    }

    @Override
    public void setDutyCycle(double output) {
        m_motor.set(output);
        m_log_duty.log(() -> output);
        log();
    }

    public boolean getForwardLimitSwitch() {
        return m_forLimitSwitch.isPressed();
    }

    public boolean getReverseLimitSwitch() {
        return m_revLimitSwitch.isPressed();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        int currentA = (int) (torqueNm / kTNm_amp());
        m_configurator.overrideStatorLimit(currentA);
    }

    /**
     * Use outboard PID control to hold the given velocity, with velocity,
     * acceleration, and torque feedforwards.
     */
    @Override
    public void setVelocity(double motorRad_S, double motorRad_S2, double motorTorqueNm) {
        final double motorRev_S = motorRad_S / (2 * Math.PI);
        final double motorRev_S2 = motorRad_S2 / (2 * Math.PI);

        final double frictionFFVolts = m_ff.frictionFFVolts(motorRev_S);
        final double velocityFFVolts = m_ff.velocityFFVolts(motorRad_S);
        final double accelFFVolts = m_ff.accelFFVolts(motorRev_S, motorRev_S2);
        final double torqueFFVolts = getTorqueFFVolts(motorTorqueNm);

        final double FF = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;

        final double motorRev_M = motorRev_S * 60;
        warn(() -> m_pidController.setSetpoint(
                motorRev_M, ControlType.kVelocity, ClosedLoopSlot.kSlot1, FF, ArbFFUnits.kVoltage));

        m_log_desired_speed.log(() -> motorRev_S);
        m_log_desired_accel.log(() -> motorRev_S2);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> velocityFFVolts);
        m_log_accel_FF.log(() -> accelFFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
        log();
    }

    /**
     * Use outboard PID control to hold the given position, with velocity and torque
     * feedforwards.
     * 
     * Motor revolutions wind up, so setting 0 revs and 1 rev are different.
     */
    @Override
    public void setUnwrappedPosition(
            double motorRad,
            double motorRad_S,
            double motorRad_S2,
            double motorTorqueNm) {
        final double motorRev = motorRad / (2 * Math.PI);
        final double motorRev_S = motorRad_S / (2 * Math.PI);
        final double motorRev_S2 = motorRad_S2 / (2 * Math.PI);

        final double frictionFFVolts = m_ff.frictionFFVolts(motorRev_S);
        final double velocityFFVolts = m_ff.velocityFFVolts(motorRad_S);
        final double accelFFVolts = m_ff.accelFFVolts(motorRev_S, motorRev_S2);
        final double torqueFFVolts = getTorqueFFVolts(motorTorqueNm);

        final double FF = frictionFFVolts + velocityFFVolts + accelFFVolts + torqueFFVolts;

        warn(() -> m_pidController.setSetpoint(
                motorRev, ControlType.kPosition, ClosedLoopSlot.kSlot0, FF, ArbFFUnits.kVoltage));

        m_log_desired_position.log(() -> motorRev);
        m_log_desired_speed.log(() -> motorRev_S);
        m_log_desired_accel.log(() -> motorRev_S2);
        m_log_friction_FF.log(() -> frictionFFVolts);
        m_log_velocity_FF.log(() -> velocityFFVolts);
        m_log_accel_FF.log(() -> accelFFVolts);
        m_log_torque_FF.log(() -> torqueFFVolts);
        log();
    }

    /** Value is updated in Robot.robotPeriodic(). */
    @Override
    public double getVelocityRad_S() {
        return getRateRPM() * 2 * Math.PI / 60;
    }

    @Override
    public double getCurrent() {
        return m_current.getAsDouble();
    }

    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        setEncoderPosition(positionRad / (2 * Math.PI));
    }

    @Override
    public CANSparkEncoder encoder() {
        return new CANSparkEncoder(m_log, this);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public void reset() {
        m_encoder_position.reset();
        m_encoder_velocity.reset();
    }

    @Override
    public void close() {
        m_motor.close();
    }

    /**
     * Value is updated in Robot.robotPeriodic().
     * 
     * @return torque in Nm
     */
    // public double getMotorTorque() {
    // return m_current.getAsDouble() * kTNm_amp();
    // }

    /**
     * Value is updated in Robot.robotPeriodic().
     * 
     * @return integrated sensor position in rotations.
     */
    public double getPositionRot() {
        return m_encoder_position.getAsDouble();
    }

    @Override
    public double getUnwrappedPositionRad() {
        double motorPositionRev = getPositionRot();
        double positionRad = motorPositionRev * 2 * Math.PI;
        return positionRad;
    }

    /**
     * Value is updated in Robot.robotPeriodic().
     * 
     * @return integrated sensor velocity in RPM
     */
    public double getRateRPM() {
        return m_encoder_velocity.getAsDouble();
    }

    /**
     * Sets integrated sensor position to zero.
     */
    public void resetEncoderPosition() {
        warn(() -> m_encoder.setPosition(0));
        m_encoder_position.reset();
        m_encoder_velocity.reset();
    }

    /**
     * Set integrated sensor position in rotations.
     */
    public void setEncoderPosition(double motorPositionRev) {
        warn(() -> m_encoder.setPosition(motorPositionRev));
    }

    protected void log() {
        m_log_position.log(m_encoder_position);
        m_log_velocity.log(() -> m_encoder_velocity.getAsDouble() / 60);
        m_log_rpm.log(m_encoder_velocity);
        m_log_current.log(m_current);
        m_log_supplyVoltage.log(m_supplyVoltage);
        m_log_duty.log(m_output);
        // m_log_torque.log(this::getMotorTorque);
        // m_log_temp.log(m_temp);
    }

    @Override
    public void periodic() {
        log();
    }

    @Override
    public void play(double freq) {
    }

    private static void warn(Supplier<REVLibError> s) {
        REVLibError errorCode = s.get();
        if (errorCode != REVLibError.kOk) {
            System.out.println("WARNING: " + errorCode.name());
        }
    }
}
