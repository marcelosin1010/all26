package org.team100.frc2025;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.Logging;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.network.RawTags;
import org.team100.lib.sensor.position.absolute.EncoderDrive;
import org.team100.lib.sensor.position.absolute.wpi.AS5048RotaryPositionSensor;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;
import org.team100.lib.util.TimeInterpolatableBuffer100;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

public class Robot extends TimedRobot100 {
    /**
     * The position sensor is assumed to have a fixed delay of 600 us.
     */
    private static final double SENSOR_DELAY_S = 0.0006;
    private final BareMotor m_motor;
    private final AS5048RotaryPositionSensor m_sensor;
    private final RawTags m_rawTags;

    private final TimeInterpolatableBuffer100<Rotation2d> m_sensorBuffer;
    private final TimeInterpolatableBuffer100<Rotation2d> m_cameraBuffer;

    private final DoubleLogger m_logSensor;
    private final DoubleLogger m_logCamera;
    private final DoubleLogger m_logDiff;

    private final Simulator m_sim;

    public Robot() {
        Logging logging = Logging.instance();
        LoggerFactory log = logging.rootLogger;

        m_motor = NeoVortexCANSparkMotor.get(
                log,
                new CanId(1),
                MotorPhase.FORWARD,
                1, // current limit
                NeoVortexCANSparkMotor.ff(log),
                PIDConstants.makeVelocityPID(log, 0.0002));

        RoboRioChannel sensorChannel = new RoboRioChannel(1);
        m_sensor = new AS5048RotaryPositionSensor(
                log,
                sensorChannel,
                0.0, // offset
                EncoderDrive.DIRECT);

        m_sensorBuffer = new TimeInterpolatableBuffer100<>(2, 0, Rotation2d.kZero);
        m_cameraBuffer = new TimeInterpolatableBuffer100<>(2, 0, Rotation2d.kZero);

        // Update the buffer with the roll component, and accept the supplied timestamp
        // as true.
        m_rawTags = new RawTags(log, new Roll((r, t) -> m_cameraBuffer.put(t, r)));

        m_logSensor = log.doubleLogger(Level.TRACE, "lagged sensor");
        m_logCamera = log.doubleLogger(Level.TRACE, "lagged camera");
        m_logDiff = log.doubleLogger(Level.TRACE, "lagged difference");

        if (RobotBase.isSimulation()) {
            m_sim = new Simulator(log, m_sensor);
        } else {
            m_sim = null;
        }
    }

    @Override
    public void robotPeriodic() {
        // Update the clock.
        Takt.update();

        // Sim runs first so that it sees the Takt time.
        if (m_sim != null) {
            m_sim.run();
        }

        // Updates the sensor.
        Cache.refresh();

        // Read the sensor and update the sensor buffer.
        m_sensorBuffer.put(Takt.actual() - SENSOR_DELAY_S, new Rotation2d(
                m_sensor.getWrappedPositionRad()));

        // Read the camera and update the camera buffer.
        m_rawTags.update();

        // Sample the buffers from 1 sec ago and log.
        double past = Takt.actual() - 1.0;
        double laggedSensor = m_sensorBuffer.get(past).getRadians();
        m_logSensor.log(() -> laggedSensor);
        double laggedCamera = m_cameraBuffer.get(past).getRadians();
        m_logCamera.log(() -> laggedCamera);
        m_logDiff.log(() -> laggedSensor - laggedCamera);
    }

    @Override
    public void teleopInit() {
        m_motor.setVelocity(1, 0, 0);
        if (m_sim != null) {
            m_sim.start();
        }
    }

    @Override
    public void teleopExit() {
        m_motor.stop();
        if (m_sim != null) {
            m_sim.stop();
        }
    }

}
