package org.team100.frc2025;

import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.sensor.position.absolute.wpi.AS5048RotaryPositionSensor;
import org.team100.lib.sensor.position.absolute.wpi.SimulatedAS5048;

/** Container for simulation stuff */
public class Simulator implements Runnable {

    private final SimulatedGroundTruth m_truth;
    private final SimulatedAS5048 m_sensor;
    private final SimulatedCamera m_camera;

    public Simulator(LoggerFactory parent, AS5048RotaryPositionSensor sensor) {
        m_truth = new SimulatedGroundTruth(parent);
        m_sensor = new SimulatedAS5048(m_truth, sensor);
        m_camera = new SimulatedCamera(m_truth);
    }

    public void start() {
        m_truth.start(Takt.actual());
    }

    public void stop() {
        m_truth.stop(Takt.actual());
    }

    @Override
    public void run() {
        m_truth.run();
        m_sensor.run();
        m_camera.run();
    }

}
