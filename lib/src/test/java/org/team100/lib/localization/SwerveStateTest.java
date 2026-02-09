package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelR1;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePosition100;
import org.team100.lib.subsystems.swerve.module.state.SwerveModulePositions;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Rotation2d;

class SwerveStateTest implements Timeless {
    private final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    // this is a 0.5 m square.
    SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.forTest(logger);

    @Test
    void testInterp0() {
        // initially at rest, finally in motion.
        // what does the interpolator do?
        ModelSE2 s0 = new ModelSE2();
        IsotropicNoiseSE2 n0 = IsotropicNoiseSE2.fromStdDev(1, 1);
        SwerveModulePositions p0 = new SwerveModulePositions(
                new SwerveModulePosition100(0, Optional.empty()),
                new SwerveModulePosition100(0, Optional.empty()),
                new SwerveModulePosition100(0, Optional.empty()),
                new SwerveModulePosition100(0, Optional.empty()));
        Rotation2d gyroYaw0 = new Rotation2d();
        SwerveState r0 = new SwerveState(
                kinodynamics.getKinematics(), s0, n0, p0, gyroYaw0);
        assertEquals(0, r0.state().theta().x(), 0.001);
        assertEquals(0, r0.state().theta().v(), 0.001);

        // 0.5m square means r = sqrt(2)/4.
        // Each wheel moves r, we turn 1 radian here.
        // if we're moving 1 rad/s now, and we moved 1 rad,
        // x = 1/2 a t ^ 2 = 1
        // v = a t = 1
        // so
        // 1 = 1/2 * 1 * t; t = 2, a = 0.5.

        ModelSE2 s1 = new ModelSE2(new ModelR1(), new ModelR1(), new ModelR1(1, 1));
        IsotropicNoiseSE2 n1 = IsotropicNoiseSE2.fromStdDev(1, 1);

        SwerveModulePositions p1 = new SwerveModulePositions(
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(-3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(-Math.PI / 4))));
        Rotation2d gyroYaw1 = new Rotation2d(1);

        // current speed is 1 rad/s
        SwerveState r1 = new SwerveState(
                kinodynamics.getKinematics(), s1, n1, p1, gyroYaw1);

        assertEquals(1, r1.state().theta().x(), 0.001);
        assertEquals(1, r1.state().theta().v(), 0.001);

        {
            // t=0 should return r0. note "t" is not time
            SwerveState lerp = r0.interpolate(r1, 0.0);
            assertEquals(0, lerp.state().theta().x(), 0.001);
            assertEquals(0, lerp.state().theta().v(), 0.001);
            assertEquals(0, lerp.gyroYaw().getRadians(), 0.001);

        }
        {
            // t=1 should return r1. note this is a special case.
            SwerveState lerp = r0.interpolate(r1, 1.0);
            assertEquals(1, lerp.state().theta().x(), 0.001);
            assertEquals(1, lerp.state().theta().v(), 0.001);
            assertEquals(1, lerp.gyroYaw().getRadians(), 0.001);
        }
        {
            // t=0.5 is just halfway there.
            SwerveState lerp = r0.interpolate(r1, 0.5);
            assertEquals(0.5, lerp.state().theta().x(), 0.001);
            assertEquals(0.5, lerp.state().theta().v(), 0.001);
            assertEquals(0.5, lerp.gyroYaw().getRadians(), 0.001);
        }
    }

    @Test
    void testInterp1() {
        ModelSE2 s0 = new ModelSE2();
        IsotropicNoiseSE2 n0 = IsotropicNoiseSE2.fromStdDev(1, 1);
        // initally driving straight
        SwerveModulePositions p0 = new SwerveModulePositions(
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())),
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())),
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())),
                new SwerveModulePosition100(0, Optional.of(new Rotation2d())));
        Rotation2d gyroYaw0 = new Rotation2d();
        SwerveState r0 = new SwerveState(
                kinodynamics.getKinematics(), s0, n0, p0, gyroYaw0);
        assertEquals(0, r0.state().theta().x(), 0.001);
        assertEquals(0, r0.state().theta().v(), 0.001);

        // 0.5m square means r = sqrt(2)/4.
        // Each wheel moves r, we turn 1 radian here.
        // if we're moving 1 rad/s now, and we moved 1 rad,
        // x = 1/2 a t ^ 2 = 1
        // v = a t = 1
        // so
        // 1 = 1/2 * 1 * t; t = 2, a = 0.5.

        ModelSE2 s1 = new ModelSE2(new ModelR1(), new ModelR1(), new ModelR1(1, 1));
        IsotropicNoiseSE2 n1 = IsotropicNoiseSE2.fromStdDev(1, 1);
        SwerveModulePositions p1 = new SwerveModulePositions(
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(-3 * Math.PI / 4))),
                new SwerveModulePosition100(Math.sqrt(2) / 4,
                        Optional.of(new Rotation2d(-Math.PI / 4))));
        Rotation2d gyroYaw1 = new Rotation2d(1);

        // current speed is 1 rad/s
        SwerveState r1 = new SwerveState(
                kinodynamics.getKinematics(), s1, n1, p1, gyroYaw1);

        assertEquals(1, r1.state().theta().x(), 0.001);
        assertEquals(1, r1.state().theta().v(), 0.001);

        {
            // t=0 should return r0. note "t" is not time
            SwerveState lerp = r0.interpolate(r1, 0.0);
            assertEquals(0, lerp.state().theta().x(), 0.001);
            assertEquals(0, lerp.state().theta().v(), 0.001);
            assertEquals(0, lerp.gyroYaw().getRadians(), 0.001);
        }
        {
            // t=1 should return r1. note this is a special case.
            SwerveState lerp = r0.interpolate(r1, 1.0);
            assertEquals(1, lerp.state().theta().x(), 0.001);
            assertEquals(1, lerp.state().theta().v(), 0.001);
            assertEquals(1, lerp.gyroYaw().getRadians(), 0.001);
        }
        {
            // t=0.5
            SwerveState lerp = r0.interpolate(r1, 0.5);
            // wheels are assumed to turn immediately to the r1 angles,
            // so this case is the same as the case above.
            assertEquals(0.5, lerp.state().theta().x(), 0.001);
            assertEquals(0.5, lerp.state().theta().v(), 0.001);
            assertEquals(0.5, lerp.gyroYaw().getRadians(), 0.001);
        }
    }

}
