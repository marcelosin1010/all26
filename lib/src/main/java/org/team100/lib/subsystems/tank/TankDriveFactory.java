package org.team100.lib.subsystems.tank;

import org.team100.lib.config.SimpleDynamics;
import org.team100.lib.config.Friction;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.rev.NeoCANSparkMotor;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

public class TankDriveFactory {

    public static TankDrive make(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            int statorLimit,
            CanId canL,
            CanId canR,
            double trackWidthM,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Tank Drive");
        LoggerFactory logL = log.name("left");
        LoggerFactory logR = log.name("right");

        SimpleDynamics ff = NeoCANSparkMotor.ff(log);
        Friction friction = NeoCANSparkMotor.friction(log);
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.005);

        BareMotor motorL = NeoCANSparkMotor.get(
                log, canL, MotorPhase.REVERSE, statorLimit, ff, friction, pid);
        BareMotor motorR = NeoCANSparkMotor.get(
                log, canR, MotorPhase.FORWARD, statorLimit, ff, friction, pid);

        return new TankDrive(fieldLogger, trackWidthM,
                OutboardLinearVelocityServo.make(logL, motorL, gearRatio, wheelDiaM),
                OutboardLinearVelocityServo.make(logR, motorR, gearRatio, wheelDiaM));
    }
}
