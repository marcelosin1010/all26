package org.team100.lib.subsystems.tank;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.RevPIDConstants;
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

        // all wheels need the same ff/pid values
        Feedforward100 ff = NeoCANSparkMotor.ff(log);
        // 10/22/25: Anay found this value worked well
        RevPIDConstants pid = RevPIDConstants.makeVelocityPID(log, 0.00005);
        // 10/22/25: Lincoln used this value
        // RevPIDConstants.makeVelocityPID(0.0003));

        BareMotor motorL = NeoCANSparkMotor.get(
                log, canL, MotorPhase.REVERSE, statorLimit, ff, pid);
        BareMotor motorR = NeoCANSparkMotor.get(
                log, canR, MotorPhase.FORWARD, statorLimit, ff, pid);

        return new TankDrive(fieldLogger, trackWidthM,
                OutboardLinearVelocityServo.make(logL, motorL, gearRatio, wheelDiaM),
                OutboardLinearVelocityServo.make(logR, motorR, gearRatio, wheelDiaM));
    }
}
