package org.team100.lib.subsystems.mecanum;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.rev.NeoCANSparkMotor;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.ReduxGyro;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.subsystems.mecanum.kinematics.MecanumKinematics100.Slip;
import org.team100.lib.util.CanId;

public class MecanumDriveFactory {

    public static MecanumDrive100 make(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            int statorLimit,
            CanId gyroId,
            CanId canFL,
            CanId canFR,
            CanId canRL,
            CanId canRR,
            double trackWidthM,
            double wheelbaseM,
            Slip slip,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Mecanum Drive");
        LoggerFactory logFL = log.name("frontLeft");
        LoggerFactory logFR = log.name("frontRight");
        LoggerFactory logRL = log.name("rearLeft");
        LoggerFactory logRR = log.name("rearRight");

        // all wheels need the same ff/pid values
        Feedforward100 ff = NeoCANSparkMotor.ff(log);
        // 10/22/25: Anay found this value worked well
        // PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.00005);
// Doubled the value for auton 
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.0001);
        // 10/22/25: Lincoln used this value
        // PIDConstants.makeVelocityPID(0.0003));

        Gyro gyro = gyro(log, gyroId);
        slip = slip(slip);

        BareMotor motorFL = NeoCANSparkMotor.get(log, canFL, MotorPhase.REVERSE, statorLimit, ff, pid);
        BareMotor motorFR = NeoCANSparkMotor.get(log, canFR, MotorPhase.FORWARD, statorLimit, ff, pid);
        BareMotor motorRL = NeoCANSparkMotor.get(log, canRL, MotorPhase.REVERSE, statorLimit, ff, pid);
        BareMotor motorRR = NeoCANSparkMotor.get(log, canRR, MotorPhase.FORWARD, statorLimit, ff, pid);

        return new MecanumDrive100(
                log, fieldLogger, gyro, trackWidthM, wheelbaseM, slip,
                OutboardLinearVelocityServo.make(logFL, motorFL, gearRatio, wheelDiaM),
                OutboardLinearVelocityServo.make(logFR, motorFR, gearRatio, wheelDiaM),
                OutboardLinearVelocityServo.make(logRL, motorRL, gearRatio, wheelDiaM),
                OutboardLinearVelocityServo.make(logRR, motorRR, gearRatio, wheelDiaM));
    }

    static Gyro gyro(LoggerFactory log, CanId gyroId) {
        if (gyroId == null)
            return null;
        return switch (Identity.instance) {
            case BLANK -> null;
            default -> new ReduxGyro(log, gyroId);
        };
    }

    static Slip slip(Slip slip) {
        return switch (Identity.instance) {
            case BLANK -> new Slip(1, 1, 1);// sim does not slip;
            default -> slip;
        };
    }

}
