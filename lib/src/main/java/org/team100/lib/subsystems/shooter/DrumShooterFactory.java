package org.team100.lib.subsystems.shooter;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.rev.Neo550CANSparkMotor;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.util.CanId;

/** Configuration of motors on the demobot shooter. */
public class DrumShooterFactory {
    private static final double GEAR_RATIO = 5.2307692308;
    private static final double WHEEL_DIA_M = .33;

    public static DualDrumShooter make(
            LoggerFactory parent, int currentLimit) {
        LoggerFactory log = parent.name("shooter");
        LoggerFactory logL = log.name("left");
        LoggerFactory logR = log.name("right");

        CanId canL = new CanId(39);
        CanId canR = new CanId(19);

        Feedforward100 ff = Feedforward100.makeNeo550(log);
        // This P value is a guess
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.0002);

        BareMotor motorL = Neo550CANSparkMotor.get(log, canL, MotorPhase.FORWARD, currentLimit, ff, pid);
        BareMotor motorR = Neo550CANSparkMotor.get(log, canR, MotorPhase.REVERSE, currentLimit, ff, pid);

        return new DualDrumShooter(parent,
                OutboardLinearVelocityServo.make(logL, motorL, GEAR_RATIO, WHEEL_DIA_M),
                OutboardLinearVelocityServo.make(logR, motorR, GEAR_RATIO, WHEEL_DIA_M));
    }

}
