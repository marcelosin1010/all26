package org.team100.sim2026;

import java.util.function.IntSupplier;

public class Robot implements Actor, BallContainer {
    private static final int BIN_CAPACITY = 50;
    /** between adjacent zones */
    private static final int TRAVEL_TIME = 3;
    /** don't hunt for balls when there aren't very many */
    private static final int MIN_ON_FLOOR_TO_INTAKE = 5;
    /** The last few take longer to shoot so don't bother */
    private static final int MIN_COUNT_TO_SHOOT = 5;
    private static final int INTAKE_RATE = 10;
    private static final int SHOOT_RATE = 10;
    final Zone myZone;
    final Zone neutralZone;
    final Zone otherZone;
    final Hub myHub;
    final IntSupplier matchTimer;
    Zone location;
    int count;

    // this is null if we're not going anywhere
    // if it's not null, we complete the trip (to keep from changing our mind)
    Zone destination;
    int travelTimer;
    boolean active;
    // there are only 5 actions:
    // these in parallel:
    // * intake (qty)
    // * shoot/lob (qty)
    // ** can be the same balls intaken in the same second
    // ** if intake qty > shoot qty, balls accumulate until full
    // ** if intake qty < shoot qty, balls drain until empty
    // these exclusively:
    // * block (drivetrain)
    // * climb (climber)
    // * travel (drivetrain)
    // so 5 columns:
    int shot = 0;
    int taken = 0;
    int block = 0; // 1 if blocking
    int climb = 0; // 1 if climbing
    int move = 0; // 1 if moving to another zone
    // 2 chars
    final String name;

    public Robot(
            String name,
            Zone myZone,
            Zone neutralZone,
            Zone otherZone,
            Hub myHub,
            int initialCount,
            IntSupplier matchTimer) {
        this.name = name;
        this.myZone = myZone;
        this.neutralZone = neutralZone;
        this.otherZone = otherZone;
        this.myHub = myHub;
        this.matchTimer = matchTimer;
        // initial location is my own zone
        this.location = myZone;
        count = initialCount;
    }

    public String header() {
        return "loc,  ct, ta, sh, bl, cl, mo";
    }

    @Override
    public String toString() {
        return String.format("%3s, %3d, %2d, %2d, %2d, %2d, %2d",
                location.name, count(), taken, shot, block, climb, move);
    }

    @Override
    public int count() {
        return count;
    }

    @Override
    public Runnable step() {
        return this::action;
    }

    // make this into some sort of action object
    void move() {
        move = 1;
        shot = 0;
        taken = 0;
        block = 0;
        climb = 0;
    }

    void action() {
        if (destination != null) {
            // we're going somewhere, so keep going
            move();
            if (travelTimer > 0) {
                // still going
                --travelTimer;
            } else {
                // we have arrived
                location = destination;
                destination = null;
            }
            return;
        }

        if (matchTimer.getAsInt() < 20)
            // auton
            ferry();
        else
            teleop();

        // shoot();
        // lobToTheOtherZone();

    }

    void teleop() {
        if (active)
            ferry();
        else
            lob();
    }

    void shootaction(int actual) {
        shot = actual;
        block = 0;
        move = 0;
        climb = 0;
    }

    void intakeaction(int actual) {
        taken = actual;
        block = 0;
        move = 0;
        climb = 0;
    }

    void lob() {
        if (location == myZone) {
            // we're in our zone.
            if (count > MIN_COUNT_TO_SHOOT) {
                // we have balls to shoot, so shoot them
                int actual = shoot();
                shootaction(actual);
            } else {
                // nothing left to shoot
                moveTo(neutralZone);
            }
        } else {
            if (location.count() > 5) {
                // there are balls to intake.
                int taken = location.take(INTAKE_RATE);
                count += taken;
                intakeaction(taken);
            }
            if (count > 5) {
                // we have balls to lob (including the ones just taken),
                // so lob them
                int actual = shootLob();
                shootaction(actual);
            }

        }
    }

    void ferry() {
        if (location == myZone) {
            // we're in our zone.
            if (myZone.count() > MIN_ON_FLOOR_TO_INTAKE) {
                // there are balls to intake
                int taken = myZone.take(INTAKE_RATE);
                count += taken;
                intakeaction(taken);
            }
            if (count > MIN_COUNT_TO_SHOOT) {
                // we have balls to shoot, so shoot them
                int actual = shoot();
                shootaction(actual);
            } else {
                // nothing left here.
                moveTo(neutralZone);
            }
        } else {
            // we're in the neutral zone
            // for now let's ferry
            if (count > BIN_CAPACITY) {
                // time to go back
                moveTo(myZone);
            } else {
                if (location.count() > 5) {
                    int taken = location.take(INTAKE_RATE);
                    count += taken;
                    intakeaction(taken);
                } else {
                    // nothing left here to pick up so go back
                    moveTo(myZone);
                }
            }

        }

    }

    void moveTo(Zone zone) {
        destination = zone;
        travelTimer = TRAVEL_TIME;
        move();
    }

    /** Shoot up to everything we have in one second */
    int shoot() {
        int actual = Math.min(count, SHOOT_RATE);
        myHub.accept(actual);
        count = count - actual;
        return actual;
    }

    /** Lob up to everything we have in one second */
    int shootLob() {
        int actual = Math.min(count, SHOOT_RATE);
        myZone.accept(actual);
        count = count - actual;
        return actual;

    }

    void lobToTheOtherZone() {
        // try to take some
        int taken = myZone.take(INTAKE_RATE);
        // taken balls are immediately available
        count += taken;
        // lob everything to the other side (this cannot be refused)
        otherZone.accept(count);
        count = 0;
    }
}
