package org.team100.sim2026.robots;

import org.team100.sim2026.Actor;
import org.team100.sim2026.Alliance;
import org.team100.sim2026.BallAcceptor;
import org.team100.sim2026.BallContainer;
import org.team100.sim2026.Hub;
import org.team100.sim2026.Sim;
import org.team100.sim2026.Tower;
import org.team100.sim2026.Zone;
import org.team100.sim2026.actions.Action;
import org.team100.sim2026.actions.Block;
import org.team100.sim2026.actions.Climb;
import org.team100.sim2026.actions.Idle;
import org.team100.sim2026.actions.IntakeAndLob;
import org.team100.sim2026.actions.IntakeAndScore;
import org.team100.sim2026.actions.IntakeOnly;
import org.team100.sim2026.actions.LobOnly;
import org.team100.sim2026.actions.Move;
import org.team100.sim2026.actions.ScoreOnly;

public abstract class Robot implements Actor, BallContainer {
    private static final int BIN_CAPACITY = 50;
    /** between adjacent zones */
    public static final int TRAVEL_TIME = 3;
    /** don't hunt for balls when there aren't very many */
    private static final int MIN_ON_FLOOR_TO_INTAKE = 5;
    /** The last few take longer to shoot so don't bother */
    private static final int MIN_COUNT_TO_SHOOT = 5;
    private static final int INTAKE_RATE = 25;
    private static final int SHOOT_RATE = 10;
    final Zone myZone;
    final Zone neutralZone;
    final Zone otherZone;
    final Hub myHub;
    final Tower myTower;
    Zone location;
    int count;

    // this is null if we're not going anywhere
    // if it's not null, we complete the trip (to keep from changing our mind)
    Zone destination;
    int travelTimer;
    public boolean active;

    public Action action = new Idle();
    // 2 chars
    public final String name;
    final Sim sim;
    final Alliance alliance;
    /** another robot is blocking us, which slows everything down. */
    public boolean blocked;

    public Robot(
            Alliance alliance,
            String name,
            int initialCount,
            Sim sim) {
        this.alliance = alliance;
        this.sim = sim;
        this.name = name;
        this.myZone = alliance == Alliance.RED ? sim.redZone : sim.blueZone;
        this.neutralZone = sim.neutralZone;
        this.otherZone = alliance == Alliance.RED ? sim.blueZone : sim.redZone;
        this.myHub = alliance == Alliance.RED ? sim.redHub : sim.blueHub;
        this.myTower = alliance == Alliance.RED ? sim.redTower : sim.blueTower;
        // initial location is my own zone
        this.location = myZone;
        count = initialCount;
    }

    @Override
    public Runnable step() {
        return () -> {
            if (stillClimbing()) {
                return;
            }
            if (stillMoving()) {
                return;
            }
            action();
        };
    }

    // TODO: make this use the "action" and work for all actions with duration.
    boolean stillMoving() {
        if (destination != null) {
            // we're going somewhere
            --travelTimer;
            if (travelTimer > 0) {
                // still going
                action = new Move(destination.name);
                return true;
            } else {
                // we have arrived
                location = destination;
                destination = null;
            }
        }
        return false;
    }

    boolean stillClimbing() {
        if (action.getClass() == Climb.class) {
            Climb climb = (Climb) action;
            // TODO: step() should be called by Sim.
            // System.out.println(name);
            climb.step();
            return !(climb.done());
        }
        return false;
    }

    /** Decide what to do during this period. */
    abstract void action();

    /**
     * score until our zone is exhausted, then move to the neutral zone.
     */
    void score() {
        if (location != myZone)
            throw new IllegalStateException("only score from my zone");
        // we're in our zone, so attempt to score whatever
        // we have in the bin, and whatever is nearby.
        if (location.count() > MIN_ON_FLOOR_TO_INTAKE) {
            intakeAndScore();
        } else {
            if (count > MIN_COUNT_TO_SHOOT) {
                // we have balls to shoot, so shoot them
                int actual = shoot(myHub);
                action = new ScoreOnly(actual);
            } else {
                // nothing left
                moveTo(neutralZone);
            }
        }
    }

    /**
     * Intake and score from our zone.
     */
    void scoreOnly() {
        if (location != myZone) {
            moveTo(myZone);
        } else if (location.count() > MIN_ON_FLOOR_TO_INTAKE) {
            intakeAndScore();
        } else {
            if (count > MIN_COUNT_TO_SHOOT) {
                // we have balls to shoot, so shoot them
                int actual = shoot(myHub);
                action = new ScoreOnly(actual);
            } else {
                // nothing left, wait
                action = new Idle();
            }
        }
    }

    int intakeRate() {
        if (blocked)
            return INTAKE_RATE / 2;
        return INTAKE_RATE;
    }

    void intakeAndScore() {
        // there are balls nearby so shoot them
        int intakeCount = Math.min(Math.max(0, BIN_CAPACITY - count), intakeRate());
        int taken = location.take(intakeCount);
        count += taken;
        int actual = shoot(myHub);
        action = new IntakeAndScore(taken, actual);
    }

    /**
     * LOB:
     * * move to the neutral zone
     * * lob balls from there into our zone
     * * when exhausted, drive to our zone and score
     */
    void lob() {
        if (location == myZone) {
            if (active) {
                score();
            } else {
                moveTo(neutralZone);
            }
        } else {
            // we're in the neutral zone
            if (location.count() > MIN_ON_FLOOR_TO_INTAKE) {
                intakeAndLob();
            } else {
                if (count > MIN_COUNT_TO_SHOOT) {
                    // lob the remainder
                    int lobbed = shoot(myZone);
                    action = new LobOnly(lobbed);
                } else {
                    // nothing on the floor, nothing in the bin
                    moveTo(myZone);
                }
            }
        }
    }

    /**
     * LOB ONLY:
     * * lob without scoring
     */
    void lobOnly() {
        if (location == myZone) {
            moveTo(neutralZone);
        } else {
            // we're in the neutral zone
            if (location.count() > MIN_ON_FLOOR_TO_INTAKE) {
                intakeAndLob();
            } else {
                if (count > MIN_COUNT_TO_SHOOT) {
                    // lob the remainder
                    int lobbed = shoot(myZone);
                    action = new LobOnly(lobbed);
                }
            }
        }
    }

    private void intakeAndLob() {
        // both intake and lob
        int intakeCount = Math.min(Math.max(0, BIN_CAPACITY - count), intakeRate());
        int taken = location.take(intakeCount);
        count += taken;
        int lobbed = shoot(myZone);
        action = new IntakeAndLob(taken, lobbed);
    }

    /**
     * FERRY:
     * repeat:
     * * drive to the neutral zone
     * * pick up until full
     * * drive to our zone
     * * score
     */
    void ferry() {
        if (location == myZone) {
            score();
        } else {
            // we're in the neutral zone
            // for now let's ferry
            if (count >= BIN_CAPACITY) {
                // time to go back
                moveTo(myZone);
            } else {
                if (location.count() > MIN_ON_FLOOR_TO_INTAKE) {
                    intakeOnly();
                } else {
                    // nothing left here to pick up so go back
                    moveTo(myZone);
                }
            }
        }
    }

    void intakeOnly() {
        int intakeCount = Math.min(Math.max(0, BIN_CAPACITY - count), intakeRate());
        int taken = location.take(intakeCount);
        count += taken;
        action = new IntakeOnly(taken);
    }

    /**
     * * drive to the opposite zone
     * * block opponents if any
     * * steal and lob to our zone
     */
    void defendInOppositeZone() {
        if (location == myZone) {
            moveTo(neutralZone);
        } else if (location == neutralZone) {
            moveTo(otherZone);
        } else {
            for (Robot r : sim.robots()) {
                if (r.location == otherZone && r != this) {
                    action = new Block(r);
                    return;
                }
            }
            if (location.count() > MIN_ON_FLOOR_TO_INTAKE) {
                intakeAndLob();
            }
        }
    }

    void moveTo(Zone zone) {
        destination = zone;
        travelTimer = TRAVEL_TIME;
        action = new Move(zone.name);
    }

    int shootRate() {
        if (blocked)
            return SHOOT_RATE / 2;
        return SHOOT_RATE;
    }

    int shoot(BallAcceptor target) {
        int actual = Math.min(count, shootRate());
        target.accept(actual);
        count = count - actual;
        return actual;
    }

    public String header() {
        return "loc,  ct,             action";
    }

    @Override
    public String toString() {
        return String.format("%3s, %3d, %18s",
                location.name, count(), action);
    }

    @Override
    public int count() {
        return count;
    }
}
