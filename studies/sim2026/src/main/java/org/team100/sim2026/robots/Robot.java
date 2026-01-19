package org.team100.sim2026.robots;

import org.team100.sim2026.Actor;
import org.team100.sim2026.AllianceColor;
import org.team100.sim2026.BallAcceptor;
import org.team100.sim2026.BallContainer;
import org.team100.sim2026.Hub;
import org.team100.sim2026.SimRun;
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
    /** between adjacent zones */
    public static final int TRAVEL_TIME = 3;
    /** don't hunt for balls when there aren't very many */
    private static final int MIN_ON_FLOOR_TO_INTAKE = 5;
    /** The last few take longer to shoot so don't bother */
    private static final int MIN_COUNT_TO_SHOOT = 5;
    static final int CLIMB_TIME = 5;
    static final int CLIMB_BUFFER = 10;

    final SimRun sim;
    final AllianceColor alliance;

    public final String name;
    private final int capacity;
    private final int intakeRate;
    private final int shootRate;

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

    /** another robot is blocking us, which slows everything down. */
    public boolean blocked;

    public Robot(
            AllianceColor alliance,
            String name,
            int capacity,
            int intakeRate,
            int shootRate,
            int initialCount,
            SimRun sim) {
        this.alliance = alliance;
        this.sim = sim;
        this.name = name;
        this.capacity = capacity;
        this.intakeRate = intakeRate;
        this.shootRate = shootRate;
        this.myZone = alliance == AllianceColor.RED ? sim.redZone : sim.blueZone;
        this.neutralZone = sim.neutralZone;
        this.otherZone = alliance == AllianceColor.RED ? sim.blueZone : sim.redZone;
        this.myHub = alliance == AllianceColor.RED ? sim.redHub : sim.blueHub;
        this.myTower = alliance == AllianceColor.RED ? sim.redTower : sim.blueTower;
        // initial location is my own zone
        this.location = myZone;
        this.count = initialCount;
    }

    /** Decide what to do during this auton period. */
    abstract void auton();

    /** Decide what do to when active. */
    abstract void active();

    /** Decide what do to when inactive. */
    abstract void inactive();

    @Override
    public Runnable step() {
        return () -> {
            if (stillClimbing()) {
                return;
            }
            if (stillMoving()) {
                return;
            }
            if (sim.time() < 20) {
                auton();
                return;
            }
            if (endgame()) {
                return;
            }
            if (active) {
                active();
                return;
            }
            inactive();
        };
    }

    /** For now, all climb behaviors are the same. */
    boolean endgame() {
        if (shouldGoFromOtherZoneToNeutralToClimb()) {
            moveTo(neutralZone);
            return true;
        } else if (shouldGoFromNeutralToOurZoneToClimb()) {
            moveTo(myZone);
            return true;
        } else if (shouldClimb()) {
            climb();
            return true;
        }
        return false;
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
        int rate = intakeRate;
        int count = location.count();
        if (count < 2 * MIN_ON_FLOOR_TO_INTAKE) {
            // as the floor density goes down, intake takes longer
            rate = rate / 2;
        }
        if (blocked) {
            // defense slows down intaking
            return rate / 2;
        }
        return rate;
    }

    /** Intake and shoot simultaneously. */
    void intakeAndScore() {
        if (location != myZone)
            throw new IllegalStateException();
        int intakeCount = Math.min(Math.max(0, capacity - count), intakeRate());
        int taken = location.take(intakeCount);
        count += taken;
        int actual = shoot(myHub);
        action = new IntakeAndScore(taken, actual);
    }

    /**
     * LOB:
     * * move to the neutral zone
     * * lob balls from there into our zone
     * * when exhausted and active, drive to our zone and score
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
                } else if (active) {
                    // nothing on the floor, nothing in the bin, can score
                    moveTo(myZone);
                } else {
                    // wait
                }
            }
        }
    }

    /**
     * LOB ONLY:
     * * lob without scoring
     */
    void lobOnly() {
        if (location != neutralZone) {
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
        int intakeCount = Math.min(Math.max(0, capacity - count), intakeRate());
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
    void ferryAndScore() {
        if (location == myZone) {
            score();
        } else {
            // we're in the neutral zone
            // for now let's ferry
            if (count >= capacity) {
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
        int intakeCount = Math.min(Math.max(0, capacity - count), intakeRate());
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

    void climb() {
        if (action.getClass() == Climb.class) {
            // already climbing
            return;
        }
        action = new Climb(myTower, CLIMB_TIME);
    }

    void moveTo(Zone zone) {
        destination = zone;
        travelTimer = TRAVEL_TIME;
        action = new Move(zone.name);
    }

    int shootRate() {
        if (blocked)
            return shootRate / 2;
        return shootRate;
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

    boolean shouldGoFromOtherZoneToNeutralToClimb() {
        return location == otherZone && sim.time() >= farGoClimbDeadline();
    }

    boolean shouldGoFromNeutralToOurZoneToClimb() {
        return location == neutralZone && sim.time() >= goClimbDeadline();
    }

    boolean shouldClimb() {
        return location == myZone && sim.time() >= climbDeadline();
    }

    /**
     * When we should drive from the neutral zone to our zone, to prepare to climb.
     */
    private int goClimbDeadline() {
        return SimRun.MATCH_LENGTH_SEC - (CLIMB_BUFFER + CLIMB_TIME + TRAVEL_TIME);
    }

    /** When we should start climbing. */
    private int climbDeadline() {
        return SimRun.MATCH_LENGTH_SEC - (CLIMB_BUFFER + CLIMB_TIME);
    }

    /**
     * When we should drive from other zone to the neutral zone, to prepare to
     * climb.
     */
    private int farGoClimbDeadline() {
        return SimRun.MATCH_LENGTH_SEC - (CLIMB_BUFFER + CLIMB_TIME + 2 * TRAVEL_TIME);
    }

}
