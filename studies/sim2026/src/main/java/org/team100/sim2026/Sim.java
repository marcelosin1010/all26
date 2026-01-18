package org.team100.sim2026;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.team100.sim2026.actions.Block;
import org.team100.sim2026.robots.Defender;
import org.team100.sim2026.robots.ExampleRobot;
import org.team100.sim2026.robots.Robot;
import org.team100.sim2026.robots.Scorer;

/** The main simulation loop. */
public class Sim {
    // note 10 seconds longer this year
    public static final int MATCH_LENGTH_SEC = 160;

    private int matchTimer;

    // 504 total
    public final Zone redZone = new Zone("red", 24); // staged in the depot
    public final Zone neutralZone = new Zone("mid", 360);
    public final Zone blueZone = new Zone("blu", 24);// staged in the depot

    final Outpost redOutpost = new Outpost(redZone, 24);
    final Outpost blueOutpost = new Outpost(blueZone, 24);

    final Score redScore = new Score(this::phase);
    final Score blueScore = new Score(this::phase);

    public final Hub redHub = new Hub(redScore, neutralZone);
    public final Hub blueHub = new Hub(blueScore, neutralZone);

    public final Tower redTower = new Tower(redScore);
    public final Tower blueTower = new Tower(blueScore);

    ///////////////
    /// ALLIANCES
    ///
    /// TODO: make an alliance class
    /// TODO: make robot performance parameters

    // three-role alliance
    final Robot red1 = new Scorer(Alliance.RED, "r1", 8, this);
    final Robot red2 = new ExampleRobot(Alliance.RED, "r2", 8, this);
    final Robot red3 = new Defender(Alliance.RED, "r3", 8, this);

    // three-role alliance
    // final Robot blue1 = new Scorer(Alliance.BLUE, "b1", 8, this);
    // final Robot blue2 = new ExampleRobot(Alliance.BLUE, "b2", 8, this);
    // final Robot blue3 = new Defender(Alliance.BLUE, "b3", 8, this);

    // all-around alliance
    final Robot blue1 = new ExampleRobot(Alliance.BLUE, "b1", 8, this);
    final Robot blue2 = new ExampleRobot(Alliance.BLUE, "b2", 8, this);
    final Robot blue3 = new ExampleRobot(Alliance.BLUE, "b3", 8, this);

    final List<BallContainer> containers;
    final List<Actor> actors;
    final List<Robot> robots;

    // set after auto
    Alliance firstActive;

    public Sim() {
        containers = List.of(redZone, neutralZone, blueZone,
                redHub, blueHub,
                redOutpost, blueOutpost,
                red1, red2, red3, blue1, blue2, blue3);
        actors = List.of(redHub, blueHub,
                redOutpost, blueOutpost,
                red1, red2, red3, blue1, blue2, blue3);
        robots = List.of(red1, red2, red3, blue1, blue2, blue3);
        System.out.printf("initial total %d\n", total());
    }

    private int total() {
        return containers.stream().map(BallContainer::count).reduce(0, Integer::sum);
    }

    public void run() {
        header();
        for (matchTimer = 0; matchTimer < MATCH_LENGTH_SEC; ++matchTimer) {
            updateActiveHubs();
            // Ask each actor what they want to do, without doing anything.
            List<Runnable> actions = new ArrayList<>();
            for (Actor actor : actors) {
                actions.add(actor.step());
            }
            // Update interactions.
            // First clear the blocks.
            robots.stream().forEach(r -> r.blocked = false);
            // Then apply any blocks that exist.
            robots.stream().filter(r -> r.action.getClass() == Block.class)
                    .forEach(r -> ((Block) r.action).target.blocked = true);
            // Execute the actions in random order.
            Collections.shuffle(actions);
            for (Runnable runnable : actions) {
                runnable.run();
            }
            row();
        }
        score();
    }

    public List<Robot> robots() {
        return robots;
    }

    GamePhase phase() {
        return GamePhase.at(matchTimer);
    }

    public int time() {
        return matchTimer;
    }

    void red(boolean active) {
        redHub.active = active;
        red1.active = active;
        red2.active = active;
        red3.active = active;
    }

    void blue(boolean active) {
        blueHub.active = active;
        blue1.active = active;
        blue2.active = active;
        blue3.active = active;
    }

    void updateActiveHubs() {
        // choose which hub is active in the first shift
        if (matchTimer == 20) {
            // this should only happen once per match
            if (blueScore.autoFuel > redScore.autoFuel) {
                firstActive = Alliance.RED;
            } else if (blueScore.autoFuel < redScore.autoFuel) {
                firstActive = Alliance.BLUE;
            } else {
                firstActive = new Random().nextBoolean() ? Alliance.RED : Alliance.BLUE;
            }
        }
        // then set the active hubs
        switch (phase()) {
            case AUTO -> {
                red(true);
                blue(true);
            }
            case TRANSITION -> {
                red(true);
                blue(true);
            }
            case SHIFT_1 -> {
                boolean redActive = firstActive == Alliance.RED;
                red(redActive);
                blue(!redActive);
            }
            case SHIFT_2 -> {
                boolean redActive = firstActive == Alliance.BLUE;
                red(redActive);
                blue(!redActive);
            }
            case SHIFT_3 -> {
                boolean redActive = firstActive == Alliance.RED;
                red(redActive);
                blue(!redActive);
            }
            case SHIFT_4 -> {
                boolean redActive = firstActive == Alliance.BLUE;
                red(redActive);
                blue(!redActive);
            }
            case END_GAME -> {
                red(true);
                blue(true);
            }
        }
    }

    /** print the ball location header */
    void header() {
        System.out.print(
                "            |---SCORE----|--------ZONE--------|----HUB----|--OUTPOST--");
        System.out.print("|----ACTIVE----");
        System.out.print(
                "|------------RED 1-------------|------------RED 2-------------|------------RED 3-------------");
        System.out.print(
                "|------------BLUE 1------------|------------BLUE 2------------|------------BLUE 3------------|\n");
        System.out.print(
                "time, balls |  red, blue | red, neutral, blue | red, blue | red, blue ");
        System.out.print("|  red,  blue  ");
        System.out.print(
                "| " + red1.header() + " | " + red2.header() + " | " + red3.header() + " | "
                        + blue1.header() + " | " + blue1.header() + " | " + blue1.header() + " |\n");
        System.out.println(
                "--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
    }

    /** print the states and events */
    void row() {
        System.out.printf("%4d, %5d | %4d, %4d | %3d, %7d, %4d | %3d, %4d | %3d, %4d ",
                matchTimer, total(), redScore.total(), blueScore.total(),
                redZone.count(), neutralZone.count(), blueZone.count(),
                redHub.count(), blueHub.count(),
                redOutpost.count(), blueOutpost.count());
        System.out.printf("| %5b, %5b ",
                redHub.active, blueHub.active);

        System.out.printf("| %s | %s | %s | %s | %s | %s |\n",
                red1, red2, red3, blue1, blue2, blue3);

    }

    void score() {
        System.out.printf("RED\n  %s\nBLUE\n  %s\n", redScore, blueScore);
    }

}
