package org.team100.lib.state;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.trajectory.path.PathSE2Point;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;

public class ControlSE2Test {
    @Test
    void test0() {
        // moving +x
        WaypointSE2 waypoint = new WaypointSE2(new Pose2d(), new DirectionSE2(1, 0, 0), 1);
        // positive curvature
        PathSE2Point point = new PathSE2Point(waypoint, 0, VecBuilder.fill(0, 1));
        // moving at 1 m/s
        ControlSE2 control = ControlSE2.fromMovingPathSE2Point(point, 1, 0);
        assertEquals(0, control.x().a(), 0.001);
        // accelerating to the left
        assertEquals(1, control.y().a(), 0.001);
    }

    @Test
    void test1() {
        // moving +x
        WaypointSE2 waypoint = new WaypointSE2(new Pose2d(), new DirectionSE2(1, 0, 0), 1);
        // negative curvature
        PathSE2Point point = new PathSE2Point(waypoint, 0, VecBuilder.fill(0, -1));
        // moving at 1 m/s
        ControlSE2 control = ControlSE2.fromMovingPathSE2Point(point, 1, 0);
        assertEquals(0, control.x().a(), 0.001);
        // accelerating to the right
        assertEquals(-1, control.y().a(), 0.001);
    }
}
