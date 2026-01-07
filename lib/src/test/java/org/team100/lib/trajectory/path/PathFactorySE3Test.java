package org.team100.lib.trajectory.path;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE3;
import org.team100.lib.geometry.WaypointSE3;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PathFactorySE3Test {
    private static final double DELTA = 0.001;

    @Test
    void testLinear() {
        List<WaypointSE3> waypoints = List.of(
                new WaypointSE3(
                        new Pose3d(
                                new Translation3d(),
                                new Rotation3d()),
                        new DirectionSE3(1, 0, 0, 0, 0, 0), 1),
                new WaypointSE3(
                        new Pose3d(
                                new Translation3d(1, 0, 0),
                                new Rotation3d()),
                        new DirectionSE3(1, 0, 0, 0, 0, 0), 1));
        PathFactorySE3 pathFactory = new PathFactorySE3(0.1, 0.01, 0.1);
        PathSE3 path = pathFactory.fromWaypoints(waypoints);
        assertEquals(17, path.length());
        PathPointSE3 p = path.getPoint(0);
        assertEquals(0, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getAngle(), DELTA);
        assertEquals(0, p.headingRate().norm(), DELTA);
        p = path.getPoint(8);
        assertEquals(0.5, p.waypoint().pose().getTranslation().getX(), DELTA);
        assertEquals(0, p.waypoint().pose().getRotation().getAngle(), DELTA);
        assertEquals(0, p.headingRate().norm(), DELTA);
    }

}
