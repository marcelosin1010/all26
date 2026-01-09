package org.team100.lib.trajectory.spline;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class OffsetSE2Test {
    @Test
    void test0() {
        WaypointSE2 w0 = new WaypointSE2(
                new Pose2d(new Translation2d(), new Rotation2d(Math.PI / 2)),
                new DirectionSE2(1, 0, -1), 1);
        WaypointSE2 w1 = new WaypointSE2(
                new Pose2d(new Translation2d(Math.PI / 2, 0), new Rotation2d(0)),
                new DirectionSE2(1, 0, -1), 1);
        SplineSE2 toolpoint = new SplineSE2(w0, w1);
        double length = 1.0;
        OffsetSE2 offset = new OffsetSE2(toolpoint, length);
        SplineSE2ToVectorSeries splineConverter = new SplineSE2ToVectorSeries(0.1);
        // List<VectorSeries> series = splineConverter.convert(List.of(offset));
        // ChartUtil.plotOverlay(series, 500);
    }

}
