package org.team100.lib.trajectory;

import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.XYSeries;
import org.team100.lib.trajectory.path.PathSE2Point;
import org.team100.lib.trajectory.path.PathUtil;
import org.team100.lib.trajectory.path.PathSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathToVectorSeries {
    private static final boolean DEBUG = false;
    /** Length of the vector indicating heading */
    private final double m_scale;

    public PathToVectorSeries(double scale) {
        m_scale = scale;
    }

    /** Maps x to x, y to y */
    public List<VectorSeries> convert(PathSE2 path) {
        VectorSeries s = new VectorSeries("path");
        double l = path.distance(path.length() - 1);
        double dl = l / 20;
        for (double d = 0; d < l; d += dl) {
            if (DEBUG)
                System.out.printf("%f\n", d);
            Pose2d p = PathUtil.sample(path, d).waypoint().pose();
            double x = p.getX();
            double y = p.getY();
            Rotation2d heading = p.getRotation();
            double dx = m_scale * heading.getCos();
            double dy = m_scale * heading.getSin();
            s.add(x, y, dx, dy);
        }
        return List.of(s);
    }

    public List<VectorSeries> curvature(PathSE2 path) {
        VectorSeries s = new VectorSeries("path");
        double l = path.distance(path.length() - 1);
        double dl = l / 20;
        for (double d = 0; d < l; d += dl) {
            Pose2d p = PathUtil.sample(path, d).waypoint().pose();
            double x = p.getX();
            double y = p.getY();

        }
        return List.of(s);
    }

    public static XYSeries x(String name, List<PathSE2Point> poses) {
        XYSeries series = new XYSeries(name);
        for (int i = 0; i < poses.size(); ++i) {
            PathSE2Point pose = poses.get(i);
            series.add(i, pose.waypoint().pose().getX());
        }
        return series;
    }

}
