package org.team100.lib.trajectory;

import java.util.List;

import org.jfree.data.xy.VectorSeries;
import org.jfree.data.xy.XYSeries;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.state.ControlSE2;

import edu.wpi.first.math.geometry.Rotation2d;

public class TrajectorySE2ToVectorSeries {
    private static final boolean DEBUG = false;

    private static final int POINTS = 50;
    /** Length of the vector indicating heading */
    private final double m_scale;

    public TrajectorySE2ToVectorSeries(double scale) {
        m_scale = scale;
    }

    /** Maps x to x, y to y */
    public List<VectorSeries> convert(TrajectorySE2 t) {
        VectorSeries s = new VectorSeries("trajectory");
        double duration = t.duration();
        if (DEBUG)
            System.out.printf("duration %f\n", duration);
        double dt = duration / POINTS;
        for (double time = 0; time < duration; time += dt) {
            TrajectorySE2Entry p = t.sample(time);
            WaypointSE2 pp = p.point().point().waypoint();
            double x = pp.pose().getTranslation().getX();
            double y = pp.pose().getTranslation().getY();
            Rotation2d heading = pp.pose().getRotation();
            double dx = m_scale * heading.getCos();
            double dy = m_scale * heading.getSin();
            s.add(x, y, dx, dy);
            if (DEBUG)
                System.out.printf("%f\n", time);
        }
        return List.of(s);
    }

    public List<VectorSeries> accel(TrajectorySE2 trajectory) {
        VectorSeries series = new VectorSeries("trajectory");
        double duration = trajectory.duration();
        double dt = duration / POINTS;
        for (double time = 0; time < duration; time += dt) {
            TrajectorySE2Point point = trajectory.sample(time).point();
            ControlSE2 control = ControlSE2.fromTrajectorySE2Point(point);
            double x = control.x().x();
            double y = control.y().x();
            double ax = m_scale * control.x().a();
            double ay = m_scale * control.y().a();
            series.add(x, y, ax, ay);
        }
        return List.of(series);
    }

    /**
     * X as a function of t.
     * 
     * @return (t, x)
     */
    public static XYSeries x(String name, TrajectorySE2 trajectory) {
        XYSeries series = new XYSeries(name);
        double duration = trajectory.duration();
        double dt = duration / POINTS;
        for (double t = 0; t <= duration + 0.0001; t += dt) {
            TrajectorySE2Entry p = trajectory.sample(t);
            WaypointSE2 pp = p.point().point().waypoint();
            double x = pp.pose().getTranslation().getX();
            series.add(t, x);
        }
        return series;
    }

    /**
     * X dot: dx/dt, as a function of t.
     * 
     * @return (t, \dot{x})
     */
    public static XYSeries xdot(String name, TrajectorySE2 trajectory) {
        XYSeries series = new XYSeries(name);
        double duration = trajectory.duration();
        double dt = duration / POINTS;
        for (double t = 0; t <= duration + 0.0001; t += dt) {
            TrajectorySE2Entry p = trajectory.sample(t);
            Rotation2d course = p.point().point().waypoint().course().toRotation();
            double velocityM_s = p.point().velocity();
            double xv = course.getCos() * velocityM_s;
            series.add(t, xv);
        }
        return series;
    }
}
