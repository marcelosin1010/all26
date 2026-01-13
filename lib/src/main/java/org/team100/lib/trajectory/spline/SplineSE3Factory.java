package org.team100.lib.trajectory.spline;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.geometry.WaypointSE3;

public class SplineSE3Factory {

    /**
     * Make N-1 splines from N waypoint knots.
     */
    public static List<SplineSE3> splinesFromWaypoints(List<WaypointSE3> waypoints) {
        List<SplineSE3> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i0 = 0; i0 < waypoints.size() - 1; ++i0) {
            int i1 = i0 + 1;
            splines.add(new SplineSE3(waypoints.get(i0), waypoints.get(i1)));
        }
        return splines;
    }

}
