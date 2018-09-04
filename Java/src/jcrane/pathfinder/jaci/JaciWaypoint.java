package jcrane.pathfinder.jaci;

import jaci.pathfinder.Waypoint;

public class JaciWaypoint extends Waypoint {
    /**
     * Create a Waypoint (setpoint) for Trajectory Generation
     * @param x         The X position of the waypoint in meters
     * @param y         The Y position of the waypoint in meters
     * @param angle     The exit angle of the waypoint in radians
     */
    public JaciWaypoint(double x, double y, double angle) {
        super(x, y, angle);
    }

}
