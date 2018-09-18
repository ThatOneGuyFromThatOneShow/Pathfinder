package jcrane.pathfinder.jaci;

import jcrane.pathfinder.nodes.Node;

public class MakeJaciWaypoint {

    //Assumes starting angle of 0
    public static JaciWaypoint[] pathToJaciWaypoints(Node[] nodes, double endingAngle) {
        JaciWaypoint[] JaciWaypoints = new JaciWaypoint[nodes.length-1];

        Node firstNode = nodes[0];

        for (int i = 1; i < nodes.length - 1; i++) {
            int nextX = nodes[i + 1].getX();
            int nextY = nodes[i + 1].getY();
            int curX = nodes[i].getX();
            int curY = nodes[i].getY();
            double deltaAngle = Math.atan2(nextX - curX, nextY - curY);

            //FIXME must insure x and y are inches before turning them into meteres
            JaciWaypoints[i-1] = new JaciWaypoint((curY - firstNode.getY()) * 0.3048, (curX - firstNode.getX()) * 0.3048, -deltaAngle);
        }
        endingAngle = endingAngle > 180 ? endingAngle - 360 : endingAngle;
        endingAngle = -Math.toRadians(endingAngle);
        JaciWaypoints[JaciWaypoints.length-1] = new JaciWaypoint((nodes[nodes.length-1].getY() - firstNode.getY()) * 0.3048, (nodes[nodes.length-1].getX() - firstNode.getX()) * 0.3048, endingAngle);

        return JaciWaypoints;
    }
}
