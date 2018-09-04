package jcrane.pathfinder.frc;

import java.util.Map;

import jcrane.pathfinder.aStar.AStarResult;
import jcrane.pathfinder.nodemaps.NodeMap;
import jcrane.pathfinder.nodes.Node;

/**
 * A basic class for holding and using information returned from an A* FRCPathFinder search.
 */
public class FRCResult extends AStarResult {

    /**
     * This object should NOT be created manually!
     *
     * @param nodeTrace  The map of every node and what node it came from.
     * @param costOfNode The map of the cost to move to any node.
     * @param startNode  The node the search originated from.
     * @param goalNode   The node the search ended on.
     * @param lastNode   The last node reached by the algorithm.
     */
    public FRCResult(NodeMap nodeMap, Map<Node, Node> nodeTrace, Map<Node, Double> costOfNode, Node startNode, Node goalNode, Node lastNode) {
        super(nodeMap, nodeTrace, costOfNode, startNode, goalNode, lastNode);
    }

    private double toDegrees(double rad) {
        rad *= -1;
        return rad >= 0 ? Math.toDegrees(rad) : 360 + Math.toDegrees(rad);
    }

    /**
     * Turns a Node array into a Waypoint array. Essentially this just adds a target angle to each Node.
     * This can be used with the MotorOutput class.
     *
     * @param nodes         a Node array.
     * @param startingAngle The starting angle of the robot, relative to the NodeMap.
     * @return A Waypoint array.
     */
    public Waypoint[] pathToWaypoints(Node[] nodes, double startingAngle) {
        Waypoint[] waypoints = new Waypoint[nodes.length];

        Node firstNode = nodes[0];
        waypoints[0] = new Waypoint(firstNode.getX(), firstNode.getY(), startingAngle);

        for (int i = 1; i < nodes.length; i++) {
            int lastX = nodes[i - 1].getX();
            int lastY = nodes[i - 1].getY();
            int curX = nodes[i].getX();
            int curY = nodes[i].getY();
            double deltaAngle = (toDegrees(Math.atan2(curX - lastX, curY - lastY)) + startingAngle) % 360;

            waypoints[i] = new Waypoint(curX, curY, deltaAngle);
        }

        return waypoints;
    }

    /**
     * Creates a new path from a completed aStarSearch and turns it into a Waypoint array.
     * This is note a straight line path!
     * To Generate a straight line path you must generate it using makeStraightLinePath!
     * <p>
     * This can be used with the MotorOutput class.
     *
     * @param startingAngle The starting angle of the robot, relative to the NodeMap.
     * @return A Waypoiny array.
     */
    public Waypoint[] pathToWaypoints(double startingAngle) {
        return pathToWaypoints(makePath(), startingAngle);
    }
}
