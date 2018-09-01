package main.aStar;

import java.util.ArrayList;
import java.util.Collections;
import main.nodemaps.NodeMap;
import main.nodemaps.WidthMap;
import main.nodes.Node;

import java.util.Map;

/**
 * A basic class for holding and using information returned from an A* search.
 */
public class AStarResult {
    private Map<Node, Node> nodeTrace;
    private Map<Node, Double> costOfNode;
    private Node startNode, goalNode, lastNode;
    private NodeMap nodeMap;

    /**
     * This object should NOT be created manually!
     *
     * @param nodeTrace The map of every node and what node it came from.
     * @param costOfNode The map of the cost to move to any node.
     * @param startNode The node the search originated from.
     * @param goalNode The node the search ended on.
     * @param lastNode The last node reached by the algorithm.
     */
    public AStarResult(NodeMap nodeMap, Map<Node, Node> nodeTrace, Map<Node, Double> costOfNode, Node startNode, Node goalNode, Node lastNode) {
        this.nodeTrace = nodeTrace;
        this.costOfNode = costOfNode;
        this.startNode = startNode;
        this.goalNode = goalNode;
        this.lastNode = lastNode;
        this.nodeMap = nodeMap;
    }

    /**
     * Returns a map of every node and what node it came from.
     *
     * @return Returns a node trace map.
     */
    public Map<Node, Node> getNodeTrace() {
        return nodeTrace;
    }

    /**
     * Returns a map of every node and the cost to get to it.
     *
     * @return A map of every node to cost.
     */
    public Map<Node, Double> getCostOfNode() {
        return costOfNode;
    }

    /**
     * Returns the node at which the A* search originated from.
     *
     * @return A* start node.
     */
    public Node getStartNode() {
        return startNode;
    }

    /**
     * Returns the node which the A* search was trying to reach.
     *
     * @return A* goal node.
     */
    public Node getGoalNode() {
        return goalNode;
    }

    /**
     * Returns the last node reached by the algorithm.
     *
     * @return The last node.
     */
    public Node getLastNode() {
        return lastNode;
    }

    public NodeMap getNodeMap() {
        return nodeMap;
    }

    public boolean failed() {
        return !getGoalNode().equals(getLastNode());
    }

    /**
     * Takes the results from an A* search and reconstructs it into a usable node path array.
     *
     * @return An array path with [0] being the start and the last index being the goal.
     */
    public Node[] makePath() {
        ArrayList<Node> output = new ArrayList<>();
        Map<Node, Node> nodeTrace = getNodeTrace();

        Node startNode = getStartNode();
        Node currentNode = getGoalNode();

        output.add(currentNode);
        while (!currentNode.equals(startNode)) {
            currentNode = nodeTrace.get(currentNode);
            output.add(currentNode);
        }

        Collections.reverse(output);
        return output.toArray(new Node[output.size()]);
    }

    /**
     * Takes a path array and turns it into a waypoint array. A waypoint array is the same as a path but with only two points per line.
     *
     * @param path A path Node array.
     * @return A waypoint Node array.
     */
    public Node[] makeStraightLinePath(Node[] path) {

        ArrayList<Node> nodes = new ArrayList<>();
        nodes.add(path[0]);

        for (int i = 1; i < path.length-1; i++) {
            int x1 = path[i-1].getX();
            int y1 = path[i-1].getY();
            int x2 = path[i].getX();
            int y2 = path[i].getY();
            int x3 = path[i+1].getX();
            int y3 = path[i+1].getY();

            if (Math.atan2(y3 - y2, x3 - x2) != Math.atan2(y2 - y1, x2 - x1))
                nodes.add(path[i]);
        }
        nodes.add(path[path.length-1]);

        return nodes.toArray(new Node[nodes.size()]);
    }

    public Node[] makeWaypontPath() {
        if (nodeMap instanceof WidthMap)
            return makeStraightLinePath(makePath(), ((WidthMap) nodeMap).getActualWidth());
        else
            return makeStraightLinePath(makePath());
    }

    /**
     * Takes a path and actual width and produces a point to point straight line trajectory.
     *
     * @param path A node path as outputted from makePath().
     * @param actualWidth The actual width.
     * @return A waypoint trajectory.
     */
    public Node[] makeStraightLinePath(Node[] path, int actualWidth) {
        //Removes nodes that are directly next to each other and are in a straight line.
        //On a high resolution bitmap this helps reduce the work by the line segment identifier.
        ArrayList<Node> nodes = new ArrayList<>();
        nodes.add(path[0]);
        for (int i = 1; i < path.length-1; i++) {
            int x1 = path[i-1].getX();
            int y1 = path[i-1].getY();
            int x2 = path[i].getX();
            int y2 = path[i].getY();
            int x3 = path[i+1].getX();
            int y3 = path[i+1].getY();

            if (Math.atan2(y3 - y2, x3 - x2) != Math.atan2(y2 - y1, x2 - x1))
                nodes.add(path[i]);
        }
        nodes.add(path[path.length-1]);


        //Compute a series of lines that are a minimum of half actualWidth away from any occupied nodes.
        int lastIndexOfSegment = 0;
        ArrayList<Node> lastNodes = new ArrayList<>();
        lastNodes.add(nodes.get(0));

        //Takes the first node then draws a line to the 2nd, then 3rd, etc. Once the line collides with an occupied spot
        // it backs up one (ensuring the line has no collision), and saves the starting node of the segment.
        // The next segment starts where the previous one ended. Repeats until the last node is the end node.

        //Loop through starting nodes, the starting node is equal to the last node of the previous segment.
        while (true) {
            if (lastIndexOfSegment == nodes.size() - 1)
                break;

            Node startingNode = nodes.get(lastIndexOfSegment);
            Node currentNode;

            //Make a line from startingNode to currentNode. Each iteration moves current node to the next node in nodes.
            boolean shouldContinue = true;
            while (shouldContinue) {
                if (lastIndexOfSegment == nodes.size() - 1)
                    break;
                currentNode = nodes.get(lastIndexOfSegment + 1);

                boolean infSlope = false;
                int slope = 0;
                if (startingNode.getX() == currentNode.getX())
                    infSlope = true;
                else
                    slope = (startingNode.getY() - currentNode.getY()) / (startingNode.getX() - currentNode.getX());

                int xIntercept = startingNode.getY() - (slope * startingNode.getX());

                //If slope is infinite iterate by y instead of x.
                int minItr = (infSlope) ? Math.min(currentNode.getY(), startingNode.getY()) : Math.min(currentNode.getX(), startingNode.getX());
                int maxItr = (infSlope) ? Math.max(currentNode.getY(), startingNode.getY()) : Math.max(currentNode.getX(), startingNode.getX());

                //Check collision on every point on the line startingNode-currentNode
                for (int itr = minItr; itr < maxItr; itr++) {
                    //Calculate x and y based on the slope and x-intercept.
                    int x, y;
                    if (!infSlope) {
                        x = itr;
                        y = (slope * x) + xIntercept;
                    } else if (Math.abs(slope) == 0) {
                        x = itr;
                        y = startingNode.getY();
                    } else {
                        x = startingNode.getX();
                        y = itr;
                    }

                    //check collision on point.
                    int safeWidthCorner = (int) Math.ceil(Math.sqrt(2) * actualWidth / 4);
                    Node tn0 = new Node(x + safeWidthCorner, y + safeWidthCorner);
                    Node tn1 = new Node(x + safeWidthCorner, y - safeWidthCorner);
                    Node tn2 = new Node(x - safeWidthCorner, y + safeWidthCorner);
                    Node tn3 = new Node(x - safeWidthCorner, y - safeWidthCorner);

                    Node tn4 = new Node(x, y + (actualWidth / 2));
                    Node tn5 = new Node(x + (actualWidth / 2), y);
                    Node tn6 = new Node(x, y - (actualWidth / 2));
                    Node tn7 = new Node(x - (actualWidth / 2), y);

                    boolean collision = !(!nodeMap.isOccupied(tn0) && !nodeMap.isOccupied(tn1) && !nodeMap.isOccupied(tn2)
                        && !nodeMap.isOccupied(tn3) && !nodeMap.isOccupied(tn4) && !nodeMap.isOccupied(tn5)
                        && !nodeMap.isOccupied(tn6) && !nodeMap.isOccupied(tn7));

                    //If collision use the last collision free node (previous node) as the end point of the segment.
                    if (collision) {
                        lastNodes.add(nodes.get(lastIndexOfSegment));
                        shouldContinue = false;
                        break;
                    }
                }
                lastIndexOfSegment++;
            }

        }
        lastNodes.add(nodes.get(nodes.size()-1));

        return lastNodes.toArray(new Node[lastNodes.size()]);
    }

    public Node[] makeStraightLinePath(int actualWidth) {
        return makeStraightLinePath(makePath(), actualWidth);
    }
}
