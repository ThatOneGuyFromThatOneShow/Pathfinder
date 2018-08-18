package main;

import jdk.nashorn.internal.ir.WhileNode;
import main.aStar.AStarResult;
import main.nodemaps.NodeMap;
import main.nodemaps.WidthImageNodeMap;
import main.nodes.Node;
import main.nodes.NodeComparatorLowPriority;
import main.nodes.PriorityNode;

import java.util.*;

/**
 * Pathfinder uses an A* algorithm to finder the shortest route from a starting location to a goal location, while avoiding obstacles.
 * This algorithm moves one Node at a time so it is not 100% the fastest route.
 *
 * Future update should provide a method for refining the outputted path more to straighten lines.
 */
public class Pathfinder {
    private NodeMap nodeMap;

    public Pathfinder(NodeMap nodeMap) {
        this.nodeMap = nodeMap;
    }

    public Pathfinder() {

    }

    /**
     * Set the NodeMap reference for the pathfinder algorithm to use.
     *
     * @param nodeMap The map of surroundings.
     */
    public void setNodeMap(NodeMap nodeMap) {
        this.nodeMap = nodeMap;
    }

    /**
     * Calculates the heuristic distance between two nodes.
     *
     * @param a Node a.
     * @param b Node b.
     * @return Heuristic distance.
     */
    public double heuristic(Node a, Node b) {
        return Math.abs(a.getX() - b.getX()) + Math.abs(a.getY() - b.getY());
    }

    /**
     * Calculates a path avoiding obstacles from the start node to the goal node.
     * This is done by stepping in each direction and checking out distance from the start and distance to the goal,
     * and assigning a priority of if this path will likely be fruitful.
     *
     * @param start The starting node.
     * @param goal The goal node.
     * @return an AStartResult object.
     */
    public AStarResult aStarSearch(Node start, Node goal) {
        Map<Node, Node> nodeTrace = new HashMap<>();
        Map<Node, Double> costOfNode = new HashMap<>();
        PriorityQueue<PriorityNode> queue = new PriorityQueue<>(new NodeComparatorLowPriority());

        Node currentNode = start;
        nodeTrace.put(start, null);
        costOfNode.put(start, 0.0);
        queue.add(new PriorityNode(start, 0));

        while (!queue.isEmpty()) {
            currentNode = queue.poll().toNode();

            if (currentNode.equals(goal))
                break;

            for (Node nextNode : nodeMap.getNeighbors(currentNode)) {
                double cost = costOfNode.get(currentNode) + nodeMap.calculateCost(currentNode, nextNode);
                if (!costOfNode.containsKey(nextNode) || cost < costOfNode.get(nextNode)) {
                    costOfNode.put(nextNode, cost);
                    nodeTrace.put(nextNode, currentNode);
                    double priority = cost + heuristic(currentNode, nextNode) + nodeMap.getValueFromNode(nextNode)/2;
                    queue.add(new PriorityNode(nextNode, priority));
                }
            }
        }

        return new AStarResult(nodeTrace, costOfNode, start, goal, currentNode);
    }

    /**
     * Takes the results from an A* search and reconstructs it into a usable node path array.
     *
     * @param result The A* search result.
     * @return An array path with [0] being the start and the last index being the goal.
     */
    public Node[] makePath(AStarResult result) {
        ArrayList<Node> output = new ArrayList<>();
        Map<Node, Node> nodeTrace = result.getNodeTrace();

        Node startNode = result.getStartNode();
        Node currentNode = result.getGoalNode();

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
    public Node[] makeWaypointPath(Node[] path) {
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

    /**
     * Takes a path and actual width and produces a point to point straight line trajectory.
     *
     * @param path A node path as outputted from makePath().
     * @param actualWidth The actual width.
     * @return A waypoint trajectory.
     */
    public Node[] makeWaypointPath(Node[] path, int actualWidth) {

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


        //split nodes into segments of nodes
        int lastIndexOfSegment = 0;
        ArrayList<Node> lastNodes = new ArrayList<>();
        lastNodes.add(nodes.get(0));
        while (true) {
            if (lastIndexOfSegment == nodes.size() - 1)
                break;

            Node startingNode = nodes.get(lastIndexOfSegment);
            Node currentNode;

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

                int minItr = (infSlope) ? Math.min(currentNode.getY(), startingNode.getY()) : Math.min(currentNode.getX(), startingNode.getX());
                int maxItr = (infSlope) ? Math.max(currentNode.getY(), startingNode.getY()) : Math.max(currentNode.getX(), startingNode.getX());
                for (int itr = minItr; itr < maxItr; itr++) {
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

                    //check collision on point. If collision refer to the previous point.
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

    /**
     * Combines both makepath and makeWaypointPath into one method. This method takes the data from an A* search and reconstructs it into a waypoint path, such that there's only two points per line.
     *
     * @param result The A* search result.
     * @return A waypoint Node path array.
     */
    public Node[] makeWaypointPath(AStarResult result) {
        return makeWaypointPath(makePath(result));
    }
}
