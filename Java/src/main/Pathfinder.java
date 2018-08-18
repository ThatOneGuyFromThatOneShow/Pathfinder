package main;

import main.aStar.AStarResult;
import main.nodemaps.NodeMap;
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
        //A map every where every node came from
        Map<Node, Node> nodeTrace = new HashMap<>();
        //A map of the cost to get to a node, if you used the current path
        Map<Node, Double> costOfNode = new HashMap<>();
        //A queue of nodes to check with a priority. Priority is how likely it will be that a path will get to the end the fastest. Lower priority means it will be checked first.
        PriorityQueue<PriorityNode> queue = new PriorityQueue<>(new NodeComparatorLowPriority());

        //Add the start node to the PriorityQueue and add a cost of 0.
        Node currentNode = start;
        nodeTrace.put(start, null);
        costOfNode.put(start, 0.0);
        queue.add(new PriorityNode(start, 0));

        //Loop through checking the node at the top of the queue until there are no more nodes in the queue. Or the current node pulled from the queue equals the goal node.
        while (!queue.isEmpty()) {
            currentNode = queue.poll().toNode();

            if (currentNode.equals(goal))
                break;

            //Gets all the neighbors of the current node, and sames them into nextNode.
            for (Node nextNode : nodeMap.getNeighbors(currentNode)) {
                double cost = costOfNode.get(currentNode) + nodeMap.calculateCost(currentNode, nextNode); //Calculates the cost of getting to nextNode.
                //If next node is currently not in the cost map, or the computed cost is less the saved cost.
                if (!costOfNode.containsKey(nextNode) || cost < costOfNode.get(nextNode)) {
                    costOfNode.put(nextNode, cost); //Save the cost
                    nodeTrace.put(nextNode, currentNode); //Save where next node came from. Aka currentNode.
                    double priority = cost + heuristic(currentNode, nextNode) + nodeMap.getValueFromNode(nextNode)/2; //Calculate the priority. Cost + distance from goal + priority value from nodeMap.
                    queue.add(new PriorityNode(nextNode, priority)); //Adds the current node to the PriorityQueue.
                }
            }
        }

        //Return the result saved in a AStarResult object.
        return new AStarResult(nodeTrace, costOfNode, start, goal, currentNode);
    }

    //TODO refactor the pathMaking methods into AStarResult. Because they can only be used after generating a result it makes more sense for them to be in the result class. This would also make it more intuitive for the user.

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
