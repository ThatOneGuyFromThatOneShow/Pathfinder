package main;

import main.aStar.AStarResult;
import main.nodemaps.NodeMap;
import main.nodes.Node;
import main.nodes.NodeComparatorLowPriority;
import main.nodes.PriorityNode;

import java.util.*;

/**
 * PathFinder uses an A* algorithm to find the shortest route from a starting location to a goal location, while avoiding obstacles.
 * This algorithm moves one Node at a time so it is not 100% the fastest route.
 */
public class PathFinder {
    private NodeMap nodeMap;

    public PathFinder(NodeMap nodeMap) {
        this.nodeMap = nodeMap;
    }

    public PathFinder() {

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
        return new AStarResult(nodeMap, nodeTrace, costOfNode, start, goal, currentNode);
    }
}
