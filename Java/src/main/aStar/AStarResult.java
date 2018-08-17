package main.aStar;

import main.nodes.Node;

import java.util.Map;

/**
 * A basic class for holding information returned from an A* search.
 */
public class AStarResult {
    private Map<Node, Node> nodeTrace;
    private Map<Node, Double> costOfNode;
    private Node startNode, goalNode, lastNode;

    /**
     * Creates a new A* result object.
     *
     * @param nodeTrace The map of every node and what node it came from.
     * @param costOfNode The map of the cost to move to any node.
     * @param startNode The node the search originated from.
     * @param goalNode The node the search ended on.
     * @param lastNode The last node reached by the algorithm.
     */
    public AStarResult(Map<Node, Node> nodeTrace, Map<Node, Double> costOfNode, Node startNode, Node goalNode, Node lastNode) {
        this.nodeTrace = nodeTrace;
        this.costOfNode = costOfNode;
        this.startNode = startNode;
        this.goalNode = goalNode;
        this.lastNode = lastNode;
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

    public boolean failed() {
        return !getGoalNode().equals(getLastNode());
    }
}
