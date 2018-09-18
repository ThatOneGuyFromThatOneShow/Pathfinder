package jcrane.pathfinder.nodemaps;

import jcrane.pathfinder.nodes.Node;

/**
 * Basic interface for a NodeMap. Nodes start at (x, y) = (0, 0). Nodes should never have a negative x or y!
 */
public interface NodeMap {

    /**
     * Returns a int 0-10. 0 being safe to move on, 10 being not safe, everything else is respective to it's value.
     *
     * @param node The location node to check.
     * @return Effort that should be used to avoid this node.
     */
    int getValueFromNode(Node node);

    /**
     * Returns whether a node is not reachable (value of 10).
     *
     * @param node The node to check.
     * @return True if node is unreachable.
     */
    boolean isOccupied(Node node);

    /**
     * Returns an array of all accessible nodes neighboring node.
     *
     * @param node The node to check.
     * @return All neighboring nodes that are not occupied (value of 10).
     */
    Node[] getNeighbors(Node node);

    /**
     * Calculates the cost of moving in a straight line from node1 to node2. This ignores obstacle collision.
     *
     * @param node1 The starting location.
     * @param node2 The ending position.
     * @return The cost of moving.
     */
    double calculateCost(Node node1, Node node2);

    void setResolution(int resolution);

    int getResolution();
}
