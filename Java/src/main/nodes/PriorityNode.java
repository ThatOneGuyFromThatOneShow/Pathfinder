package main.nodes;

import java.util.Arrays;

/**
 * A Priority Node holds x, y, and priority values, which can not be changed.
 */
public class PriorityNode extends Node {
    private double priority;

    /**
     * Creates a new PriorityNode at a location, with a priority value.
     *
     * @param x The x (horizontal) location of the node.
     * @param y The y (vertical) location of the node.
     * @param priority The priority to assign to this node.
     */
    public PriorityNode(int x, int y, double priority) {
        super(x, y);
        this.priority = priority;
    }

    /**
     * Creates a new PriorityNode from a Node. Effectively adding a priority.
     *
     * @param node The node to create a PriorityNode from.
     * @param priority The priority to assign to this node.
     */
    public PriorityNode(Node node, double priority) {
        super(node.getX(), node.getY());
        this.priority = priority;
    }

    /**
     * Returns the priority of this node.
     *
     * @return The priority of this node.
     */
    public double getPriority() {
        return priority;
    }

    @Override
    public int hashCode() {
        double[] xyp = {getX(), getY(), getPriority()};
        return Arrays.hashCode(xyp);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof PriorityNode))
            return false;
        PriorityNode objN = (PriorityNode) obj;
        return getX() == objN.getX() && getY() == objN.getY() && getPriority() == objN.getPriority();
    }

    /**
     * Creates a normal node. This is particularly handy because Nodes and PriorityNodes aren't directly comparable.
     *
     * @return A normal node.
     */
    public Node toNode() {
        return new Node(getX(), getY());
    }
}