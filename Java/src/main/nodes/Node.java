package main.nodes;

import java.util.Arrays;

/**
 * A node holds an x and y value, which can not be changed.
 */
public class Node {
    private int x;
    private int y;

    /**
     * Creates a new node representing the location (x, y).
     *
     * @param x The x (horizontal) location of the node.
     * @param y The y (vertical) location of the node.
     */
    public Node(int x, int y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Returns the x value of this node.
     *
     * @return The x value.
     */
    public int getX() {
        return x;
    }

    /**
     * Returns the y value of this node.
     *
     * @return The y value.
     */
    public int getY() {
        return y;
    }

    @Override
    public int hashCode() {
        int[] xy = {getX(), getY()};
        return Arrays.hashCode(xy);
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof Node && hashCode() == obj.hashCode();
    }
}