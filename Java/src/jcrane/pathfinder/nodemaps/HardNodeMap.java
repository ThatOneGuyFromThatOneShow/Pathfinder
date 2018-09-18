package jcrane.pathfinder.nodemaps;

import jcrane.pathfinder.nodes.Node;

import java.util.ArrayList;

public class HardNodeMap implements NodeMap {
    private int[][] graph;
    private int resolution;

    /**
     * A 2d map with 0 being an available location and 10 being a unavailable location.
     *
     * 1-9 are how much effort should be used to avoid the location.
     * @param graph A 2d array map.
     */
    public HardNodeMap(int[][] graph) {
        this.graph = graph;
    }

    @Override
    public void setResolution(int resolution) {
        this.resolution = resolution;
    }

    @Override
    public int getResolution() {
        return resolution;
    }

    public int getValueFromNode(Node node) {
        return graph[node.getY()][node.getX()];
    }

    public boolean isOccupied(Node node) {
        if (node.getX() < 0 || node.getX() >= graph[0].length || node.getY() < 0 || node.getY() >= graph.length)
            return true;
        return getValueFromNode(node) >= 10;
    }

    public Node[] getNeighbors(Node node) {
        ArrayList<Node> neighbors = new ArrayList<>();
        for (int xi = -1; xi <= 1; xi++) {
            for (int yi = -1; yi <= 1; yi++) {
                if (!(xi == 0 && yi == 0)) {
                    int x = node.getX() + xi;
                    int y = node.getY() + yi;

                    Node curNode = new Node(x, y);
                    if (!isOccupied(curNode))
                        neighbors.add(curNode);
                }
            }
        }
        return neighbors.toArray(new Node[neighbors.size()]);
    }

    public double calculateCost(Node node1, Node node2) {
        return Math.hypot(node1.getX() - node2.getX(), node1.getY() - node2.getY());
    }
}
