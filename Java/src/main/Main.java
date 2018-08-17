package main;

import main.aStar.AStarResult;
import main.nodes.Node;
import main.nodemaps.HardNodeMap;

public class Main {
    public static void main(String args[]) {
        Pathfinder pathfinder = new Pathfinder();

        int[][] graph = {{0, 10, 0, 0},
                        {0, 10, 0, 0},
                        {0, 0, 10, 0},
                        {0, 0, 10, 0}};
        HardNodeMap nodeMap = new HardNodeMap(graph);
        pathfinder.setNodeMap(nodeMap);
        AStarResult result = pathfinder.aStarSearch(new Node(0, 0), new Node(3, 3));
        Node[] path = pathfinder.makePath(result);
        Node[] waypointPath = pathfinder.makeWaypointPath(path);

        for (Node i : path) {
            System.out.println(String.format("(%d, %d)", i.getX(), i.getY()));
        }

    }
}
