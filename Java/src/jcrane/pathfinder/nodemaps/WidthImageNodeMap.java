package jcrane.pathfinder.nodemaps;

import jcrane.pathfinder.nodes.Node;

import java.io.File;
import java.util.ArrayList;

//TODO JavaDoc
public class WidthImageNodeMap extends ImageNodeMap implements WidthMap {
    private int safeWidth = 0;
    private int actualWidth = 0;
    private int resolution = 0; //Nodes per foot in either x or y direction. Note this is not nodes per foot x * nodes per foot y.

    public WidthImageNodeMap(File file) {
        super(file);
    }

    public WidthImageNodeMap(File file, int safeWidth, int actualWidth, int resolution) {
        super(file);
        setSafeWidth(safeWidth);
        setActualWidth(actualWidth);
        setResolution(resolution);
    }

    public WidthImageNodeMap(String string) {
        super(string);
    }

    public WidthImageNodeMap(String string,  int safeWidth, int actualWidth, int resolution) {
        super(string);
        setSafeWidth(safeWidth);
        setActualWidth(actualWidth);
        setResolution(resolution);
    }

    public void setSafeWidth(int safeWidth) {
        this.safeWidth = safeWidth;
    }

    public void setActualWidth(int actualWidth) {
        this.actualWidth = actualWidth;
    }

    public void setResolution(int resolution) {
        this.resolution = resolution;
    }

    public int getSafeWidth() {
        return safeWidth;
    }

    public int getActualWidth() {
        return actualWidth;
    }

    public int getResolution() {
        return resolution;
    }

    @Override
    public Node[] getNeighbors(Node node) {
        ArrayList<Node> neighbors = new ArrayList<>();
        for (int xi = -1; xi <= 1; xi++) {
            for (int yi = -1; yi <= 1; yi++) {
                if (!(xi == 0 && yi == 0)) {
                    int x = node.getX() + xi;
                    int y = node.getY() + yi;

                    int tx = x + (safeWidth /2 * xi);
                    int ty =  y + (safeWidth /2 * yi);
                    Node testNode = new Node(tx, ty);

                    if (tx >= 0 && tx < image.getWidth() && ty >= 0 && ty < image.getHeight()) {
                        if (!isOccupied(testNode)) {
                            int safeWidthCorner = (int) Math.ceil(Math.sqrt(2) * safeWidth / 4);
                            Node tn0 = new Node(x + safeWidthCorner, y + safeWidthCorner);
                            Node tn1 = new Node(x + safeWidthCorner, y - safeWidthCorner);
                            Node tn2 = new Node(x - safeWidthCorner, y + safeWidthCorner);
                            Node tn3 = new Node(x - safeWidthCorner, y - safeWidthCorner);

                            Node tn4 = new Node(x, y + (safeWidth /2));
                            Node tn5 = new Node(x + (safeWidth /2), y);
                            Node tn6 = new Node(x, y - (safeWidth /2));
                            Node tn7 = new Node(x - (safeWidth /2), y);

                            boolean test = !isOccupied(tn0) && !isOccupied(tn1) && !isOccupied(tn2) && !isOccupied(tn3) && !isOccupied(tn4)
                                    && !isOccupied(tn5) && !isOccupied(tn6) && !isOccupied(tn7);

                            if (test)
                                neighbors.add(new Node(x, y));
                        }
                    }
                }
            }
        }
        return neighbors.toArray(new Node[neighbors.size()]);
    }
}
