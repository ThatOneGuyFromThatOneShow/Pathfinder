package main.nodemaps;

import main.nodes.Node;

import java.io.File;
import java.util.ArrayList;

//TODO JavaDoc
public class WidthImageNodeMap extends ImageNodeMap {
    private int safeWidth = 0;
    private int actualWidth = 0;

    public WidthImageNodeMap(File file) {
        super(file);
    }

    public WidthImageNodeMap(File file, int safeWidth, int actualWidth) {
        super(file);
        setSafeWidth(safeWidth);
        setActualWidth(actualWidth);
    }

    public WidthImageNodeMap(String string) {
        super(string);
    }

    public WidthImageNodeMap(String string,  int safeWidth, int actualWidth) {
        super(string);
        setSafeWidth(safeWidth);
        setActualWidth(actualWidth);
    }

    public void setSafeWidth(int w) {
        safeWidth = w;
    }

    public void setActualWidth(int w) {
        actualWidth = w;
    }

    public int getSafeWidth() {
        return safeWidth;
    }

    public int getActualWidth() {
        return actualWidth;
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
