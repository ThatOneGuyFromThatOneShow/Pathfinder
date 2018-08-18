package main.nodemaps;

import main.nodes.Node;

import javax.imageio.ImageIO;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.ArrayList;

public class ImageNodeMap implements NodeMap {
    protected BufferedImage image;
    private boolean readable;
    private ArrayList<Integer> colorReference = new ArrayList<>(11);
    public ColorModel color;

    public ImageNodeMap(File file) {
        initialize(file);
    }

    public ImageNodeMap(String string) {
        URL filePath = getClass().getResource("../"+string);
        File file = null;
        try {
            file = new File(filePath.toURI());
        } catch (URISyntaxException e) {
            e.printStackTrace();
        }
        initialize(file);
    }

    private void initialize(File file) {
        try {
            image = ImageIO.read(file);

            readable = true;

            for (int i = 0; i <= 10; i++) {
                colorReference.add(i, image.getRGB(i, 0));
            }

            image = image.getSubimage(0, 1, image.getWidth(), image.getHeight()-1);

        } catch (IOException e) {
            e.printStackTrace();
            readable = false;
        }
    }

    public boolean isReadable() {
        return readable;
    }

    public ArrayList<Integer> getColorReference() {
        return colorReference;
    }

    public int getValueFromNode(Node node) {
        int colorValue = image.getRGB(node.getX(), node.getY());
        int bestIndex = 0;
        int lowestValue = Math.abs(colorReference.get(0) - colorValue);
        for (int i = 1; i <= 10; i++) {
            int tmp = Math.abs(colorReference.get(i) - colorValue);
            if (tmp < lowestValue) {
                lowestValue = tmp;
                bestIndex = i;
            }
        }

        return bestIndex;
    }

    public boolean isOccupied(Node node) {
        if (node.getX() < 0 || node.getX() >= image.getWidth() || node.getY() < 0 || node.getY() >= image.getHeight())
            return true;
        return getValueFromNode(node) == 10;
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

    public BufferedImage makePathImage(Node[] path) {
        BufferedImage tempImage = new BufferedImage(image.getWidth(), image.getHeight(), image.getType());
        for (Node n : path) {
            tempImage.setRGB(n.getX(), n.getY(), colorReference.get(10));
        }
        return tempImage;
    }
}
