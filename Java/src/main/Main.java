package main;

import main.aStar.AStarResult;
import main.nodemaps.WidthImageNodeMap;
import main.nodes.Node;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

public class Main {
    public static void main(String args[]) {
        Pathfinder pathfinder = new Pathfinder();

        WidthImageNodeMap imgMap = new WidthImageNodeMap("images/Picture1.png");
        imgMap.setSafeWidth(40);
        imgMap.setActualWidth(24);

        pathfinder.setNodeMap(imgMap);
        AStarResult result = pathfinder.aStarSearch(new Node(162, 50), new Node(162, 230));
        Node[] path = pathfinder.makePath(result);


        Node[] waypointPath = pathfinder.makeWaypointPath(path, imgMap.getActualWidth());

        for (Node i : waypointPath) {
            System.out.println(i.getX() + "   " + i.getY());
        }

        BufferedImage img = imgMap.makePathImage(waypointPath);
        File file = new File("E:\\workspace\\Pathfinder\\Java\\src\\main\\images\\output.png");
        try {
            file.createNewFile();
            ImageIO.write(img, "png", file);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }
}
