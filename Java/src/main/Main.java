package main;

import main.frc.FRCPathFinder;
import main.frc.FRCResult;
import main.frc.MotorOutput;
import main.frc.Waypoint;
import main.nodemaps.WidthImageNodeMap;
import main.nodes.Node;

public class Main {
    public static void main(String args[]) {
        FRCPathFinder pathfinder = new FRCPathFinder();

        WidthImageNodeMap imgMap = new WidthImageNodeMap("images/Picture1.png");
        imgMap.setSafeWidth(40);
        imgMap.setActualWidth(24);

        pathfinder.setNodeMap(imgMap);
        FRCResult result = pathfinder.aStarSearch(new Node(162, 50), new Node(162, 230));
        Waypoint[] nodes = result.pathToWaypoints(0);
        MotorOutput mtr = new MotorOutput(12, nodes);

        for (Waypoint waypoint : nodes) {
            System.out.println(String.format("%f, %f, %f", waypoint.getX(), waypoint.getY(), waypoint.getAngle()));
        }

        System.out.println("\n\n\n\n-----------------------------\n\n\n\n");

        double curX = 162.0/12.0, curY = 50.0/12.0, curAngle = 0;
        while(!mtr.isFinished()) { //Mocks a accelerometer/gyro to test the speed output method
            double[] lr = mtr.getSpeed(curX, curY, curAngle);

            double left = lr[0], right = lr[1];

            //System.out.println(String.format("%f, %f, %f", curX * 12, curY * 12, curAngle));
            //System.out.println(String.format("%f, %f", left, right));

            curAngle += (left - right);
            curAngle %= 360;
            curAngle = curAngle > 0 ? curAngle : 360 - curAngle;

            double angle = Math.toRadians(curAngle);
            curX += -.01 * Math.sin(angle);
            curY += .01 * Math.cos(angle);

            try {
                Thread.sleep(0);
            } catch (InterruptedException e) {}
        }

//        BufferedImage img = imgMap.makePathImage(nodes);
//        File file = new File("/home/jcrane/dev/Pathfinding/Java/src/main/images/output.png");
//        try {
//            file.createNewFile();
//            ImageIO.write(img, "png", file);
//        } catch (IOException e) {
//            e.printStackTrace();
//        }

    }
}
