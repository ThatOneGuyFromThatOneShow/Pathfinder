package jcrane.pathfinder.frc;

import java.util.Arrays;
import java.util.Stack;

public class MotorOutput {
    private final static double DEFAULT_ANGLE_CORRECTION = 45.0, DEFAULT_OFFSET_CORRECTION = 20.0;

    private double pixelsPerFoot;
    private Stack<Waypoint> waypoints;
    private Waypoint currentPoint, startingPoint;
    private boolean finished = false;
    private double currentX, currentY;
    private double offsetCorrection = 1.0, angleCorrection = 1.0;

    /**
     * Creates a new MotorOutput, this is used to take current location of a robot and give back the speed of the left and right motors.
     *
     * @param pixelsPerFoot The pixels / integer node points that relate to one physical foot.
     * @param waypoints     An array of waypoints, this must be generated from an instance of FRCResult.
     */
    public MotorOutput(double pixelsPerFoot, Waypoint[] waypoints) {
        this.pixelsPerFoot = pixelsPerFoot;

        this.waypoints = new Stack<>();
        this.waypoints.addAll(Arrays.asList(waypoints));

        startingPoint = pop();
        currentPoint = pop();
        currentX = startingPoint.getX() / pixelsPerFoot;
        currentY = startingPoint.getY() / pixelsPerFoot;
    }

    private Waypoint pop() {
        Waypoint tmp = waypoints.firstElement();
        waypoints.remove(0);
        return tmp;
    }

    //Speeds between -1, 1. [0] = left, [1] = right

    /**
     * Takes the current location relative to the node map, NOT the starting position, and returns the left and right motor speeds.
     *
     * @param feetX        The current x location in feet.
     * @param feetY        The current y location in feet.
     * @param angleDegrees The current angle in degrees, 0-360.
     * @return An array of length 2, when element 0 and 1, are the left and right speeds respectively.
     */
    public double[] getSpeed(double feetX, double feetY, double angleDegrees) {
        double[] leftRight = new double[2];

        if (finished) {
            leftRight[0] = 0;
            leftRight[1] = 0;

            return leftRight;
        }

        boolean pastPoint = pastPoint(feetX, feetY, angleDegrees, currentPoint);

        if (pastPoint && !waypoints.isEmpty()) {
            currentPoint = pop();
        } else if (pastPoint) {
            finished = true;

            leftRight[0] = 0;
            leftRight[1] = 0;

            return leftRight;
        }

        //Calculate speed

        double targetAngle = Math.atan2(currentPoint.getY() / pixelsPerFoot - feetY, currentPoint.getX() / pixelsPerFoot - feetX);
        targetAngle = Math.toDegrees(targetAngle);
        angleDegrees = angleDegrees > 180 ? angleDegrees - 360 : angleDegrees;
        double deltaAngle = (angleDegrees - targetAngle) / DEFAULT_ANGLE_CORRECTION * angleCorrection; //At 25 degrees of one wheel will be at 1 and the other at 0; lower is more correction.
        double rightDelta = rightDeviation(feetX, feetY, currentPoint) * DEFAULT_OFFSET_CORRECTION * offsetCorrection; //Greater coefficient is greater correction
        double correction = deltaAngle + rightDelta;

        double left = 1 - correction;
        double right = 1 + correction;

        leftRight[0] = clamp(left, -1, 1);
        leftRight[1] = clamp(right, -1, 1);

        return leftRight;
    }

    public double[] getSpeed(double distanceFeet, double angleDegrees) {
        double angle = Math.toRadians(angleDegrees);
        currentX -= distanceFeet * Math.sin(angle);
        currentY += distanceFeet * Math.cos(angle);
        return getSpeed(currentX, currentY, angleDegrees);
    }

    public double getCurrentX() {
        return currentX;
    }

    public double getCurrentY() {
        return currentY;
    }

    public void setAngleCorrection(double angleCorrection) {
        this.angleCorrection = angleCorrection;
    }

    public void setOffsetCorrection(double offsetCorrection) {
        this.offsetCorrection = offsetCorrection;
    }

    private double clamp(double num, double min, double max) {
        return num < max ? (num > min ? num : min) : max;
    }

    private boolean pastPoint(double feetX, double feetY, double angleDegrees, Waypoint waypoint) {
        return forwardDeviation(feetX, feetY, waypoint) >= 0;
    }

    //In feet
    private double forwardDeviation(double feetX, double feetY, Waypoint waypoint) {
        double angle = Math.toRadians(-waypoint.getAngle());
        double y = (waypoint.getY() * Math.cos(angle) + waypoint.getX() * Math.sin(angle)) / pixelsPerFoot;

        //double physicalAngle = Math.toRadians(-angleDegrees);
        double physicalY = feetY * Math.cos(angle) + feetX * Math.sin(angle);

        return physicalY - y;
    }

    //In feet
    public double rightDeviation(double feetX, double feetY, Waypoint waypoint) {
        double angle = Math.toRadians(-waypoint.getAngle());
        double x = (waypoint.getX() * Math.cos(angle) - waypoint.getY() * Math.sin(angle)) / pixelsPerFoot;
        double physicalX = feetX * Math.cos(angle) - feetY * Math.sin(angle);

        return x - physicalX;
    }

    public boolean isFinished() {
        return finished;
    }

}
