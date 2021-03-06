// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.hyperdrive.util;

/**
 * Simple class structure that holds a point in the XY plane.
 */
public class Point2D {
    private double
        x,
        y,
        heading;

    /**
     * Creates a new Point2D.
     * @param x X-coordinate of the point.
     * @param y Y-coordinate of the point.
     * @param heading Heading of the point.
     */
    public Point2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Returns the X-coordinate of the point.
     * @return X-coordinate of point.
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the Y-coordinate of the point.
     * @return Y-coordinate of point.
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the heading of the point.
     * @return Heading (exit angle) of point.
     */
    public double getHeading() {
        return heading;
    }

    /**
     * Returns the distance from the given point.
     * @param point The point to measure distance to.
     * @return The distance between this point and the passed point.
     */
    public double getDistanceFrom(Point2D point) {
        //pythagorean theorem moment
        double xDist = point.getX() - this.x;
        double yDist = point.getY() - this.y;
        return Math.sqrt(Math.pow(xDist, 2) + Math.pow(yDist, 2));
    }

    /**
     * Returns the heading needed to point at the passed point.
     * @param point The point to calculate the heading to.
     * @return The heading between this point and the passed point.
     */
    public double getHeadingTo(Point2D point) {
        double displacementX = point.getX() - getX();
        double displacementY = point.getY() - getY();
        double targetHeading = Math.toDegrees(Math.atan2(displacementY, displacementX));
        return targetHeading;
    }

    /**
     * Returns a representation of this point in String format.
     * Format: [x],[y],[heading]
     * The output from this method can be used in the Point2D.fromString() method.
     * @return Human and computer readable String representation of the Point.
     */
    public String toString() {
        double roundedX = HyperdriveUtil.roundTo(this.getX(), 2);
        double roundedY = HyperdriveUtil.roundTo(this.getY(), 2);
        double roundedH = HyperdriveUtil.roundTo(this.getHeading(), 2);
        return Double.valueOf(roundedX).toString() + "," + Double.valueOf(roundedY).toString() + "," + Double.valueOf(roundedH).toString();
    }

    /**
     * Returns a Point2D from a given String input
     * - Expected Format: [x],[y],[heading]
     * @param input Formatted String representing a Point2D (can use output from
     * {@link #toString()})
     * @return The Point2D represented by the string, or {@code null} if the String
     * is not formatted correctly (too few numbers, letters instead of numbers, etc).
     */
    public static Point2D fromString(String input) {
        String[] parts = input.split(",");
        if(parts.length < 3) {
            return null;
        }

        try {
            double x = Double.valueOf(parts[0]).doubleValue();
            double y = Double.valueOf(parts[1]).doubleValue();
            double heading = Double.valueOf(parts[2]).doubleValue();
            return new Point2D(x, y, heading);
        } catch(NumberFormatException ex) {
            return null;
        }
    }
}
