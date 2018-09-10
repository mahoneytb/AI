package problem;

import java.awt.geom.Point2D;
import java.util.ArrayList;

/**
 * This class represents the configuration of the agent in Assignment 1.
 * 
 * @author Sergiy Dudnikov
 */
public class RobotConfig {

    /** The position of the robot */
    public Point2D pos;
    /** The orientation of the robot */
    public double angle;
    public ArrayList<RobotConfig> neighbours = new ArrayList<>();
    public int cost;
    public int g;
    public RobotConfig parent;
    public boolean opposite_rotation = false;

    /**
	 * Constructs a configuration of the robot
	 * 
	 * @param coords
	 *            the position of the center of the robot in an array of 2 integers
	 * @param angle
	 *            the orientation of the robot
	 */
    public RobotConfig(double[] coords, double angle) {
        pos = new Point2D.Double(coords[0], coords[1]);
        this.angle = angle;
    }

    /**
	 * Constructs a configuration of the robot
	 * 
	 * @param pos
	 *            the position of the center of the robot as a Point2D
	 * @param angle
	 *            the orientation of the robot
	 */
    public RobotConfig(Point2D pos, double angle) {
        this.pos = (Point2D) pos.clone();
        this.angle = angle;
    }

    /** returns the position of the robot */
    public Point2D getPos() {
        return pos;
    }

    /** returns the orientation of the robot */
    public double getOrientation() {
        return angle;
    }

    public float getX1(double robotWidth) {
        return (float) (pos.getX() - Math.cos(angle) * robotWidth / 2);
    }

    public float getX2(double robotWidth) {
        return (float) (pos.getX() + Math.cos(angle) * robotWidth / 2);
    }

    public float getY1(double robotWidth) {
        return (float) (pos.getY() - Math.sin(angle) * robotWidth / 2);
    }

    public float getY2(double robotWidth) {
        return (float) (pos.getY() + Math.sin(angle) * robotWidth / 2);
    }

    public boolean equals(Object o) {
        if (o instanceof  RobotConfig) {
            RobotConfig r = (RobotConfig) o;
            return (this.angle == r.angle) && (this.pos.equals(r.pos));
        }
        return false;
    }

}