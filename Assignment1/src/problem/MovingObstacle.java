package problem;

import problem.Box;

import java.awt.geom.Point2D;


/**
 * This class represents one of the moving obstacles in Assignment 1.
 * 
 * @author Sergiy Dudnikov
 */
public class MovingObstacle extends Box {

	/**
	 * Constructs a Moving obstacle at a position width a side width
	 * 
	 * @param pos
	 *            the position of the moving obstacle
	 * @param width
	 *            the width (and height) of the moving obstacle
	 */
    public MovingObstacle(Point2D pos, double width) {
        super(pos, width);
    }
}
