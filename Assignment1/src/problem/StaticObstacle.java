package problem;

import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Scanner;

/**
 * This class represents one of the rectangular obstacles in Assignment 1.
 * 
 * @author Sergiy Dudnikov
 */
public class StaticObstacle {
	/** Stores the obstacle as a Rectangle2D */
	private Rectangle2D rect;

	/**
	 * Constructs an obstacle with the given (x,y) coordinates of the
	 * bottom-left corner, as well as the width and height.
	 * 
	 * @param x
	 *            the minimum x-value.
	 * @param y
	 *            the minimum y-value.
	 * @param w
	 *            the width of the obstacle.
	 * @param h
	 *            the height of the obstacle.
	 */
	public StaticObstacle(double x, double y, double w, double h) {
		this.rect = new Rectangle2D.Double(x, y, w, h);
	}

	/**
	 * Constructs an obstacle from the representation used in the input file:
	 * that is, the x- and y- coordinates of all of the corners of the
	 * rectangle.
	 * 
	 * @param str
	 */
	public StaticObstacle(String str) {
		Scanner s = new Scanner(str);
		List<Double> xs = new ArrayList<Double>();
		List<Double> ys = new ArrayList<Double>();
		for (int i = 0; i < 2; i++) {
			xs.add(s.nextDouble());
			ys.add(s.nextDouble());
		}
		double EXTRA = 0.001;
		double xMin = Collections.min(xs);
		xMin = ((xMin - EXTRA) > 0) ? xMin - EXTRA : 0;
		double xMax = Collections.max(xs);
		xMax = ((xMax + EXTRA) < 1) ? xMax + EXTRA : 1;
		double yMin = Collections.min(ys);
		yMin = ((yMin + EXTRA) > 0) ? yMin - EXTRA : 0;
		double yMax = Collections.max(ys);
		yMax = ((yMax + EXTRA) < 1) ? yMax + EXTRA : 1;
		this.rect = new Rectangle2D.Double(xMin-EXTRA, yMin-EXTRA, xMax - xMin + 2*EXTRA,
				yMax - yMin + 2*EXTRA);
		s.close();
	}

	/**
	 * Returns a copy of the Rectangle2D representing this obstacle.
	 * 
	 * @return a copy of the Rectangle2D representing this obstacle.
	 */
	public Rectangle2D getRect() {
		return (Rectangle2D) rect.clone();
	}

	/**
	 * Returns a String representation of this obstacle.
	 * 
	 * @return a String representation of this obstacle.
	 */
	public String toString() {
		return rect.toString();
	}
}
