package problem;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;


/**
 * This class represents one of the rectangular obstacles in Assignment 1.
 * 
 * @author Sergiy Dudnikov
 */
public abstract class Box {
	/** Stores the box as a Rectangle2D */
	private Rectangle2D rect;
	/** The position of the box */
	public Point2D pos;
	/** The width of the box */
	private double width;

	private boolean include = true;

	/** Default Constructor */
	public Box() {

	}
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
	public Box(double x, double y, double w, double h) {
	    this.rect = new Rectangle2D.Double(x, y, w, h);
	}

	/**
	 * Constructs a square box at a position width a side width
	 * bottom-left corner, as well as the width and height.
	 * 
	 * @param pos
	 *            the position of the box
	 * @param width
	 *            the width (and height) of the box
	 */
	public Box(Point2D pos, double width) {
		this.pos = (Point2D) pos.clone();
		this.width = width;
        this.rect = new Rectangle2D.Double(pos.getX(), pos.getY(), width, width);
	}

	/**
	 * returns the width of the box
	 */
	public double getWidth() {return width;}


	/**
	 * returns the position of the box
	 */
	public Point2D getPos() {
	    return pos;
    }
	/**
	 * Returns a copy of the Rectangle2D representing the box.
	 * 
	 * @return a copy of the Rectangle2D representing the box.
	 */
	public Rectangle2D getRect() {
		return (Rectangle2D) rect.clone();
	}

	/**
	 * Returns a String representation of the box.
	 *
	 * @return a String representation of the box.
	 */
	public String toString() {
		return rect.toString();
	}

	public void setRect() {
		this.rect = new Rectangle2D.Double(this.pos.getX(), this.pos.getY(), this.width, this.width);
	}

	@Override
    public boolean equals(Object o){
	    if (o instanceof Box) {
	        Box b = (Box) o;
	        return b.getRect().equals((rect));
        }
	    return false;
    }
}
