package tester;
import problem.*;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.io.IOException;
import java.util.*;

public class Tester {
    /** Maximum step size for a primitive step*/
    public static final double MAX_BASE_STEP = 0.001;
    /** Maximum error*/
    public static final double MAX_ERROR = 0.0001;

    /** Remembers the specifications of the problem. */
    private ProblemSpec ps;
    /** Maximum angle error when checking if robot is parallel to axis */
    private double angleError;

    public Tester(ProblemSpec ps){
        this.ps = ps;
        angleError = Math.asin((MAX_ERROR/2)/(ps.getRobotWidth()/2)) * 2;
    }

    /**
     * Read problem and solution. Runs tests.
     * @param args input file name for problem and solution
     */
    public static void main(String[] args) {
        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem(args[0]);
        } catch (IOException e1) {
            System.out.println("FAILED: Invalid problem file");
            System.out.println(e1.getMessage());
            return;
        }
        try {
            ps.loadSolution(args[1]);
        } catch (IOException e1) {
            System.out.println("FAILED: Invalid solution file");
            System.out.println(e1.getMessage());
            return;
        }
        Tester tester = new Tester(ps);
        tester.testSolution();
    }

    /**
     *  Runs all tests.
     */
    public void testSolution() {
        boolean pass = true;
        if (ps.getProblemLoaded() && ps.getSolutionLoaded()) {
            pass = testInitialFirst() && pass;
            pass = testStepSize() && pass;
            pass = testCollision() && pass;
            pass = testPushedBox() && pass;
        }
        if (pass) {
            int count = countGoals();
            System.out.println(count + " out of " + ps.getMovingBoxes().size() + " goals reached");
        }
    }

    /**
     * Count the amount of goals reached.
     * @return amount of goals reached.
     */
    public int countGoals() {
        List<Box> finalState = ps.getMovingBoxPath().get(ps.getMovingBoxPath().size() - 1);
        int count = 0;
        for (int i = 0; i < finalState.size(); i++){
            if (finalState.get(i).getPos().distance(ps.getMovingBoxEndPositions().get(i)) <= MAX_ERROR) {
                count++;
            }
        }
        return count;
    }

    /**
     * Creates a new Rectangle2D that is grown by delta in each direction
     * compared to the given Rectangle2D.
     *
     * @param rect
     *            the Rectangle2D to expand.
     * @param delta
     *            the amount to expand by.
     * @return a Rectangle2D expanded by delta in each direction.
     */
    public Rectangle2D grow(Rectangle2D rect, double delta) {
        return new Rectangle2D.Double(rect.getX() - delta, rect.getY() - delta,
                rect.getWidth() + 2 * delta, rect.getHeight() + 2 * delta);
    }

    /**
     * Test whether the solution starts with the initial state
     * @return true of false
     */
    public boolean testInitialFirst(){
        System.out.println("Test Initial State");
        if (hasInitialFirst()) {
            System.out.println("Passed.");
            return true;
        } else {
            System.out.println("Solution path must start at initial state.");
            return false;
        }
    }

    private boolean hasInitialFirst() {
        if (!ps.getInitialRobotConfig().equals(ps.getRobotPath().get(0))) {
            return false;
        }
        List<Box> movingBoxes = ps.getMovingBoxes();
        List<List<Box>> movingBoxesPath = ps.getMovingBoxPath();
        for (int i = 0; i < movingBoxes.size(); i++) {
            if (!movingBoxes.get(i).getPos().equals(movingBoxesPath.get(0).get(i).getPos())) {
                return false;
            }
        }

        List<Box> movingObstacles = ps.getMovingObstacles();
        List<List<Box>> movingObstaclePath = ps.getMovingObstaclePath();
        for (int i = 0; i < movingObstacles.size(); i++) {
            if (!movingObstacles.get(i).getPos().equals(movingObstaclePath.get(0).get(i).getPos())) {
                return false;
            }
        }

        return true;
    }

    /**
     * Test whether each step of the solution is strictly less than primitive step size
     * @return true or false
     */

    public boolean testStepSize() {
        System.out.println("Test Step Size");
        List<RobotConfig> robotPath = ps.getRobotPath();
        RobotConfig last = robotPath.get(0);
        boolean pass = true;

        for (int i = 1; i < robotPath.size(); i++) {
            if (!isValidStep(last, robotPath.get(i))) {
                System.out.println("Step size over 0.001 at step " + i);
                pass = false;
            }
            last = robotPath.get(i);
        }

        if (pass) {
            System.out.println("Passed");
        }

        return pass;
    }

    /**
     * Checks if the step size of the robot is valid from r1 to r2
     * @param r1 previous robot state
     * @param r2 current robot state
     * @return true or false
     */

    public boolean isValidStep(RobotConfig r1, RobotConfig r2) {
        if (getPoint1(r1).distance(getPoint1(r2)) > MAX_BASE_STEP + MAX_ERROR) {
            return false;
        }
        if (getPoint2(r1).distance(getPoint2(r2)) > MAX_BASE_STEP + MAX_ERROR) {
            return false;
        }
        return true;
    }

    /**
     * Get the first point of the robot
     * @param r the robot
     * @return A Point2D representing the first point.
     */
    public Point2D getPoint2(RobotConfig r) {
        double x = r.getPos().getX() + Math.cos(r.getOrientation()) * ps.getRobotWidth() * 0.5;
        double y = r.getPos().getY() + Math.sin(r.getOrientation()) * ps.getRobotWidth() * 0.5;
        return new Point2D.Double(x,y);
    }
    /**
     * Get the second point of the robot
     * @param r the robot
     * @return A Point2D representing the second point.
     */
    public Point2D getPoint1(RobotConfig r) {
        double x = r.getPos().getX() - Math.cos(r.getOrientation()) * ps.getRobotWidth() * 0.5;
        double y = r.getPos().getY() - Math.sin(r.getOrientation()) * ps.getRobotWidth() * 0.5;
        return new Point2D.Double(x,y);
    }

    /**
     * Test whether each push in the solution is valid.
     * @return true or false
     */
    public boolean testPushedBox() {
        System.out.println("Test pushed objects");
        boolean pass = true;
        for (int i = 1; i < ps.getRobotPath().size(); i++) {
            List<Box> oldMovingObjects = new ArrayList<Box>();
            List<Box> newMovingObjects = new ArrayList<Box>();
            oldMovingObjects.addAll(ps.getMovingBoxPath().get(i - 1));
            oldMovingObjects.addAll(ps.getMovingObstaclePath().get(i - 1));
            newMovingObjects.addAll(ps.getMovingBoxPath().get(i));
            newMovingObjects.addAll(ps.getMovingObstaclePath().get(i));
            int pushedBox = hasPushedBox(oldMovingObjects, newMovingObjects);

            switch (pushedBox){
                case -2: {
                    pass = false;
                    System.out.println("Multiple movable object moved at step" + i);
                }
                case -1 :continue;
                default: {
                    RobotConfig robot = ps.getRobotPath().get(i);
                    RobotConfig oldRobot = ps.getRobotPath().get(i - 1);
                    int direction = isCoupled(robot, newMovingObjects.get(pushedBox));
                    if (direction == -1) {
                        System.out.println("Robot not in pushing position but object moved at step " + i);
                        pass = false;
                    } else if (!testPushValidity(direction, oldRobot, robot, oldMovingObjects.get(pushedBox),
                            newMovingObjects.get(pushedBox))) {
                        System.out.println("Object not moving with robot" +
                                " or pushed to wrong direction at step " + i);
                        pass = false;
                    }
                }
            }
        }
        if (pass) {
            System.out.println("Passed");
        }
        return pass;
    }

    /**
     * Check if pushing from a given state to another state is valid.
     * @param direction the supposed direction of the push
     * @param oldRobot previous robot state
     * @param newRobot current robot state
     * @param oldBox previous movable object state
     * @param newBox current movable object state
     * @return true or false
     */
    public boolean testPushValidity(int direction, RobotConfig oldRobot, RobotConfig newRobot, Box oldBox, Box newBox) {
        if (direction == -1) {
            return false;
        }

        double robotdy = newRobot.getPos().getY() - oldRobot.getPos().getY();
        double robotdx = newRobot.getPos().getX() - oldRobot.getPos().getX();
        double boxdy = newBox.getPos().getY() - oldBox.getPos().getY();
        double boxdx = newBox.getPos().getX() - oldBox.getPos().getX();

        if (Math.abs(robotdy - boxdy) > MAX_ERROR || Math.abs(robotdx - boxdx) > MAX_ERROR) {
            return false;
        }
        int actualDirection = 0;
        int moved = 0;

        if (boxdy > MAX_ERROR) {
            actualDirection = 1;
            moved++;
        } else if (boxdy < -MAX_ERROR) {
            actualDirection = 3;
            moved++;
        } else if (boxdx > MAX_ERROR) {
            actualDirection = 2;
            moved++;
        } else if (boxdx < -MAX_ERROR) {
            actualDirection = 4;
            moved++;
        }

        if (moved > 1) {
            return false;
        }

        if (actualDirection != 0 && actualDirection != direction) {
            return false;
        }
        return true;
    }

    /**
     * Return the index of the pushed object from given previous state and current state
     * @param oldState A list of Box of all movable object from previous state
     * @param currentState A list of Box of all movable object from current state
     * @return -2 if multiple moved object found,
     *          -1 if no moved object found,
     *          index of moved object otherwise.
     */
    public int hasPushedBox(List<Box> oldState, List<Box> currentState) {
        int pushedBox = -1;
        for(int i = 0; i < oldState.size(); i++) {
            if (!oldState.get(i).getPos().equals(currentState.get(i).getPos())) {
                if (pushedBox != -1) {
                    return -2;
                }
                pushedBox = i;
            }
        }
        return pushedBox;
    }

    /**
     * Normalises an angle to the range (2pi, 4pi]
     *
     * @param angle
     *            the angle to normalise.
     * @return the normalised angle.
     */
    public double normaliseAngle(double angle) {
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        while (angle >= 2 * Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle + 2 * Math.PI;
    }

    /**
     * Checks if a given robot is axis aligned.
     * @param r the robot
     * @return true if yes false if no
     */
    public boolean isAxisAligned(RobotConfig r) {
        double angle = normaliseAngle(r.getOrientation() + angleError) % (0.5 * Math.PI);
        if (angle <= 2 * angleError) {
            return true;
        }
        return false;
    }

    /**
     * Check if a given robot and a given movable object are coupled together (ready to push)
     * @param r robot state
     * @param b movable object
     * @return -1 if not coupled,
     *          1 if robot on bottom of box
     *          2 if robot on left of box
     *          3 if robot on top of box
     *          4 if robot on right of box
     */
    public int isCoupled(RobotConfig r, Box b) {
        Point2D p1,p2;

        double angle = normaliseAngle(r.getOrientation());
        boolean horizontal;
        if (angle >= Math.PI * 4 - angleError || angle <= Math.PI * 2 + angleError) {
            p1 = getPoint1(r);
            p2 = getPoint2(r);
            horizontal = true;
        } else if (angle >= Math.PI * 2.5 - angleError && angle <= Math.PI * 2.5 + angleError) {
            p1 = getPoint1(r);
            p2 = getPoint2(r);
            horizontal = false;
        } else if (angle >= Math.PI * 3 - angleError && angle <= Math.PI * 3 + angleError) {
            p2 = getPoint1(r);
            p1 = getPoint2(r);
            horizontal = true;
        } else if (angle >= Math.PI * 3.5 - angleError && angle <= Math.PI * 3.5 + angleError) {
            p2 = getPoint1(r);
            p1 = getPoint2(r);
            horizontal = false;
        } else {
            return -1;
        }

        Rectangle2D collisionBox = grow(b.getRect(),MAX_ERROR);
        if ((!collisionBox.intersectsLine(new Line2D.Double(p1,p2)))) {
            return -1;
        }

        if (horizontal) {
            if (isCoincided(p1.getX(),p2.getX(),b.getRect().getMinX(),b.getRect().getMaxX())) {
                if (p1.getY() <= b.getPos().getY() + MAX_ERROR) {
                    return 1;
                } else {return 3;}
            }
        } else if (isCoincided(p1.getY(),p2.getY(),b.getRect().getMinY(),b.getRect().getMaxY())) {
            if (p1.getX() <= b.getPos().getX() + MAX_ERROR) {
                return 2;
            } else {return 4;}
        }

        return -1;
    }

    private boolean isCoincided(double m1, double m2, double n1, double n2) {
        if (m1 <= n1) {
            return (m2 - n1 >= 0.75 * ps.getRobotWidth());
        } else {
            return (n2 - m1 >= 0.85 * ps.getRobotWidth());
        }
    }

    /**
     * Test if any step in solution contains collision
     * @return true if passed
     */
    public boolean testCollision(){
        System.out.println("Test collision:");
        boolean pass = true;
        for (int i = 0; i < ps.getRobotPath().size(); i++) {
            List<Box> movingObjects = new ArrayList<Box>();
            movingObjects.addAll(ps.getMovingBoxPath().get(i));
            movingObjects.addAll(ps.getMovingObstaclePath().get(i));
            RobotConfig robot = ps.getRobotPath().get(i);
            if (!hasCollision(robot, movingObjects, false)) {
                System.out.println("Collision at step " + i);
                pass = false;
            }
            if (!testGapSliding(robot, movingObjects)) {
                System.out.println("Collision at step " + i);
                pass = false;
            }
        }
        if (pass) {
            System.out.println("Passed");
        }
        return pass;
    }

    /**
     * Checks if the robot is sliding through the tiny gap between two boxes
     * @param r the robot
     * @param movingObjects state of all movable objects
     * @return true if no, false if yes
     */
    public boolean testGapSliding(RobotConfig r, List<Box> movingObjects) {
        double angle = normaliseAngle(r.getOrientation());
        Point2D p1,p2,r1,r2;
        p1 = getPoint1(r);
        p2 = getPoint2(r);
        boolean horizontal;
        if (angle >= Math.PI * 4 - angleError || angle <= Math.PI * 2 + angleError) {
            r1 = new Point2D.Double(p1.getX() + MAX_BASE_STEP, p1.getY());
            r2 = new Point2D.Double(p2.getX() - MAX_BASE_STEP, p2.getY());
            horizontal = true;
        } else if (angle >= Math.PI * 2.5 - angleError && angle <= Math.PI * 2.5 + angleError) {
            r1 = new Point2D.Double(p1.getX(), p1.getY() + MAX_BASE_STEP);
            r2 = new Point2D.Double(p2.getX(), p2.getY() - MAX_BASE_STEP);
            horizontal = false;
        } else if (angle >= Math.PI * 3 - angleError && angle <= Math.PI * 3 + angleError) {
            r1 = new Point2D.Double(p2.getX() + MAX_BASE_STEP, p2.getY());
            r2 = new Point2D.Double(p1.getX() - MAX_BASE_STEP, p1.getY());
            horizontal = true;
        } else if (angle >= Math.PI * 3.5 - angleError && angle <= Math.PI * 3.5 + angleError) {
            r1 = new Point2D.Double(p2.getX(), p2.getY() + MAX_BASE_STEP);
            r2 = new Point2D.Double(p1.getX(), p1.getY() - MAX_BASE_STEP);
            horizontal = false;
        } else {
            return true;
        }
        int count = 0;
        Line2D robotLine = new Line2D.Double(r1, r2);
        List<Box> collidedBox = new ArrayList<Box>();
        for (Box b : movingObjects) {
            Rectangle2D collisionBox = grow(b.getRect(), MAX_ERROR);
            if (collisionBox.intersectsLine(robotLine)) {
                count++;
                collidedBox.add(b);
            }
        }
        if (count > 1) {
            return isAtSameSide(robotLine, collidedBox, horizontal);
        }
        return true;
    }

    private boolean isAtSameSide(Line2D line, List<Box> boxes, boolean horizontal) {
        double prevSide = 0;
        for (Box box: boxes) {
            double side;
            if (horizontal) {
                side = box.getPos().getY() + MAX_ERROR - line.getY1();
            } else {
                side = box.getPos().getX() + MAX_ERROR - line.getX1();
            }
            if (prevSide * side < 0) {
                return false;
            }
            prevSide = side;
        }
        return true;
    }

    /**
     * Check if a given state contains collision
     * @param r state of robot
     * @param movingObjects state of all movable objects
     * @return true if no collision
     */
    public boolean hasCollision(RobotConfig r, List<Box> movingObjects, boolean ignoreRobot) {
        Line2D robotLine = new Line2D.Double(getPoint1(r), getPoint2(r));
        Rectangle2D border = new Rectangle2D.Double(0,0,1,1);

        if (!ignoreRobot) {
            for (StaticObstacle o : ps.getStaticObstacles()) {
                if (robotLine.intersects(grow(o.getRect(), -MAX_ERROR))) {
                    return false;
                }
            }

            if (!border.contains(robotLine.getP1()) || !border.contains(robotLine.getP2())) {
                return false;
            }
        }

        for (Box b1: movingObjects) {

            if (!border.contains(b1.getRect())) {
                return false;
            }

            Rectangle2D collisionBox = grow(b1.getRect(),-MAX_ERROR);
            if (collisionBox.intersectsLine(robotLine) && !ignoreRobot) {
                    return false;
            }

            for (Box b2: movingObjects) {
                if ((!b1.equals(b2)) && (collisionBox.intersects(b2.getRect()))) {
                    return false;
                }
            }

            for (StaticObstacle o: ps.getStaticObstacles()) {
                if (collisionBox.intersects(o.getRect())) {
                    return false;
                }
            }
        }
        return true;
    }
}
