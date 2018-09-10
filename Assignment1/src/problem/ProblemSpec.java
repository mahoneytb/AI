package problem;

import java.awt.*;
import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.InputMismatchException;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Scanner;

/**
 * This class represents the specifications of a given problem and solution;
 * that is, it provides a structured representation of the contents of a problem
 * text file and associated solution text file, as described in the assignment
 * specifications.
 * 
 * This class doesn't do any validity checking - see the code in tester.Tester
 * for this.
 * 
 * @author Sergiy Dudnikov
 */
public class ProblemSpec {
	/** True iff a problem is currently loaded */
	private boolean problemLoaded = false;
	/** True iff a solution is currently loaded */
	private boolean solutionLoaded = false;

    /** The static obstacles */
	private List<StaticObstacle> staticObstacles;

    /** The static obstacles */
	private double robotWidth;

	/** The initial configuration */
	private RobotConfig initialRobotConfig;

	/** An array of moving boxes and obstacles */	
	private List<Box> movingBoxes;
    private List<Box> movingObstacles;
    private List<Point2D> movingBoxEndPositions;
    private List<Point2D> movingBoxesStartPositions;
    private List<Point2D> movingObstaclesStartPositions;

	/** The number of each type of obstacle **/	
    private int numMovingBoxes;
    private int numMovingObstacles;
	private int numStaticObstacles;
	
	/** The path of the robot, moving boxes, and moving obstacles **/		
	private List<RobotConfig> robotPath = new ArrayList<>();
	private List<List<Box>> movingBoxPath = new ArrayList<>();
    private List<List<Box>> movingObstaclePath = new ArrayList<>();

	
	/** Returns the width of the robot **/		
	public double getRobotWidth() {return robotWidth;}

	/** Returns a list of static obstacles **/		
    public List<StaticObstacle> getStaticObstacles() {return staticObstacles;}

	/** Returns the initial robot config **/		
    public RobotConfig getInitialRobotConfig() { return initialRobotConfig; }

	/** Returns a list of moving boxes **/		
	public List<Box> getMovingBoxes() { return movingBoxes; }

	/** Returns a list of moving obstacles **/		
	public List<Box> getMovingObstacles() { return movingObstacles; }

	/** Returns the robot path **/		
	public List<RobotConfig> getRobotPath() { return robotPath;}

	/** Returns the moving box path **/		
	public List<List<Box>> getMovingBoxPath() { return movingBoxPath; }

	/** Returns the moving obstacle path **/		
	public List<List<Box>> getMovingObstaclePath() { return movingObstaclePath; }

	public List<Point2D> getMovingBoxEndPositions() { return movingBoxEndPositions; }

	public boolean getProblemLoaded() { return problemLoaded; }

	public boolean getSolutionLoaded() { return solutionLoaded; }

    /**
	 * Loads a problem from a problem text file.
	 * 
	 * @param filename
	 *            the path of the text file to load.
	 * @throws IOException
	 *             if the text file doesn't exist or doesn't meet the assignment
	 *             specifications.
	 */
	public void loadProblem(String filename) throws IOException {
		problemLoaded = false;
		BufferedReader input = new BufferedReader(new FileReader(filename));
		String line;
		int lineNo = 0;
		Scanner s;
		try {

			// line 1
			line = input.readLine();
			lineNo++;
			s = new Scanner(line);

			robotWidth = s.nextDouble();
			initialRobotConfig =  new RobotConfig(
				new Point2D.Double(s.nextDouble(), s.nextDouble()), s.nextDouble());
			s.close();

			// line 2
			line = input.readLine();
			lineNo++;
			s = new Scanner(line);
			numMovingBoxes = s.nextInt();
			numMovingObstacles = s.nextInt();
			numStaticObstacles = s.nextInt();
			s.close();

			// this section covers moving boxes
			movingBoxEndPositions = new ArrayList<Point2D>();
			movingBoxes = new ArrayList<>();
			movingBoxesStartPositions = new ArrayList<Point2D>();
			movingObstaclesStartPositions = new ArrayList<Point2D>();
			for (int i = 0; i < numMovingBoxes; i++) {
				line = input.readLine();
				lineNo++;
				s = new Scanner(line);
				// The box creation function requires the bottom left corner.
				movingBoxes.add(new MovingBox(
					new Point2D.Double(s.nextDouble()-robotWidth/2,
										s.nextDouble()-robotWidth/2),	robotWidth));
				movingBoxesStartPositions.add(movingBoxes.get(i).getPos());
				movingBoxEndPositions.add(
					new Point2D.Double(s.nextDouble()-robotWidth/2,
										s.nextDouble()-robotWidth/2));
				s.close();
			}

            movingObstacles = new ArrayList<>();

            // this section covers moving Obstacles (still boxes)
			for (int i = 0; i < numMovingObstacles; i++) {
				line = input.readLine();
				lineNo++;
				s = new Scanner(line);
				// The box creation take the bottom left corner
				double x = s.nextDouble();
				double y = s.nextDouble();
				double w = s.nextDouble();
				movingObstacles.add(new MovingObstacle(
					new Point2D.Double(x-w/2, y-w/2),	w));
				movingObstaclesStartPositions.add(movingObstacles.get(i).getPos());
				s.close();
			}
			
			// this section represents static staticObstacles
			staticObstacles = new ArrayList<StaticObstacle>();
			for (int i = 0; i < numStaticObstacles; i++) {
				line = input.readLine();
				lineNo++;
				staticObstacles.add(new StaticObstacle(line));
			}
			
			problemLoaded = true;
		} catch (InputMismatchException e) {
			System.out.format("Invalid number format on input file - line %d: %s", lineNo,
                    e.getMessage());
			System.exit(1);
		} catch (NoSuchElementException e) {
            System.out.format("Not enough tokens on input file - line %d",
                    lineNo);
            System.exit(2);
		} catch (NullPointerException e) {
            System.out.format("Input file - line %d expected, but file ended.", lineNo);
            System.exit(3);
		} finally {
			input.close();
		}
	}

    /**
	 * Loads a solution from a solution text file.
	 * 
	 * @param filename
	 *            the path of the text file to load.
	 * @throws IOException
	 *             if the text file doesn't exist or doesn't meet the assignment
	 *             specifications.
	 */
    public void loadSolution(String filename) throws IOException {
        solutionLoaded = false;
        if (!problemLoaded) {
            System.out.println("Problem not loaded, exiting!");
            System.exit(4);
        }

        BufferedReader input = new BufferedReader(new FileReader(filename));
        String line;
        int lineNo = 0;
        Scanner s;
        try {
            // line 1
            line = input.readLine();
            lineNo++;
            s = new Scanner(line);
            int p = s.nextInt();
            s.close();

            for (int i = 0; i < p; i++) {
                line = input.readLine();
                lineNo++;
                s = new Scanner(line);
                robotPath.add(new RobotConfig(
                        new Point2D.Double(s.nextDouble(),s.nextDouble()),
                                s.nextDouble()));
                List<Box> movingBoxState = new ArrayList<>();
                for (int j = 0; j < numMovingBoxes; j++) {
                    movingBoxState.add(new MovingBox(new Point2D.Double(s.nextDouble() - robotWidth/2,
																		s.nextDouble() - robotWidth/2),
																		robotWidth));
                }
                movingBoxPath.add(movingBoxState);
                List<Box> movingObstacleState = new ArrayList<>();
                for (int k = 0; k < numMovingObstacles; k++) {
                    movingObstacleState.add(new MovingObstacle(new Point2D.Double(s.nextDouble() - movingObstacles.get(k).getWidth() / 2,
							s.nextDouble() - movingObstacles.get(k).getWidth() / 2),
                            movingObstacles.get(k).getWidth()));
                }
                movingObstaclePath.add(movingObstacleState);
                s.close();
            }
            solutionLoaded = true;
        } catch (InputMismatchException e) {
            System.out.format("Invalid number format on input file - line %d: %s", lineNo,
                    e.getMessage());
            System.exit(1);
        } catch (NoSuchElementException e) {
            System.out.format("Not enough tokens on input file - line %d",
                    lineNo);
            System.exit(2);
        } catch (NullPointerException e) {
            System.out.format("Input file - line %d expected, but file ended.", lineNo);
            System.exit(3);
        } finally {
            input.close();
        }
    }

    public void reset() {
    	for (int i = 0; i < numMovingBoxes; i++) {
    		Box b = getMovingBoxes().get(i);
    		b.pos = movingBoxesStartPositions.get(i);
		}
		for (int i = 0; i < numMovingObstacles; i++) {
			Box b = getMovingBoxes().get(i);
			b.pos = movingBoxesStartPositions.get(i);
		}
	}

}
