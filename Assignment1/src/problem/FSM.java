package problem;
import tester.Tester;

import javax.xml.stream.FactoryConfigurationError;
import java.awt.*;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.*;
import java.util.List;

public class FSM {

    // Initialize transition variables
    private static int SEED = 1;
    private static int KEEP = -1;                       // Keep box in collision check
    private boolean goal_state = false;                 // Have we reached the final goal state?
    private int boxAttempt = 0;                         // Attempt number for box to goal mapping
    private int robotAttempt = 0;
    private int n_boxes;                                // Number of boxes
    private ProblemSpec ps;                             // Current problem spec
    private RobotConfig robot;                          // Current robot configuration
    private int current_box = -1;                            // The current box
    private Tester tester;                              // Tester used for testing solutions
    private double current_grade = COURSE;
    private ArrayList<RobotConfig> robot_samples = new ArrayList<>();       // Robot config C-space samples
    private ArrayList<RobotConfig> robotCollisionSet = new ArrayList<>();
    private ArrayList<Vertex> course_grid;              // Course grid for box connecting
    private ArrayList<Vertex> fine_grid;                // Fine grid for box connecting
    private ArrayList<Vertex> current_grid;             // The currently used grid, including box and goal positions
    private ArrayList<Vertex> grid_corners;             // Mapped pathway for box to goal, including only corners and
                                                        // start position
    private ArrayList<RobotConfig> robot_map;           // List of robot configs from a start position to a goal position
    private Pathway complete_path = new Pathway();
    boolean advanced = false;
    private ArrayList<RobotConfig> robotGoals;
    Random generator = new Random(SEED);

    // FSM states
    private static int START = 0;
    private static int BOX_SAMPLE = 1;
    private static int BOX_CONNECT = 2;
    private static int BOX_MAP = 3;
    private static int ROBOT_SAMPLE = 4;
    private static int ROBOT_MAP = 5;
    private static int ADD_TO_PATH = 6;
    private static int OBSTACLE_CHECK = 7;

    // Course and fine grid gradients
    private static double COURSE = 0.05;
    private static double FINE = 0.02;

    // Initial state
    private int current = START;

    // Load the problem spec
    public void loadProblem(ProblemSpec ps) {
        this.ps = ps;
        this.n_boxes = ps.getMovingBoxes().size();
        robot = ps.getInitialRobotConfig();
        tester = new Tester(ps);
        complete_path.loadProblem(ps);
    }

    // Initialize array of methods calls to states
    interface ExecuteStateAction {
        void ExecuteState();
    }

    private ExecuteStateAction[] States = new ExecuteStateAction[] {
            new ExecuteStateAction() { public void ExecuteState() { START(); } },
            new ExecuteStateAction() { public void ExecuteState() { BOX_SAMPLE(); } },
            new ExecuteStateAction() { public void ExecuteState() { BOX_CONNECT(); } },
            new ExecuteStateAction() { public void ExecuteState() { BOX_MAP(); } },
            new ExecuteStateAction() { public void ExecuteState() { ROBOT_SAMPLE(); } },
            new ExecuteStateAction() { public void ExecuteState() { ROBOT_MAP(); } },
            new ExecuteStateAction() { public void ExecuteState() { ADD_TO_PATH(); } },
            new ExecuteStateAction() { public void ExecuteState() { OBSTACLE_CHECK(); } },
    };

    private void ExecuteState(int index) {
        States[index].ExecuteState();
    }

    // While loop which chews through states
    public boolean solve() {
        while (!goal_state) {
            ExecuteState(current);
        }
        complete_path.createSolutionFile();
        ps.reset();
        try {
            ps.loadSolution("solution.txt");
            tester.testSolution();

        } catch (IOException e1) {
            System.out.println("FAILED: Invalid problem file");
            System.out.println(e1.getMessage());
        }
        System.out.println("COMPLETED GOAL");
        return true;
    }

    //*********************************************   FSM   ************************************************************

    private void START() {
        // Reset all carried variables?
        current_box = findClosest(ps.getMovingBoxes(), robot.getPos(), false);
        current = BOX_SAMPLE;
    }

    // Sample C space for box to reach goal, ignore robot and movable boxes for now
    private void BOX_SAMPLE() {
        Box b = ps.getMovingBoxes().get(current_box);
        // Check goal is not obstructed
        if (goalObstructed(b)) {
            System.out.println("Goal is obstructed!");
            b.goalFailed = true;
            current = OBSTACLE_CHECK;
            return;
        }
        constructGrids();
        // First attempt is a course grid sample
        if (boxAttempt == 0) {
            current_grade = COURSE;
            current_grid = BoxToGrid(b, current_grade);
            current = BOX_CONNECT;
        }
        // Next attempt is a fine grid sample
        else if (boxAttempt == 1) {
            System.out.println("Using fine grid");
            current_grade = FINE;
            current_grid = BoxToGrid(b, current_grade);
            current = BOX_CONNECT;
        }
        else if (boxAttempt == 2) {
            System.out.println("Fine grid did not work, checking obstacles");
            b.goalFailed = true;
            current = OBSTACLE_CHECK;
        }
    }

    // Evaluate collisions and connect vertices
    private void BOX_CONNECT() {
        // Check if any vertices are collisions
        checkGridCollisions(true);
        for (Vertex v : current_grid) {
            if (v.collisionFree) {
                // Check if path between vertices is collision free
                checkVertexConnections(ps.getMovingBoxes().get(current_box), v);
            }
        }
        // Assert bi-directional connections
        populateNeighbours(current_grid);
        current = BOX_MAP;
    }

    // Map path with informed search
    private void BOX_MAP() {
        // Second to last element is initial position --See BoxToGrid
        Vertex start = current_grid.get(current_grid.size()-2);
        // Last element is goal position --See BoxToGrid
        Vertex goal = current_grid.get(current_grid.size()-1);
        A_Star map = new A_Star(start, goal, current_grade);
        if (map.compute()) {
            System.out.println("Box grid mapped for box: " + current_box);
            grid_corners = map.pathway;
            current = ROBOT_SAMPLE;
        } else {
            boxAttempt += 1;
            current = BOX_SAMPLE;
        }
    }

    // Iterate through box pathway, sample robot positions between start and turns
    private void ROBOT_SAMPLE() {
        boxAttempt = 0;
        Vertex goal = current_grid.get(current_grid.size()-1);
        handleSameCorners(goal);

        if (robotAttempt == 0) { // only return legal goals
            robotGoals = getRobotGoalPositions(grid_corners.get(grid_corners.size() - 1),
                    ps.getMovingBoxes().get(current_box));
            robot_samples.clear();
        }
        if (robotAttempt > 9) {
            resetRobotSampler();
            System.out.println("Could not solve robot to goal");
            Box b = ps.getMovingBoxes().get(current_box);
            b.goalFailed = true;
            current_box = findClosest(ps.getMovingBoxes(), robot.getPos(), true);
            current = BOX_SAMPLE;
            return;
        }
        if (!advanced || (robotCollisionSet.size()<4)) {
            if (robotGoals.isEmpty()) {
                resetRobotSampler();
                current_box = findClosest(ps.getMovingBoxes(), robot.getPos(), true);
                current = BOX_SAMPLE;
                return;
            }
            robotSample(robot, robotGoals);
            if (robotAttempt > 0) {
                advanced = true;
            }
        } else {
            System.out.println("Advanced robot sampling");
            advancedSample();
            advanced = false;
        }
        current = ROBOT_MAP;
    }

    private void ROBOT_MAP() {
        for (RobotConfig r : robot_samples) {
            r.g = 0;
            r.cost = 0;
        }

        A_Star_Robot map = new A_Star_Robot(robot, robotGoals);
        if (map.compute()) {
            resetRobotSampler();
            robot_map = map.pathway;
            Box b = ps.getMovingBoxes().get(current_box);
            b.goalFailed = false;
            current = ADD_TO_PATH;
        } else {
            robotAttempt += 1;
            current = ROBOT_SAMPLE;
        }
    }

    private void ADD_TO_PATH() {
        moveRobot(robot_map);
        moveBox(grid_corners, ps.getMovingBoxes().get(current_box));

        goal_state = true;
        for (int i = 0; i < ps.getMovingBoxes().size(); i++) {
            double bx = round(ps.getMovingBoxEndPositions().get(i).getX(), 6);
            double by = round(ps.getMovingBoxEndPositions().get(i).getY(), 6);
            Point2D bxy = new Point2D.Double(bx, by);
            if (!ps.getMovingBoxes().get(i).getPos().equals(bxy)) {
                goal_state = false;
            }
        }
        if (!(grid_corners.size() > 1)) { System.out.println("Box: " + current_box + " delivered to goal!"); }
        current = (grid_corners.size() > 1) ? ROBOT_SAMPLE : START;
    }

    private void OBSTACLE_CHECK() {
        current_box = obstacleSorting();
        boxAttempt = 0;
        current = (current == BOX_SAMPLE) ? BOX_SAMPLE : START;
    }

    // *************************************   Private functions   ****************************************************

    // Find closest box to robot
    private int findClosest(List<Box> boxes, Point2D pos, boolean notCurrent) {
        double shortest = 10;
        int index = 0;
        // Check each box
        for (int i = 0; i < boxes.size(); i++) {
            Box b = boxes.get(i);
            if (notCurrent && i == (current_box)) {
                continue;
            }
            //Check box is not already at goal
            double bx = round(ps.getMovingBoxEndPositions().get(i).getX(), 6);
            double by = round(ps.getMovingBoxEndPositions().get(i).getY(), 6);
            Point2D bxy = new Point2D.Double(bx, by);
            if (!b.pos.equals(bxy)) {
                double d = pos.distance(b.getPos());
                // Update closest if distance is shorter
                if (d < shortest) {
                    index = i;
                    shortest = d;
                }
            }
        }
        System.out.println("Trying box: " + index);
        return index;
    }

    // Test if the goal position is obstructed
    private boolean goalObstructed(Box b) {
        Point2D goal = ps.getMovingBoxEndPositions().get(ps.getMovingBoxes().indexOf(b));
        Vertex goalV = new Vertex(goal);

        goalV.pos = new Point2D.Double(round(goal.getX() + 0.001, 5), goal.getY());
        checkObstacleCollisions(b, goalV);
        if (!goalV.collisionFree) {return true;}
        goalV.pos = new Point2D.Double(round(goal.getX() - 0.001, 5), goal.getY());
        checkObstacleCollisions(b, goalV);
        if (!goalV.collisionFree) {return true;}
        goalV.pos = new Point2D.Double(goal.getX(), round(goal.getY() + 0.001, 5));
        checkObstacleCollisions(b, goalV);
        if (!goalV.collisionFree) {return true;}
        goalV.pos = new Point2D.Double(goal.getX(), round(goal.getY() - 0.001, 5));
        checkObstacleCollisions(b, goalV);
        return !goalV.collisionFree;
    }

    // Construct the C space box position course grids
    private void constructGrids() {
        course_grid = new ArrayList<>();
        fine_grid = new ArrayList<>();
        boolean iCourse = true;
        boolean jCourse;
        for (double i = COURSE; i<1; i=round(i+COURSE, 2)) {
            for (double j = COURSE; j < 1; j = round(j + COURSE, 2)) {
                course_grid.add(new Vertex(new Point2D.Double(i, j)));
            }
        }
        for (double i = FINE; i<1; i=round(i+FINE, 2)) {
            for (double j = FINE; j < 1; j = round(j+FINE, 2)) {
                fine_grid.add(new Vertex(new Point2D.Double(i,j)));
            }
        }

        // Add neighbours to vertices
        int ROW_LENGTH = (int)Math.sqrt(course_grid.size());
        for (int i=0; i<course_grid.size()-1; i++) {
            // Last row
            if (i >= course_grid.size()-ROW_LENGTH) {
                course_grid.get(i).addNeighbour(course_grid.get(i+1));
            }
            // Last column
            else if ((i+1)%ROW_LENGTH == 0) {
                course_grid.get(i).addNeighbour(course_grid.get(i+ROW_LENGTH));
            }
            // All other vertices
            else {
                course_grid.get(i).addNeighbour(course_grid.get(i+1));
                course_grid.get(i).addNeighbour(course_grid.get(i+ROW_LENGTH));
            }
        }

        ROW_LENGTH = (int)Math.sqrt(fine_grid.size());
        for (int i=0; i<fine_grid.size()-1; i++) {
            // Last row
            if (i >= fine_grid.size()-ROW_LENGTH) {
                fine_grid.get(i).addNeighbour(fine_grid.get(i+1));
            }
            // Last column
            else if ((i+1)%ROW_LENGTH == 0) {
                fine_grid.get(i).addNeighbour(fine_grid.get(i+ROW_LENGTH));
            }
            // All other vertices
            else {
                fine_grid.get(i).addNeighbour(fine_grid.get(i+1));
                fine_grid.get(i).addNeighbour(fine_grid.get(i+ROW_LENGTH));
            }
        }
    }

    // Add points in line with box to the grid
    private ArrayList<Vertex> BoxToGrid(Box b, double grade) {
        boolean isBox = false;
        double xg = 0, yg = 0;
        int idx = -1;
        // Only add vertices if box is not already in line with grid
        double xb = round(b.getPos().getX(), 6);
        xb = (xb % grade == 0) ? 0 : xb;
        double yb = round(b.getPos().getY(), 6);
        yb = (yb % grade == 0) ? 0 : yb;

        if (ps.getMovingBoxes().contains(b)) {
            isBox = true;
            idx = ps.getMovingBoxes().indexOf(b);
            xg = round(ps.getMovingBoxEndPositions().get(idx).getX(), 6);
            xg = (xg % grade == 0) ? 0 : xg;
            yg = round(ps.getMovingBoxEndPositions().get(idx).getY(), 6);
            yg = (yg % grade == 0) ? 0 : yg;
        }

        ArrayList<Vertex> newGrid;
        if (grade == COURSE) { newGrid = course_grid; }
        else { newGrid = fine_grid; }

        for (double i=grade; i < 1; i=round(i+grade, 2)) {

            if (yb != 0) {
                Vertex v1 = new Vertex(new Point2D.Double(i, yb));
                findNeighbours(v1, newGrid, grade);
                newGrid.add(v1);
            }

            if (xb != 0) {
                Vertex v2 = new Vertex(new Point2D.Double(xb, i));
                findNeighbours(v2, newGrid, grade);
                newGrid.add(v2);
            }

            if (isBox && (idx == current_box)) {

                if (yg != 0) {
                    Vertex v3 = new Vertex(new Point2D.Double(i, yg));
                    findNeighbours(v3, newGrid, grade);
                    newGrid.add(v3);
                }

                if (xg != 0) {
                    Vertex v4 = new Vertex(new Point2D.Double(xg, i));
                    findNeighbours(v4, newGrid, grade);
                    newGrid.add(v4);
                }
            }
        }

        if (isBox && (idx == current_box)) {
            if ((xg != 0) && (yg != 0)) {
                if ((yg != yb) && (xg != xb)) {
                    Vertex vgb1 = new Vertex(new Point2D.Double(xb, yg));
                    findNeighbours(vgb1, newGrid, grade);
                    newGrid.add(vgb1);
                    Vertex vgb2 = new Vertex(new Point2D.Double(xg, yb));
                    findNeighbours(vgb2, newGrid, grade);
                    newGrid.add(vgb1);
                }
            }
        }

        // Add current position
        if ((xb != 0) && (yb != 0)) {
            Vertex vb = new Vertex(new Point2D.Double(xb, yb));
            findNeighbours(vb, newGrid, grade);
            newGrid.add(vb);
        }

        // Add goal position
        if (isBox && (idx == current_box)) {
            if ((xg != 0) && (yg != 0)) {
                Vertex vg = new Vertex(new Point2D.Double(xg, yg));
                findNeighbours(vg, newGrid, grade);
                newGrid.add(vg);
            }
        }
        return newGrid;
    }

    // Find neighbours for newly added vertices
    private void findNeighbours(Vertex v, ArrayList<Vertex> grid, double d) {
        for (Vertex p : grid) {
            // Must be within distance
            if (round(p.getPos().distance(v.getPos()), 3) <= d) {
                // Must be new neighbour
                if (!p.getNeighbours().contains(v) && !v.getNeighbours().contains(p)) {
                    // Must not be diagonal neighbour
                    double vx = round(v.getPos().getX(), 5);
                    double vy = round(v.getPos().getY(), 5);
                    double px = round(p.getPos().getX(), 5);
                    double py = round(p.getPos().getY(), 5);
                    // Check if in line with v
                    if ((px == vx) || (py == vy)) {
                        v.addNeighbour(p);
                    }
                }
            }
        }
    }

    private void checkGridCollisions(boolean ignoreRobot) {
        double width = ps.getMovingBoxes().get(current_box).getWidth();
        // Check collisions of each vertex
        for (Vertex v : current_grid) {
            List<Box> newBoxConfig = new ArrayList<>();
            newBoxConfig.addAll(ps.getMovingBoxes());
            // Update box position to grid
            Box newPos = new MovingBox(v.getPos(), width);
            newBoxConfig.set(current_box, newPos);
            // Include movable obstacles
            newBoxConfig.addAll(ps.getMovingObstacles());
            ArrayList<Box> found = new ArrayList<>();
            for (Box b : newBoxConfig) {
                if (!b.include) {
                    found.add(b);
                }
            }
            newBoxConfig.removeAll(found);
            v.collisionFree = tester.hasCollision(robot, newBoxConfig, ignoreRobot);
        }
    }

    // Check collision for a single vertex (ie. move obs to vertex and check collision)
    private void checkObstacleCollisions(Box obs, Vertex v) {
        double width = obs.getWidth();
        // Check collisions of each vertex
        List<Box> newBoxConfig = new ArrayList<>();
        newBoxConfig.addAll(ps.getMovingBoxes());
        newBoxConfig.addAll(ps.getMovingObstacles());
        // Update box position to grid
        Box newPos = new MovingBox(v.getPos(), width);
        newBoxConfig.remove(obs);
        newBoxConfig.add(newPos);
        ArrayList<Box> found = new ArrayList<>();
        for (Box b : newBoxConfig) {
            if (!b.include) {
                found.add(b);
            }
        }
        newBoxConfig.removeAll(found);
        v.collisionFree = tester.hasCollision(robot, newBoxConfig, true);
    }

    // Check the connectivity between neighbours in box grid, uses BFS
    private void checkVertexConnections(Box box, Vertex v) {
        // Check each neighbour, result is stored in Vertex class map
        for (Vertex n : v.getNeighbours()) {
            checkObstacleCollisions(box, n);
            if (!n.collisionFree) {
                v.setConnect(n, false);
                continue;
            }
            Vertex copy_n = new Vertex(n.getPos());
            Vertex copy_v = new Vertex(v.getPos());
            copy_n.addNeighbour(copy_v);

            Queue<Vertex> queue = new LinkedList<>();
            queue.add(copy_n);

            // BFS of splits along edge between vertices, no need to check if explored as each child is unique
            while (!queue.isEmpty()) {
                Vertex current = queue.remove();
                // Found collision along connection
                if (!current.collisionFree) {
                    v.setConnect(n, false);
                    break;
                } else {
                    ArrayList<Vertex> children = split(current);
                    if (children != null) {
                        queue.addAll(children);
                        for (Vertex child : children) {
                            checkObstacleCollisions(box, child);
                        }
                    }
                }
            }
        }
    }

    // Return split vertices with neighbours of either side
    private ArrayList<Vertex> split(Vertex v) {
        ArrayList<Vertex> children = new ArrayList<Vertex>();
        // For each neighbour, create a middle child and return
        for (Vertex n : v.getNeighbours()) {
            // Might need to double check this, trying to only add children if over the movement limit
            if (n.getPos().distance(v.getPos())<0.001) { return children; }
            Point2D mid = new Point2D.Double((n.getPos().getX() + v.getPos().getX()) / 2,
                    (n.getPos().getY() + v.getPos().getY()) / 2);
            Vertex midV = new Vertex(mid);
            midV.addNeighbour(n);
            midV.addNeighbour(v);
            children.add(midV);
        }
        return children;
    }

    /* Until this point, neighbours have not been bi-directional in order to reduce double handling. This function
     * fully populates the neighbour network of the grid for collision free connections prior to pathfinding.
     */
    private void populateNeighbours(ArrayList<Vertex> grid) {
        for (Vertex v : grid) {
            if (v.collisionFree) {
                for (Vertex n : v.getNeighbours()) {
                    if (!n.equals(v)) {
                        if (n.collisionFree) {
                            if (v.queryConnect(n)) {
                                if (!n.getNeighbours().contains(v)) {
                                    n.addNeighbour(v);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Round double to d decimal places
    private double round(double num, int d) {
        num = num * Math.pow(10, d);
        num = Math.round(num);
        num = num / Math.pow(10, d);
        return num;
    }

    // Get the robot push position for the box when at vertex v
    private ArrayList<RobotConfig> getRobotGoalPositions(Vertex v, Box b) {
        char d = v.direction;
        ArrayList<RobotConfig> goals = new ArrayList<>(0);
        double gx1 = 0, gx2 = 0, gx3 = 0, gy1 = 0, gy2 = 0, gy3 = 0, galpha = 0, w = b.getWidth();
        double BUFFER = round(w/20, 6);
        switch (d) {
            case 'l':
                gx1 = round(v.getPos().getX() + w, 6);
                gy1 = round(v.getPos().getY() + w/4 + BUFFER, 6);
                gy2 = round(v.getPos().getY() + w/2, 6);
                gy3 = round(v.getPos().getY() + 3*w/4 - BUFFER, 6);
                galpha = round(Math.PI / 2, 6);
                goals.add( new RobotConfig(new Point2D.Double(gx1, gy1), galpha));
                goals.add( new RobotConfig(new Point2D.Double(gx1, gy2), galpha));
                goals.add( new RobotConfig(new Point2D.Double(gx1, gy3), galpha));
                break;
            case 'r':
                gx1 = round(v.getPos().getX(), 6);
                gy1 = round(v.getPos().getY() + w/4 + BUFFER, 6);
                gy2 = round(v.getPos().getY() + w/2, 6);
                gy3 = round(v.getPos().getY() + 3*w/4 - BUFFER, 6);
                galpha = round(Math.PI / 2, 6);
                goals.add( new RobotConfig(new Point2D.Double(gx1, gy1), galpha));
                goals.add( new RobotConfig(new Point2D.Double(gx1, gy2), galpha));
                goals.add( new RobotConfig(new Point2D.Double(gx1, gy3), galpha));
                break;
            case 'u':
                gx1 = round(v.getPos().getX() + w/4 + BUFFER, 6);
                gx2 = round(v.getPos().getX() + w/2, 6);
                gx3 = round(v.getPos().getX() + 3*w/4 - BUFFER, 6);
                gy1 = round(v.getPos().getY(), 6);
                galpha = 0;
                goals.add( new RobotConfig(new Point2D.Double(gx1, gy1), galpha));
                goals.add( new RobotConfig(new Point2D.Double(gx2, gy1), galpha));
                goals.add( new RobotConfig(new Point2D.Double(gx3, gy1), galpha));
                break;
            case 'd':
                gx1 = round(v.getPos().getX() + w/4 + BUFFER, 6);
                gx2 = round(v.getPos().getX() + w/2, 6);
                gx3 = round(v.getPos().getX() + 3*w/4 - BUFFER, 6);
                gy1 = round(v.getPos().getY() + w, 6);
                galpha = 0;
                goals.add( new RobotConfig(new Point2D.Double(gx1, gy1), galpha));
                goals.add( new RobotConfig(new Point2D.Double(gx2, gy1), galpha));
                goals.add( new RobotConfig(new Point2D.Double(gx3, gy1), galpha));
                break;
        }
        if (goals.size() == 0) {return null;}

        checkRobGoalsAreValid(goals, b);

        if (goals.size() == 0) {return null;}

        ArrayList<RobotConfig> opposites = new ArrayList<>();
        for (RobotConfig r : goals) {
            opposites.add(createOpposite(r));
        }
        goals.addAll(opposites);

        ArrayList<RobotConfig> fails = new ArrayList<>();
        ArrayList<Box> allBoxes = new ArrayList<>();
        allBoxes.addAll(ps.getMovingBoxes());
        allBoxes.addAll(ps.getMovingObstacles());
        for (RobotConfig g : goals) {
            if (!tester.hasCollision(g, allBoxes, false)) {
                fails.add(g);
            }
        }
        goals.removeAll(fails);
        return goals;
    }

    // This checks if the robot will have any collisions hanging of the end of the box while pushing
    private void checkRobGoalsAreValid (ArrayList<RobotConfig> goals, Box box) {
        Vertex start = grid_corners.get(grid_corners.size()-1);
        Vertex end = grid_corners.get(grid_corners.size()-2);
        double xDif = round(end.getPos().getX() - start.getPos().getX(), 6);
        double yDif = round(end.getPos().getY() - start.getPos().getY(), 6);

        if ((xDif == 0) && (yDif == 0)) { return; }
        ArrayList<RobotConfig> fails = new ArrayList<>();
        for (int i = 0; i < goals.size(); i++) {
            if (i == 1) { continue; }
            RobotConfig g = goals.get(i);
            Point2D g_end_p = new Point2D.Double(g.getPos().getX()+xDif, g.getPos().getY()+yDif);
            RobotConfig g_end = new RobotConfig(g_end_p, g.getOrientation());
            ArrayList<RobotConfig> check = new ArrayList<>();
            check.add(g);
            checkRobotConnections(g_end, check, box);
            if (!g.neighbours.contains(g_end)) { fails.add(g); }
        }
        goals.removeAll(fails);
    }

    // Return 100 tested and connected robotConfigs, mostly between r and g
    private void robotSample(RobotConfig r, ArrayList<RobotConfig> goals) {
        ArrayList<RobotConfig> samples = robot_samples;

        if (robotAttempt == 0) {
            samples.clear();
            // Check direct start to goal connection
            samples.add(r);

            for (RobotConfig g : goals) {
                checkRobotConnections(g, samples, null);
                if (g.neighbours.contains(r)) {
                    samples.add(g);
                    return;
                }
            }

            samples.addAll(goals);

            addBasicMoves(samples);
            return;
        }

        RobotConfig g;
        if (goals.size() > 1) { g = goals.get(1); } // The middle goal
        else g = goals.get(0);
        // Get bounds for probable samples: 0.1 outside of current max and min positions
        double BOUND = 0.1 * (robotAttempt);
        double xmax = (r.getPos().getX() > g.getPos().getX()) ? r.getPos().getX() : g.getPos().getX();
        xmax = ((xmax + BOUND) > 1) ? 1 : (xmax + BOUND);
        double xmin = (r.getPos().getX() > g.getPos().getX()) ? g.getPos().getX() : r.getPos().getX();
        xmin = ((xmin - BOUND) < 0) ? 0 : (xmin - BOUND);

        double ymax = (r.getPos().getY() > g.getPos().getY()) ? r.getPos().getY() : g.getPos().getY();
        ymax = ((ymax + BOUND) > 1) ? 1 : (ymax + BOUND);
        double ymin = (r.getPos().getY() > g.getPos().getY()) ? g.getPos().getY() : r.getPos().getY();
        ymin = ((ymin - BOUND) < 0) ? 0 : (ymin - BOUND);

        double alphamax = (r.getOrientation() > g.getOrientation()) ? r.getOrientation() : g.getOrientation();
        double alphamin = (r.getOrientation() > g.getOrientation()) ? g.getOrientation() : r.getOrientation();

        ArrayList<Box> newBoxConfig = new ArrayList<>();
        newBoxConfig.addAll(ps.getMovingBoxes());
        newBoxConfig.addAll(ps.getMovingObstacles());

        // Sample 100 times with 70% chance of forcing being between the robot current position and the goal
        int n = 0;
        while (n<50) {
            double x, y, alpha;
            // 70% we choose a point in the start to goal area
            if (generator.nextDouble() > 0.3) {
                x = generator.nextDouble() * (xmax - xmin) + xmin;
                y = generator.nextDouble() * (ymax - ymin) + ymin;
                alpha = generator.nextDouble() * (alphamax - alphamin) + alphamin;
                // Introduce opposite robot orientations
                alpha += (generator.nextDouble() > 0.5) ? Math.PI : 0;
                alpha -= (alpha > (Math.PI * 2)) ? Math.PI * 2 : 0;
            } else {
                x = generator.nextDouble();
                y = generator.nextDouble();
                alpha = generator.nextDouble();
            }
            //alpha = generator.nextDouble()*Math.PI*2; // Keep alpha random
            RobotConfig s = new RobotConfig(new Point2D.Double(x, y), alpha);

            // Check for collisions
            if (!tester.hasCollision(s, newBoxConfig, false)) {
                robotCollisionSet.add(s);
                continue;
            }

            // Check for connections
            checkRobotConnections(s, samples, null);
            n++;
            RobotConfig s2 = createOpposite(s);
            checkRobotConnections(s2, samples, null);
            n++;
            samples.add(s);
            samples.add(s2);

            // Early finish if the current sample bridges the start and goal positions
            if (s.neighbours.contains(r)) {
                for (RobotConfig goal : goals) {
                    if (s.neighbours.contains(goal)) {
                        samples.clear();
                        samples.add(r);
                        samples.add(g);
                        samples.add(s);
                        return;
                    }
                }
            }
            if (s2.neighbours.contains(r)) {
                for (RobotConfig goal : goals) {
                    if (s.neighbours.contains(goal)) {
                        samples.clear();
                        samples.add(r);
                        samples.add(g);
                        samples.add(s);
                        return;
                    }
                }
            }
        }
        return;
    }

    // Sample more frequently about collisions to find gap
    private void advancedSample() {

        ArrayList<Box> newBoxConfig = new ArrayList<>();
        newBoxConfig.addAll(ps.getMovingBoxes());
        newBoxConfig.addAll(ps.getMovingObstacles());

        // Sample 100 times with 70% chance of forcing being between the robot current position and the goal
        int n = 0;
        while (n<50) {
            double x, y, alpha;
            RobotConfig c = robotCollisionSet.get(generator.nextInt(robotCollisionSet.size()-1));
            double xmax = (c.getPos().getX() + 0.05 < 1) ? c.getPos().getX() - 0.05 : 1;
            double xmin = (xmax - 0.1 > 0) ? xmax - 0.1 : 0;
            double ymax = (c.getPos().getY() + 0.05 < 1) ? c.getPos().getX() - 0.05 : 1;
            double ymin = (xmax - 0.1 > 0) ? xmax - 0.1 : 0;

            x = generator.nextDouble()*(xmax - xmin) + xmin;
            y = generator.nextDouble()*(ymax - ymin) + ymin;

            // Focus on vertical and horizontal positions to navigate rectangular gaps
            double alphaDelta = generator.nextDouble()*0.4 + Math.PI - 0.2;
            int alphaOrientation = generator.nextInt(10);
            alpha = (alphaOrientation > 5) ? alphaDelta : alphaDelta + Math.PI/2;

            RobotConfig s = new RobotConfig(new Point2D.Double(x, y), alpha);

            // Check for collisions
            if (!tester.hasCollision(s, newBoxConfig, false)) {
                robotCollisionSet.add(s);
                continue;
            }

            // Check for connections
            checkRobotConnections(s, robot_samples, null);
            n++;
            RobotConfig s2 = createOpposite(s);
            checkRobotConnections(s2, robot_samples, null);
            n++;
            robot_samples.add(s);
            robot_samples.add(s2);
        }
        return;
    }

    // Adds basic left down right up movements from start and goal to sample space
    private void addBasicMoves(ArrayList<RobotConfig> samples) {
        double STEP = 0.01;
        ArrayList<RobotConfig> newMoves = new ArrayList<>();

        for (RobotConfig r : samples) {
            newMoves.add(new RobotConfig(new Point2D.Double(round(r.getPos().getX() - STEP, 6),
                    r.getPos().getY()), r.getOrientation()));
            newMoves.add(new RobotConfig(new Point2D.Double(round(r.getPos().getX() + STEP, 6),
                    r.getPos().getY()), r.getOrientation()));
            newMoves.add(new RobotConfig(new Point2D.Double(r.getPos().getX(),
                    round(r.getPos().getY() - STEP, 6)), r.getOrientation()));
            newMoves.add(new RobotConfig(new Point2D.Double(r.getPos().getX(),
                    round(r.getPos().getY() + STEP, 6)), r.getOrientation()));
        }

        samples.addAll(newMoves);

        // Mid moves
        RobotConfig r = samples.get(0);
        ArrayList<RobotConfig> newBridgeMoves = new ArrayList<>();
        for (int i = 1; i < samples.size(); i++) {
            RobotConfig g = samples.get(i);

            RobotConfig mid1 = new RobotConfig(new Point2D.Double(r.getPos().getX(), g.getPos().getY()), r.getOrientation());
            RobotConfig mid2 = new RobotConfig(new Point2D.Double(r.getPos().getX(), g.getPos().getY()), g.getOrientation());
            RobotConfig mid3 = new RobotConfig(new Point2D.Double(g.getPos().getX(), r.getPos().getY()), r.getOrientation());
            RobotConfig mid4 = new RobotConfig(new Point2D.Double(g.getPos().getX(), r.getPos().getY()), g.getOrientation());

            newBridgeMoves.add(mid1);
            newBridgeMoves.add(mid2);
            newBridgeMoves.add(mid3);
            newBridgeMoves.add(mid4);
        }
        samples.addAll(newBridgeMoves);

        ArrayList<Box> newBoxConfig = new ArrayList<>();
        newBoxConfig.addAll(ps.getMovingBoxes());
        newBoxConfig.addAll(ps.getMovingObstacles());

        ArrayList<RobotConfig> found = new ArrayList<>();
        for (RobotConfig s : samples) {
            if (!tester.hasCollision(s, newBoxConfig, false)) {
                found.add(s);
            }
        }
        samples.removeAll(found);
        for (RobotConfig s : samples) {
            // Check for connections
            checkRobotConnections(s, samples, null);
        }
    }

    private void checkRobotConnections(RobotConfig r, ArrayList<RobotConfig> samples, Box box) {
        ArrayList<Box> newBoxConfig = new ArrayList<>();
        newBoxConfig.addAll(ps.getMovingBoxes());
        newBoxConfig.addAll(ps.getMovingObstacles());
        if (box != null) {
            newBoxConfig.remove(box);
        }

        for (RobotConfig s : samples) {
            if (s.neighbours.contains(r) || s.equals(r)) {
                continue;
            }

            // Set up initial guess at valid step size (includes sign yay)
            int FACTOR = 10;
            double x_dist = round((s.getPos().getX() - r.getPos().getX()), 6);
            double y_dist = round((s.getPos().getY() - r.getPos().getY()), 6);
            double alpha_dist = (s.getOrientation() - r.getOrientation());

            if ((Math.abs(x_dist) > 0.1*(robotAttempt+1)) || (Math.abs(y_dist) > 0.1*(robotAttempt+1))) {
                continue;
            }

            RobotConfig test = new RobotConfig(new Point2D.Double(s.getPos().getX() + x_dist/FACTOR,
                    s.getPos().getY() + y_dist/FACTOR), s.getOrientation() + alpha_dist/FACTOR);

            // Decrease step size until valid
            boolean extraSure = false;
            while (!tester.isValidStep(s, test) || !extraSure) {
                if (tester.isValidStep(s, test)) {extraSure = true;}
                FACTOR *= 2;
                test.pos = new Point2D.Double(s.getPos().getX() + x_dist/FACTOR,
                        s.getPos().getY() + y_dist/FACTOR);
                test.angle = s.getOrientation() + alpha_dist/FACTOR;
            }

            // Add positions to pathway
            RobotConfig current = new RobotConfig(s.getPos(), s.getOrientation());
            boolean collision = false;
            int count = 1;
            while (!tester.isValidStep(current, r)) {
                current.pos = new Point2D.Double(round(s.getPos().getX() - count*x_dist/FACTOR, 6),
                        round(s.getPos().getY() - count*y_dist/FACTOR, 6));
                current.angle = round(s.getOrientation() - count*alpha_dist/FACTOR, 6);
                if (!tester.hasCollision(current, newBoxConfig, false)) {
                    collision = true;
                    break;
                }
                count++;
            }
            if (!collision) {
                r.neighbours.add(s);
                s.neighbours.add(r);
            }
        }
    }

    // Add robot positions along mapped path to the complete pathway
    private void moveRobot(ArrayList<RobotConfig> path) {

        List<Box> boxes = ps.getMovingBoxes();
        List<Box> obstacles = ps.getMovingObstacles();

        for (int i = path.size()-1; i > 0; i--) {
            RobotConfig start = path.get(i);
            RobotConfig end = path.get(i-1);

            // Set up initial guess at valid step size (includes sign yay)
            int FACTOR = 10;
            double x_dist = round((start.getPos().getX() - end.getPos().getX()), 6);
            double y_dist = round((start.getPos().getY() - end.getPos().getY()), 6);
            double alpha_dist = round((start.getOrientation() - end.getOrientation()), 6);

            RobotConfig test = new RobotConfig(new Point2D.Double(start.getPos().getX() + x_dist/FACTOR,
                    start.getPos().getY() + y_dist/FACTOR), start.getOrientation() + alpha_dist/FACTOR);

            // Decrease step size until valid
            boolean extraSure = false;
            while (!tester.isValidStep(start, test) || !extraSure) {
                if (tester.isValidStep(start, test)) {extraSure = true;}
                FACTOR *= 2;
                test.pos = new Point2D.Double(start.getPos().getX() + x_dist/FACTOR,
                        start.getPos().getY() + y_dist/FACTOR);
                test.angle = start.getOrientation() + alpha_dist/FACTOR;
            }

            // Add positions to pathway
            RobotConfig current = new RobotConfig(start.getPos(), start.getOrientation());
            int count = 1;
            while (!tester.isValidStep(current, end)) {
                current.pos = new Point2D.Double(round(start.getPos().getX() - count*x_dist/FACTOR, 6),
                        round(start.getPos().getY() - count*y_dist/FACTOR, 6));
                current.angle = round(start.getOrientation() - count*alpha_dist/FACTOR, 6);
                count++;
                complete_path.addToPath(current, boxes, obstacles);
            }
            complete_path.addToPath(end, boxes, obstacles);
            robot = end;
        }
    }

    // Add robot and box positions along mapped path to the complete pathway
    private void moveBox(ArrayList<Vertex> grid, Box b) {
        //if (grid.size() < 2) { return; }
        List<Box> boxes = ps.getMovingBoxes();
        List<Box> obstacles = ps.getMovingObstacles();

        // Get robot start position --hope this works...
        RobotConfig start = robot;//getRobotGoalPositions(grid.get(grid.size()-1), b);

        // Compute robot end position based on chang in box position
        double x_shift = grid.get(grid.size()-1).getPos().getX() - grid.get(grid.size()-2).getPos().getX();
        double y_shift = grid.get(grid.size()-1).getPos().getY() - grid.get(grid.size()-2).getPos().getY();
        double[] pos = {start.getPos().getX() - x_shift, start.getPos().getY() - y_shift};

        RobotConfig end = new RobotConfig(pos, start.getOrientation());

        // Set up initial guess at valid step size (includes sign yay)
        int FACTOR = 10;
        double x_dist = (start.getPos().getX() - end.getPos().getX());
        double y_dist = (start.getPos().getY() - end.getPos().getY());

        RobotConfig test = new RobotConfig(new Point2D.Double(start.getPos().getX() + x_dist/FACTOR,
                start.getPos().getY() + y_dist/FACTOR), start.getOrientation());

        // Decrease step size until valid
        while (!tester.isValidStep(start, test)) {
            FACTOR *= 2;
            test.pos = new Point2D.Double(start.getPos().getX() + x_dist/FACTOR,
                    start.getPos().getY() + y_dist/FACTOR);
        }

        // Add positions to pathway
        RobotConfig current_robot = new RobotConfig(start.getPos(), start.getOrientation());
        Box current_box = b;
        int count = 1;
        while (!tester.isValidStep(current_robot, end)) {
            current_robot.pos = new Point2D.Double(round(current_robot.getPos().getX() - count*x_dist/FACTOR, 6),
                    round(current_robot.getPos().getY() - count*y_dist/FACTOR, 6));
            current_box.pos = new Point2D.Double(round(current_box.getPos().getX() - count*x_dist/FACTOR, 6),
                    round(current_box.getPos().getY() - count*y_dist/FACTOR, 6));
           complete_path.addToPath(current_robot, boxes, obstacles);
        }
        robot = end;
        current_box.pos = new Point2D.Double(round(grid.get(grid.size()-2).getPos().getX(), 6),
                round(grid.get(grid.size()-2).getPos().getY(), 6));
        current_box.setRect();
        // Add the goal position
        complete_path.addToPath(robot, boxes, obstacles);
        // Delete last element of grid corners
        grid.remove(grid.size()-1);
    }

    private RobotConfig createOpposite(RobotConfig r) {
        double counterAlpha = r.getOrientation() + Math.PI;
        counterAlpha -= (counterAlpha > (2*Math.PI)) ? 2*Math.PI : 0;
        RobotConfig r2 = new RobotConfig(r.getPos(), counterAlpha);
        //r2.opposite_rotation = true;
        return r2;
    }

    // Recomputes a mapping for current box on current grid
    private boolean recheckBoxPath() {
        constructGrids();
        Box b = ps.getMovingBoxes().get(current_box);
        current_grid = BoxToGrid(b, FINE);

        checkGridCollisions(true);
        for (Vertex v : current_grid) {
            if (v.collisionFree) {
                // Check if path between vertices is collision free
                checkVertexConnections(b, v);
            }
        }
        // Assert bi-directional connections
        populateNeighbours(current_grid);

        // Second to last element is initial position --See BoxToGrid
        Vertex start = current_grid.get(current_grid.size()-2);
        // Last element is goal position --See BoxToGrid
        Vertex goal = current_grid.get(current_grid.size()-1);
        A_Star map = new A_Star(start, goal, current_grade);
        return map.compute();
    }

    private int obstacleSorting() {
        // First confirm there is a path! Include only the current box
        includeOnly(ps.getMovingBoxes().get(current_box));

        if (!recheckBoxPath()) {
            System.out.println("No solution found");
            goal_state = true;
            return -1;
        }

        // if yes keep adding boxes until no path found
        boolean allClear = false;
        ArrayList<Box> tried = new ArrayList<>();
        while (!allClear) {
            includeOnly(ps.getMovingBoxes().get(current_box));
            boolean found = false;
            Box obstruction = null;
            for (int i = 0; i < n_boxes; i++) {
                Box b = ps.getMovingBoxes().get(i);
                b.include = true;
                if (!found && !tried.contains(b) && !recheckBoxPath()) {
                    found = true;
                    obstruction = b;
                    if (!b.getPos().equals(ps.getMovingBoxEndPositions().get(i))) {
                        // goalFailed tells us the box has been attempted to move to goal but cannot
                        if (!b.goalFailed) {
                            // we attempt to get the box to the goal to solve the obstruction
                            System.out.println("Trying to get another box to goal");
                            current_box = i;
                            current = BOX_SAMPLE;
                            boxAttempt = 0;
                            allClear = true;
                        }
                    }
                }
            }
            for (Box b : ps.getMovingObstacles()) {
                b.include = true;
                if (!found && !tried.contains(b) && !recheckBoxPath()) {
                    obstruction = b;
                }
            }
            // We have found a box in the way and are yet to attempt to take it to goal, skip the rest and return to try
            if (allClear) { return current_box; }

            // If no obstruction found which we have not tried, we have failed
            if (obstruction == null) {
                return -1;
            }

            // Attempt to move box to goal position (g = no longer an obstruction for current box)
            if (moveObstruction(obstruction)) { tried.clear(); }
            // If move attempt fails we try another obstruction
            else { tried.add(obstruction); }

            constructGrids();
            BoxToGrid(ps.getMovingBoxes().get(current_box), current_grade);

            if (recheckBoxPath()) { allClear = true; }
        }
        // Try next box
        current_box = findClosest(ps.getMovingBoxes(), robot.getPos(), true);
        return current_box;
    }

    // Uses BFS to find closest position of box to move which will not be obstructing the path of current box
    private boolean moveObstruction(Box obstruction) {
        // Construct the grid for the obstruction to navigate
        constructGrids();
        ArrayList<Vertex> grid = BoxToGrid(obstruction, COURSE);

        // Assert bi-directional connections
        populateNeighbours(grid);
        // Second to last element is initial position --See BoxToGrid
        Vertex start = grid.get(grid.size()-1);

        // **************************************** BFS ************************************************

        ArrayList<Box> allBoxes = new ArrayList<>();
        allBoxes.addAll(ps.getMovingBoxes());
        allBoxes.addAll(ps.getMovingObstacles());
        Queue<Vertex> queue = new LinkedList<>();
        queue.add(start);
        Point2D initial = new Point2D.Double(start.getPos().getX(), start.pos.getY());
        Vertex goal = null;
        ArrayList<Vertex> explored = new ArrayList<>();
        includeOnly(obstruction);

        // BFS of splits along edge between vertices, no need to check if explored as each child is unique
        while (!queue.isEmpty()) {
            Vertex current = queue.remove();
            // Update box position
            obstruction.pos = current.pos;
            obstruction.setRect();
            // Check if solution
            if (recheckBoxPath()) {
                includeAll();
                goal = current;
                obstruction.pos = start.pos;
                obstruction.setRect();
                break;
            } else {
                for (Vertex n : current.getNeighbours()) {
                    if (!explored.contains(n) && n.collisionFree && n.queryConnect(current)) {
                        n.parent = current;
                        queue.add(n);
                    }
                }
            }
            explored.add(current);
        }

        obstruction.pos = initial;
        if (goal == null) { return false; }

        grid_corners = createPath(goal, start);

        robotAttempt = 0;
        boolean mapped = false;
        ArrayList<RobotConfig> obs_map = new ArrayList<>();
        ArrayList<RobotConfig> Rgoals = new ArrayList<>();
        while (robotAttempt < 9) {

            if (handleSameCorners(goal)) { return true; }

            if (robotAttempt == 0) {
                Rgoals = getRobotGoalPositions(grid_corners.get(grid_corners.size()-1), obstruction);
                robot_samples.clear();
            }
            if (!advanced || (robotCollisionSet.size()<4)) {
                if (Rgoals.isEmpty()) {
                    resetRobotSampler();
                    return false;
                }
                robotSample(robot, Rgoals);
                if (robotAttempt > 0) {
                    advanced = true;
                }
            } else {
                advanced = false;
                advancedSample();
            }
            A_Star_Robot map = new A_Star_Robot(robot, Rgoals);
            if (map.compute()) {
                resetRobotSampler();
                obs_map = map.pathway;
                moveRobot(obs_map);
                moveBox(grid_corners, obstruction);

                if (handleSameCorners(goal)) { return true; }
            } else {
                robotAttempt += 1;
            }
        }
        System.out.println("Could not solve robot to goal");
        return false;
    }

    // Construct pathway only including corner vertices
    private ArrayList<Vertex> createPath(Vertex v, Vertex start) {
        ArrayList<Vertex> pathway = new ArrayList<>();
        boolean finished = false;
        //pathway.add(v);
        while (!finished) {
            Vertex p;

            if (v.parent == null) {
                break;
            } else {
                p = v.parent;
            }

            // Recompute direction
            if (p.getPos().getX() < v.getPos().getX()) {
                p.direction = 'r';
            } else if (p.getPos().getX() > v.getPos().getX()) {
                p.direction = 'l';
            } else if (p.getPos().getY() > v.getPos().getY()) {
                p.direction = 'd';
            } else if (p.getPos().getY() < v.getPos().getY()) {
                p.direction = 'u';
            }

            // If a corner add to pathway
            if (p.direction != v.direction) {
                pathway.add(v);
            }
            v = p;
        }
        pathway.add(start);
        return pathway;
    }

    // Includes includeBox and current box only in recheckpathway
    private void includeOnly(Box includeBox) {
        for (int i = 0; i < n_boxes; i++) {
            Box b = ps.getMovingBoxes().get(i);
            b.include = (i == current_box) || b.equals(includeBox);
        }
        for (Box o : ps.getMovingObstacles()) {
            o.include = o.equals(includeBox);
        }
    }

    // Include all boxes
    private void includeAll() {
        for (Box b : ps.getMovingBoxes()) {
            b.include = true;
        }
        for (Box o : ps.getMovingObstacles()) {
            o.include = true;
        }
    }

    // It just resets the robot sampler really
    private void resetRobotSampler() {
        advanced = false;
        robotAttempt = 0;
        robotCollisionSet.clear();
        robot_samples.clear();
    }

    // If the same corners popup in the grid, they get deleted
    private boolean handleSameCorners(Vertex goal) {
        if (grid_corners.get(grid_corners.size() - 1).equals(goal)) {
            current = BOX_SAMPLE;
            return true;
        }

        while(grid_corners.get(grid_corners.size()-1).getPos().equals(
                grid_corners.get(grid_corners.size()-2).getPos())) {
            grid_corners.remove(grid_corners.size()-1);
            if (grid_corners.get(grid_corners.size() - 1).equals(goal)) {
                current = BOX_SAMPLE;
                return true;
            }
        }
        return false;
    }
}
