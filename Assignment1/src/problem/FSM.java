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
    private boolean goal_state = false;                 // Have we reached the final goal state?
    private int boxAttempt = 0;                         // Attempt number for box to goal mapping
    private int robotAttempt = 0;
    private int n_boxes;                                // Number of boxes
    private ProblemSpec ps;                             // Current problem spec
    private RobotConfig robot;                          // Current robot configuration
    private int current_box;                            // The current box
    private Tester tester;                              // Tester used for testing solutions
    private double current_grade;
    private ArrayList<RobotConfig> robot_samples;       // Robot config C-space samples
    private ArrayList<Vertex> course_grid;              // Course grid for box connecting
    private ArrayList<Vertex> fine_grid;                // Fine grid for box connecting
    private ArrayList<Vertex> current_grid;             // The currently used grid, including box and goal positions
    private ArrayList<Vertex> grid_corners;             // Mapped pathway for box to goal, including only corners and
                                                        // start position
    private ArrayList<RobotConfig> robot_map;           // List of robot configs from a start position to a goal position
    private Pathway complete_path = new Pathway();
    private int step = 0;
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
    private static double COURSE = 0.1;
    private static double FINE = 0.01;

    // Initial state
    private int current = START;

    // Load the problem spec
    public void loadProblem(ProblemSpec ps) {
        this.ps = ps;
        this.n_boxes = ps.getMovingBoxes().size();
        robot = ps.getInitialRobotConfig();
        constructGrids();
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
        return true;
    }

    //*********************************************   FSM   ************************************************************

    private void START() {
        // Reset all carried variables?
        current = BOX_SAMPLE;
    }

    // Sample C space for box to reach goal, ignore robot and movable boxes for now
    private void BOX_SAMPLE() {
        // Find closest box to robot not at goal yet
        current_box = findClosest(ps.getMovingBoxes(), robot.getPos());
        // First attempt is a course grid sample
        if (boxAttempt == 0) {
            current_grade = COURSE;
            current_grid = BoxToGrid(current_box, current_grade);
            current = BOX_CONNECT;
        }
        // Next attempt is a fine grid sample
        else if (boxAttempt == 1) {
            System.out.println("Using fine grid");
            current_grade = FINE;
            current_grid = BoxToGrid(current_box, current_grade);
            current = BOX_CONNECT;
        }
        else if (boxAttempt == 2) {
            System.out.println("Fine grid did not work, checking obstacles");
            current = OBSTACLE_CHECK;
        }
    }

    // Evaluate collisions and connect vertices
    private void BOX_CONNECT() {
        // Check if any vertices are collisions
        checkGridCollisions();
        for (Vertex v : current_grid) {
            if (v.collisionFree) {
                // Check if path between vertices is collision free
                checkVertexConnections(v);
            }
        }
        // Assert bi-directional connections
        populateNeighbours();
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
            System.out.println("Box grid mapped");
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
        RobotConfig goal = getRobotGoalPosition(grid_corners.get(grid_corners.size()-1));
        robot_samples = robotSample(robot, goal);
        current = ROBOT_MAP;
    }

    private void ROBOT_MAP() {
        A_Star_Robot map = new A_Star_Robot(robot, robot_samples);
        if (map.compute()) {
            robotAttempt = 0;
            robot_map = map.pathway;
            current = ADD_TO_PATH;
        } else {
            robotAttempt += 1;
            current = ROBOT_SAMPLE;
        }
    }

    private void ADD_TO_PATH() {
        moveRobot();
        moveBox();

        goal_state = true;
        for (int i = 0; i < ps.getMovingBoxes().size(); i++) {
            double bx = round(ps.getMovingBoxEndPositions().get(i).getX(), 6);
            double by = round(ps.getMovingBoxEndPositions().get(i).getY(), 6);
            Point2D bxy = new Point2D.Double(bx, by);
            if (!ps.getMovingBoxes().get(i).getPos().equals(bxy)) {
                goal_state = false;
            }
        }
        current = (grid_corners.size() > 1) ? ROBOT_SAMPLE : BOX_SAMPLE;
    }

    private void OBSTACLE_CHECK() {

    }

    // *************************************   Private functions   ****************************************************

    // Find closest box to robot
    private int findClosest(List<Box> boxes, Point2D pos) {
        double shortest = 10;
        int index = 0;
        // Check each box
        for (int i = 0; i < boxes.size(); i++) {
            Box b = boxes.get(i);
            //Check box is not already at goal
            double bx = round(ps.getMovingBoxEndPositions().get(i).getX(), 6);
            double by = round(ps.getMovingBoxEndPositions().get(i).getY(), 6);
            Point2D bxy = new Point2D.Double(bx, by);
            if (!b.pos.equals(bxy)) {
                double d = pos.distance(bxy);
                // Update closest if distance is shorter
                if (d < shortest) {
                    index = i;
                    shortest = d;
                }
            }
        }
        return index;
    }

    // Construct the C space box position course grids
    private void constructGrids() {
        course_grid = new ArrayList<>();
        fine_grid = new ArrayList<>();
        boolean iCourse = true;
        boolean jCourse;
        for (double i = 0.05; i<1; i=round(i+0.05, 2)) {
            iCourse = !iCourse;
            jCourse = true;
            for (double j = 0.05; j<1; j=round(j+0.05, 2)) {
                jCourse = !jCourse;
                // 19x19 grid
                fine_grid.add(new Vertex(new Point2D.Double(i,j)));
                if (iCourse && jCourse) {
                    // 9x9 grid
                    course_grid.add(new Vertex(new Point2D.Double(i,j)));
                }
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
    private ArrayList<Vertex> BoxToGrid(int b, double grade) {
        double xb = ps.getMovingBoxes().get(b).getPos().getX();
        double yb = ps.getMovingBoxes().get(b).getPos().getY();
        double xg = ps.getMovingBoxEndPositions().get(b).getX();
        double yg = ps.getMovingBoxEndPositions().get(b).getY();

        ArrayList<Vertex> newGrid;
        if (grade == COURSE) { newGrid = course_grid; }
        else { newGrid = fine_grid; }

        for (double i=grade; i < 1; i=round(i+grade, 2)) {
            Vertex v1 = new Vertex(new Point2D.Double(i, yb));
            findNeighbours(v1, newGrid, grade);
            newGrid.add(v1);

            Vertex v2 = new Vertex(new Point2D.Double(xb, i));
            findNeighbours(v2, newGrid, grade);
            newGrid.add(v1);

            Vertex v3 = new Vertex(new Point2D.Double(i, yg));
            findNeighbours(v3, newGrid, grade);
            newGrid.add(v3);

            Vertex v4 = new Vertex(new Point2D.Double(xg, i));
            findNeighbours(v4, newGrid, grade);
            newGrid.add(v4);
        }
        Vertex vb = new Vertex(new Point2D.Double(xb, yb));
        findNeighbours(vb, newGrid, grade);
        newGrid.add(vb);

        Vertex vg = new Vertex(new Point2D.Double(xg, yg));
        findNeighbours(vg, newGrid, grade);
        newGrid.add(vg);

        return newGrid;
    }

    // Find neighbours for newly added vertices
    private void findNeighbours(Vertex v, ArrayList<Vertex> grid, double d) {
        for (Vertex p : grid) {
            // Must be within distance
            if (p.getPos().distance(v.getPos()) < d) {
                // Must be new neighbour
                if (!p.getNeighbours().contains(v)) {
                    // Must not be diagonal neighbour
                    if (p.getPos().getX() == v.getPos().getX() ||
                            p.getPos().getY() == v.getPos().getY()) {
                        v.addNeighbour(p);
                    }
                }
            }
        }
    }

    // Check collision for a single vertex
    private void checkVertexCollisions(Vertex v) {
        double width = ps.getMovingBoxes().get(current_box).getWidth();
        // Check collisions of each vertex
        List<Box> newBoxConfig = new ArrayList<>();
        newBoxConfig.addAll(ps.getMovingBoxes());
        // Update box position to grid
        Box newPos = new MovingBox(v.getPos(), width);
        newBoxConfig.set(current_box, newPos);
        // Include movable obstacles
        newBoxConfig.addAll(ps.getMovingObstacles());
        if (!tester.hasCollision(robot, newBoxConfig)) {
            v.collisionFree = false;
        }
    }

    private void checkGridCollisions() {
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
            if (!tester.hasCollision(robot, newBoxConfig)) {
                v.collisionFree = false;
            }
        }
    }

    // Check the connectivity between neighbours in box grid, uses BFS
    private void checkVertexConnections(Vertex v) {
        // Check each neighbour, result is stored in Vertex class map
        for (Vertex initial : v.getNeighbours()) {
            checkVertexCollisions(initial);

            // Initial neighbour has collision
            if (!initial.collisionFree) {
                v.setConnect(initial, false);
            } else {
                initial.addNeighbour(v);

                Queue<Vertex> queue = new LinkedList<>();
                queue.add(initial);

                // BFS of splits along edge between vertices, no need to check if explored as each child is unique
                while (!queue.isEmpty()) {
                    Vertex current = queue.remove();
                    // Found collision along connection
                    if (!current.collisionFree) {
                        v.setConnect(initial, false);
                        break;
                    } else {
                        ArrayList<Vertex> children = split(current);
                        if (children != null) {
                            queue.addAll(children);
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
            if (n.getPos().distance(v.getPos())<0.001) { return children; };
            Point2D mid = new Point2D.Double((n.getPos().getX() + v.getPos().getX()) / 2,
                    (n.getPos().getY() + v.getPos().getY()) / 2);
            Vertex midV = new Vertex(mid);
            midV.addNeighbour(n);
            midV.addNeighbour(v);
            children.add(midV);
        }
        return children;
    }

    // Return split robot configs with neighbours of either side ****Implement actual tester checks to see if valid moves!!
    private ArrayList<RobotConfig> split(RobotConfig r) {
        ArrayList<RobotConfig> children = new ArrayList<>();
        for (RobotConfig n : r.neighbours) {

            //if (tester.isValidStep(r, n)) {
            if ((r.getPos().distance(n.getPos()) < 0.0001) &&
                    (Math.abs(r.getOrientation()-n.getOrientation()) < 0.0001)) {
                return null;
            }

            Point2D mid = new Point2D.Double((n.getPos().getX() + r.getPos().getX()) / 2,
                    (n.getPos().getY() + r.getPos().getY()) / 2);
            double alpha = (r.opposite_rotation) ? Math.PI + (n.getOrientation() + r.getOrientation()) / 2 :
                                                    (n.getOrientation() + r.getOrientation()) / 2;
            alpha -= (alpha > (2*Math.PI)) ? 2*Math.PI : 0;
            RobotConfig child = new RobotConfig(mid, alpha);
            child.neighbours.add(n);
            child.neighbours.add(r);
            children.add(child);
        }
        return children;
    }

    /* Until this point, neighbours have not been bi-directional in order to reduce double handling. This function
     * fully populates the neighbour network of the grid for collision free connections prior to pathfinding.
     */
    private void populateNeighbours() {
        for (Vertex v : current_grid) {
            if (v.collisionFree) {
                for (Vertex n : v.getNeighbours()) {
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

    // Round double to d decimal places
    private double round(double num, int d) {
        num = num * Math.pow(10, d);
        num = Math.round(num);
        num = num / Math.pow(10, d);
        return num;
    }

    // Get the robot push position for the box when at vertex v
    private RobotConfig getRobotGoalPosition(Vertex v) {
        char d = v.direction;
        double gx = 0, gy = 0, galpha = 0;
        Box b = ps.getMovingBoxes().get(current_box);
        switch (d) {
            case 'l':
                gx = v.getPos().getX() + b.getWidth();
                gy = round(v.getPos().getY() + b.getWidth()/2, 6);
                galpha = round(Math.PI / 2, 6);
                break;
            case 'r':
                gx = v.getPos().getX();
                gy = round(v.getPos().getY() + b.getWidth()/2, 6);
                galpha = round(Math.PI / 2, 6);
                break;
            case 'u':
                gx = round(v.getPos().getX() + b.getWidth()/2, 6);
                gy = v.getPos().getY();
                galpha = 0;
                break;
            case 'd':
                gx = round(v.getPos().getX() + b.getWidth()/2, 6);
                gy = v.getPos().getY() + b.getWidth();
                galpha = 0;
                break;
        }
        RobotConfig goal = new RobotConfig(new Point2D.Double(gx, gy), galpha);
        return goal;
    }

    // Return 100 tested and connected robotConfigs, mostly between r and g
    private ArrayList<RobotConfig> robotSample(RobotConfig r, RobotConfig g) {
        ArrayList<RobotConfig> samples = new ArrayList<>();

        // Check direct start to goal connection
        samples.add(r);
        checkRobotConnections(g, samples);
        if (g.neighbours.contains(r)) {
            samples.add(g);
            return samples;
        }

        RobotConfig g2 = createOpposite(g);
        checkRobotConnections(g2, samples);
        if (g.neighbours.contains(r)) {
            samples.add(g2);
            return samples;
        }

        samples.add(g);
        samples.add(g2);

        addBasicMoves(samples);
        if (robotAttempt == 0) {
            return samples;
        }

        // Get bounds for probable samples: 0.1 outside of current max and min positions
        double BOUND = 0.1;
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

        List<Box> newBoxConfig = ps.getMovingBoxes();
        newBoxConfig.addAll(ps.getMovingObstacles());

        // Sample 100 times with 70% chance of forcing being between the robot current position and the goal
        int n = 0;
        while (n<100) {
            double x, y, alpha;
            // 70% we choose a point in the start to goal area
            if (generator.nextDouble() > 0.3) {
                x = generator.nextDouble()*(xmax - xmin) + xmin;
                y = generator.nextDouble()*(ymax - ymin) + ymin;
                alpha = generator.nextDouble()*(alphamax - alphamin) + alphamin;
                // Introduce opposite robot orientations
                alpha +=  (generator.nextDouble() > 0.5) ? Math.PI : 0;
                alpha -= (alpha > (Math.PI * 2)) ? Math.PI * 2 : 0;
            } else {
                x = generator.nextDouble();
                y = generator.nextDouble();
                alpha = generator.nextDouble();
            }
            //alpha = generator.nextDouble()*Math.PI*2; // Keep alpha random
            RobotConfig s = new RobotConfig(new Point2D.Double(x, y), alpha);

            // Check for collisions
            if (!tester.hasCollision(s, newBoxConfig)) {
                continue;
            }

            // Check for connections
            checkRobotConnections(s, samples);
            n++;
            RobotConfig s2 = createOpposite(s);
            checkRobotConnections(s2, samples);
            n++;
            samples.add(s);
            samples.add(s2);

            // Early finish if the current sample bridges the start and goal positions
            if ((s.neighbours.contains(r) && (s.neighbours.contains(g) || s.neighbours.contains(g2))) ||
                    (s2.neighbours.contains(r) && (s2.neighbours.contains(g) || s2.neighbours.contains(g2)))) {
                samples.clear();
                samples.add(r);
                samples.add(g);
                samples.add(g2);
                samples.add(s);
                samples.add(s2);
                return samples;
            }
        }
        return samples;
    }
    // Adds basic left down right up movements from start and goal to sample space
    private void addBasicMoves(ArrayList<RobotConfig> samples) {
        double STEP = 0.01;
        ArrayList<RobotConfig> newMoves = new ArrayList<>();

        for (RobotConfig r : samples) {
            newMoves.add(new RobotConfig(new Point2D.Double(r.getPos().getX() - STEP,
                    r.getPos().getY()), r.getOrientation()));
            newMoves.add(new RobotConfig(new Point2D.Double(r.getPos().getX() + STEP,
                    r.getPos().getY()), r.getOrientation()));
            newMoves.add(new RobotConfig(new Point2D.Double(r.getPos().getX(),
                    r.getPos().getY() + STEP), r.getOrientation()));
            newMoves.add(new RobotConfig(new Point2D.Double(r.getPos().getX(),
                    r.getPos().getY() + STEP), r.getOrientation()));
        }

        samples.addAll(newMoves);

        // Mid moves
        RobotConfig r = samples.get(0);
        RobotConfig g = samples.get(1);
        RobotConfig g2 = samples.get(2);
        RobotConfig mid1 = new RobotConfig(new Point2D.Double(r.getPos().getX(), g.getPos().getY()), r.getOrientation());
        RobotConfig mid2 = new RobotConfig(new Point2D.Double(r.getPos().getX(), g.getPos().getY()), g.getOrientation());
        RobotConfig mid3 = new RobotConfig(new Point2D.Double(g.getPos().getX(), r.getPos().getY()), r.getOrientation());
        RobotConfig mid4 = new RobotConfig(new Point2D.Double(g.getPos().getX(), r.getPos().getY()), g.getOrientation());
        RobotConfig mid5 = new RobotConfig(new Point2D.Double(r.getPos().getX(), g.getPos().getY()), g2.getOrientation());
        RobotConfig mid6 = new RobotConfig(new Point2D.Double(g.getPos().getX(), r.getPos().getY()), g2.getOrientation());

        samples.add(mid1);
        samples.add(mid2);
        samples.add(mid3);
        samples.add(mid4);
        samples.add(mid5);
        samples.add(mid6);

        List<Box> newBoxConfig = ps.getMovingBoxes();
        newBoxConfig.addAll(ps.getMovingObstacles());

        ArrayList<RobotConfig> found = new ArrayList<>();
        for (RobotConfig s : samples) {
            if (!tester.hasCollision(s, newBoxConfig)) {
                found.add(s);
            }
        }
        samples.removeAll(found);
        for (RobotConfig s : samples) {
            // Check for connections
            checkRobotConnections(s, samples);
        }
    }

    // Check the connectivity between neighbours in robot C space, uses BFS
    /*
    private void checkRobotConnections(RobotConfig r, ArrayList<RobotConfig> samples) {
        // Check each neighbour, result is stored in RobotConfig class map
        List<Box> newBoxConfig = ps.getMovingBoxes();
        newBoxConfig.addAll(ps.getMovingObstacles());

        for (RobotConfig n : samples) {
            // Check direct start goal connection, or any connection less than 0.2 apart
            if ((samples.size() > 1) && (r.getPos().distance(n.getPos()) > 0.2) || r.equals(n)) {
                continue;
            }
            Queue<RobotConfig> queue = new LinkedList<>();
            RobotConfig copy = new RobotConfig(n.getPos(), n.getOrientation());
            copy.neighbours.add(r);
            ArrayList<RobotConfig> initial = split(copy);
            if (initial == null) {continue;}
            queue.addAll(initial);
            boolean collision_found = false;

            // BFS of splits along edge between vertices, no need to check if explored as each child is unique
            while (!queue.isEmpty() && !collision_found) {
                RobotConfig current = queue.remove();
                // Found collision along connection
                if ((current.getPos().getX() != 0.1246) && (current.getPos().getX() > 0.12446) &&
                        (current.getPos().getY() < 0.16612) && (current.getPos().getY() != 0.165) &&
                        (current.getOrientation() != 1.504) && (current.getOrientation() > 1.5)) {
                            System.out.println("here\n");
                }
                if (!tester.hasCollision(current, newBoxConfig)) {
                    collision_found = true;
                    continue;
                }
                ArrayList<RobotConfig> children = split(current);
                if (children != null) {
                    for (RobotConfig child : children) {
                        if (!tester.hasCollision(child, newBoxConfig)) {
                            collision_found = true;
                            continue;
                        }
                    }
                    queue.addAll(children);
                }
            }

            if (!collision_found) {
                r.neighbours.add(n);
                n.neighbours.add(r);
            }
        }
    }
*/

    private void checkRobotConnections(RobotConfig r, ArrayList<RobotConfig> samples) {
        List<Box> newBoxConfig = ps.getMovingBoxes();
        newBoxConfig.addAll(ps.getMovingObstacles());

        for (RobotConfig s : samples) {
            if (s.neighbours.contains(r) || s.equals(r)) {
                continue;
            }

            // Set up initial guess at valid step size (includes sign yay)
            int FACTOR = 10;
            double x_dist = (s.getPos().getX() - r.getPos().getX());
            double y_dist = (s.getPos().getY() - r.getPos().getY());
            double alpha_dist = (s.getOrientation() - r.getOrientation());

            if ((Math.abs(x_dist) > 0.1*robotAttempt) || (Math.abs(y_dist) > 0.1*robotAttempt)) {
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
                if (!tester.hasCollision(current, newBoxConfig)) {
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
    private void moveRobot() {

        List<Box> boxes = ps.getMovingBoxes();
        List<Box> obstacles = ps.getMovingObstacles();

        for (int i = robot_map.size()-1; i > 0; i--) {
            RobotConfig start = robot_map.get(i);
            RobotConfig end = robot_map.get(i-1);

            // Set up initial guess at valid step size (includes sign yay)
            int FACTOR = 10;
            double x_dist = (start.getPos().getX() - end.getPos().getX());
            double y_dist = (start.getPos().getY() - end.getPos().getY());
            double alpha_dist = (start.getOrientation() - end.getOrientation());

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
        }
    }

    // Add robot and box positions along mapped path to the complete pathway
    private void moveBox() {

        List<Box> boxes = ps.getMovingBoxes();
        List<Box> obstacles = ps.getMovingObstacles();

        // Get robot start position
        RobotConfig start = getRobotGoalPosition(grid_corners.get(grid_corners.size()-1));

        // Compute robot end position based on chang in box position
        double x_shift = grid_corners.get(grid_corners.size()-1).getPos().getX() -
                grid_corners.get(grid_corners.size()-2).getPos().getX();
        double y_shift = grid_corners.get(grid_corners.size()-1).getPos().getY() -
                grid_corners.get(grid_corners.size()-2).getPos().getY();
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
        Box current_box = boxes.get(this.current_box);
        int count = 1;
        while (!tester.isValidStep(current_robot, end)) {
            current_robot.pos = new Point2D.Double(round(current_robot.getPos().getX() - count*x_dist/FACTOR, 6),
                    round(current_robot.getPos().getY() - count*y_dist/FACTOR, 6));
            current_box.pos = new Point2D.Double(round(current_box.getPos().getX() - count*x_dist/FACTOR, 6),
                    round(current_box.getPos().getY() - count*y_dist/FACTOR, 6));
           complete_path.addToPath(current_robot, boxes, obstacles);
        }
        robot = end;
        current_box.pos = new Point2D.Double(round(grid_corners.get(grid_corners.size()-2).getPos().getX(), 6),
                round(grid_corners.get(grid_corners.size()-2).getPos().getY(), 6));
        current_box.setRect();
        // Add the goal position
        complete_path.addToPath(robot, boxes, obstacles);
        // Delete last element of grid corners
        grid_corners.remove(grid_corners.size()-1);
    }

    private RobotConfig createOpposite(RobotConfig r) {
        double counterAlpha = r.getOrientation() + Math.PI;
        counterAlpha -= (counterAlpha > (2*Math.PI)) ? 2*Math.PI : 0;
        RobotConfig r2 = new RobotConfig(r.getPos(), counterAlpha);
        //r2.opposite_rotation = true;
        return r2;
    }
}
