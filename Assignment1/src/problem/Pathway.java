package problem;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.List;

public class Pathway {
    ArrayList<ArrayList<Double>> pathway = new ArrayList<ArrayList<Double>>();

    public void loadProblem(ProblemSpec ps) {
        addToPath(ps.getInitialRobotConfig(), ps.getMovingBoxes(), ps.getMovingObstacles());
    }

    public void addToPath(RobotConfig r, List<Box> boxes, List<Box> obstacles) {
        ArrayList<Double> move = new ArrayList<Double>();
        move.add(r.getPos().getX());
        move.add(r.getPos().getY());
        move.add(r.getOrientation());

        for (Box b : boxes) {
            // When the problem is loaded we store the bottom left corner, solution file requires the center
            move.add(b.getPos().getX() + b.getWidth()/2);
            move.add(b.getPos().getY() + b.getWidth()/2);
        }

        for (Box o : obstacles) {
            move.add(o.getPos().getX() + o.getWidth()/2);
            move.add(o.getPos().getY() + o.getWidth()/2);
        }

        pathway.add(move);
    }

    public void createSolutionFile() {
        try {
            PrintWriter writer = new PrintWriter("solution.txt", "UTF-8");
            writer.println(pathway.size());
            writer.flush();
            for (ArrayList<Double> line : pathway) {
                for (Double number : line) {
                    writer.print(number);
                    writer.print(' ');
                }
                writer.print('\n');
                writer.flush();
            }
        } catch (FileNotFoundException e) {
            // do nothing
        } catch (UnsupportedEncodingException e) {
            // do nothing
        }
    }
}