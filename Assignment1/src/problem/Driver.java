package problem;

import java.io.IOException;
import tester.Tester;
import visualiser.Visualiser;

public class Driver {

    public static void main(String[] args) {

        long startTime = System.nanoTime();
        ProblemSpec ps = new ProblemSpec();
        if (args.length < 2) {
            System.out.println("Not enough input arguments");
            return;
        }
        try {
            ps.loadProblem(args[0]);
        } catch (IOException e1) {
            System.out.println("FAILED: Invalid problem file");
            System.out.println(e1.getMessage());
        }

        FSM query = new FSM();
        query.loadProblem(ps);
        if (query.solve(args[1])) {
        }
        long endTime   = System.nanoTime();
        long totalTime = endTime - startTime;
        System.out.println("Runtime: " + totalTime/1000000000 + " seconds");
        Tester tester = new Tester(ps);
        try {
            ps.loadSolution("solution.txt");
            tester.testSolution();

        } catch (IOException e1) {
            System.out.println("FAILED: Invalid problem file");
            System.out.println(e1.getMessage());
        }
    }
}
