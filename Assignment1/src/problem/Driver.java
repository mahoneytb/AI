package problem;

import java.io.IOException;
import tester.Tester;
import visualiser.Visualiser;

public class Driver {

    private Tester tester;

    public static void main(String[] args) {

        long startTime = System.nanoTime();
        ProblemSpec ps = new ProblemSpec();
        try {
            ps.loadProblem(args[0]);
        } catch (IOException e1) {
            System.out.println("FAILED: Invalid problem file");
            System.out.println(e1.getMessage());
        }

        FSM query = new FSM();
        query.loadProblem(ps);
        if (query.solve()) {
        }
        long endTime   = System.nanoTime();
        long totalTime = endTime - startTime;
        System.out.println("Runtime: " + totalTime/1000000000 + " seconds");
    }

    private void createTester(ProblemSpec ps) {
        tester = new Tester(ps);
    }
}
