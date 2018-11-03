package problem;

import java.io.IOException;
import MCTS.*;

public class Main {

    public static void main(String[] args) {

        ProblemSpec ps;
        try {
            ps = new ProblemSpec("examples/level_3/input_lvl3.txt");
            System.out.println(ps.toString());
            MCTS MCTS = new MCTS(ps);
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
            System.exit(1);
        }
        System.out.println("Finished loading!");

    }
}
