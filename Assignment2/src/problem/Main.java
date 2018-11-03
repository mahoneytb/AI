package problem;

import java.io.IOException;
import MCTS.*;
import simulator.*;

public class Main {



    public static void main(String[] args) {

        ProblemSpec ps;
        try {
            ps = new ProblemSpec("examples/level_3/input_lvl3.txt");
            Simulator simulator = new Simulator("examples/level_3/input_lvl3.txt", "lvl3Out");
            System.out.println(ps.toString());
            MCTS MCTS = new MCTS(ps);

            while (true) {
                Action action = MCTS.run();
                State nextState = simulator.step(action);
                MCTS = new MCTS(ps, nextState, simulator.getSteps());
            }
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
            System.exit(1);
        }
        System.out.println("Finished loading!");
    }
}
