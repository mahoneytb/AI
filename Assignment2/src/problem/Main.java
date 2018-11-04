package problem;

import java.io.IOException;
import java.util.ArrayList;

import MCTS.*;
import simulator.*;

public class Main {



    public static void main(String[] args) {

        ProblemSpec ps;
        try {
            double wins = 0;
            int steps = 0;
            //for (int i=0; i<100; i++) {
                //String filename = "examples/level_5/input_lvl5_" + i + ".txt";
            String filename = "examples/level_3/input_lvl3_" + 0 + ".txt";
            ps = new ProblemSpec(filename);
            Simulator simulator = new Simulator(filename, "lvl5Out");
            Simulate testSimulator = new Simulate(ps);
            System.out.println(ps.toString());
            MCTS MCTS = new MCTS(ps, testSimulator);


            while (simulator.getSteps() <= ps.getMaxT()) {
                Action action = MCTS.run();
                State nextState = simulator.step(action);
                MCTS = new MCTS(ps, testSimulator, nextState, simulator.getSteps(), action);
                if (nextState.getPos() >= ps.getN()) {
                    wins++;
                    steps += simulator.getSteps();
                    break;
                }
            }
            //}
            System.out.println(wins + " out of 100");
            System.out.println(steps/wins + " steps on average");
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
            System.exit(1);
        }
        System.out.println("Finished loading!");
    }
}
