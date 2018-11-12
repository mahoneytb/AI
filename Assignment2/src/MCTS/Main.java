package MCTS;

import java.io.IOException;

import problem.*;
import simulator.*;

public class Main {

    public static void main(String[] args) {

        ProblemSpec ps;
        try {
            String filename = args[0];
            ps = new ProblemSpec(filename);
            Simulator simulator = new Simulator(filename, args[1]);
            Simulate testSimulator = new Simulate(ps);
            System.out.println(ps.toString());
            MCTS MCTS = new MCTS(ps, testSimulator);

            int counter = 0;
            while (simulator.getSteps() <= ps.getMaxT()) {
                Action action = MCTS.run();
                // force action every 4 without
                if (action.getActionType() == ActionType.MOVE) {
                    counter = 0;
                } else {

                    counter++;
                }
                if (counter > 3) {
                    action = new Action(ActionType.MOVE);
                    counter = 0;
                }

                State nextState = simulator.step(action);
                MCTS = new MCTS(ps, testSimulator, nextState, simulator.getSteps(), action);
                if (nextState.getPos() >= ps.getN()) {
                    break;
                }
            }
        } catch (IOException e) {
            System.out.println("IO Exception occurred");
            System.exit(1);
        }
        System.out.println("Finished loading!");
    }
}
