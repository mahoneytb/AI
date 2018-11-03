package MCTS;

import problem.*;
import simulator.*;

import java.util.ArrayList;
import java.util.Random;

public class MCTS {

    private ProblemSpec ps;
    private Node current_node;
    private Action current_action;
    private static int SELECTION = 0;
    private static int EXPANSION_SIMULATION = 1;
    private static int UPDATE = 2;
    private Random rand = new Random();
    private Simulate simulator;
    private int iterations = 0;
    private static double epsilon = 1e-6;

    // Initial state
    private int FSM;

    // Initialize array of methods calls to states
    interface ExecuteStateAction {
        void ExecuteState();
    }

    private ExecuteStateAction[] States = new ExecuteStateAction[] {
            new ExecuteStateAction() { public void ExecuteState() { SELECTION(); } },
            new ExecuteStateAction() { public void ExecuteState() { EXPANSION_SIMULATION(); } },
            new ExecuteStateAction() { public void ExecuteState() { UPDATE(); } },
    };

    private void ExecuteState(int index) {
        States[index].ExecuteState();
    }

    public MCTS(ProblemSpec ps) {
        this.ps = ps;
        current_node = new Node(1, false, false, ps.getFirstCarType(), ProblemSpec.FUEL_MAX,
                TirePressure.ONE_HUNDRED_PERCENT, ps.getFirstDriver(), ps.getFirstTireModel(), ps, 0);
        current_node.isStart = true;
        simulator = new Simulate(ps);
        FSM = SELECTION;
    }

    public MCTS(ProblemSpec ps, State state, int steps) {
        this.ps = ps;
        current_node = new Node(state.getPos(), state.isInSlipCondition(), state.isInBreakdownCondition(),
                state.getCarType(), state.getFuel(), state.getTirePressure(), state.getDriver(),
                state.getTireModel(), ps, steps);
        current_node.isStart = true;
        simulator = new Simulate(ps);
        FSM = SELECTION;
    }

    // Run the FSM
    public Action run() {
        while (iterations<10000) {
            ExecuteState(FSM);
        }
        return bestAction();
    }

    // Return the final action
    private Action bestAction() {
        Node best_child = current_node.getChildren().get(0);
        for(Node child : current_node.getChildren()) {
            if (child.getScore() > best_child.getScore()) {
                best_child = child;
            }
        }
        return best_child.getAction();
    }

    // Choose which state to expand from
    private void SELECTION() {
        // start from start node and put selection into current_node
        current_action = current_node.getNextUnexpanded();
        // if every action has been explored, pick the best scoring next
        while(current_action == null) {
            Node best_child = current_node.getChildren().get(0);
            for(Node child : current_node.getChildren()) {
                double s = best_child.getScore();
                if (child.getScore() > s | Double.isNaN(s)) {
                    best_child = child;
                }
            }
            current_node = best_child;
            current_action = current_node.getNextUnexpanded();
        }
        FSM = EXPANSION_SIMULATION;
    }

    // Choose an action to expand
    private void EXPANSION_SIMULATION() {
        Node nextNode = simulator.next(current_node, current_action);
        nextNode.addParent(current_node);
        current_node.addChild(nextNode);
        nextNode.setAction(current_action);
        nextNode.setStep(current_node.getStep()+1);

        // from here we do not save the computed nodes to save space
        int generations = 0;
        while (generations < 10) {
            current_action = nextNode.getNextUnexpanded();
            nextNode = simulator.next(nextNode, current_action);
            if (nextNode.getPos() == ps.getN()) {
                break;
            }
            generations++;
        }
        // Get the node back (we lost it above through the iterations)
        current_node = current_node.getChildren().get(current_node.getChildren().size()-1);
        // Its score is how much closer we got to the goal in the 10 steps
        current_node.newScore(Math.pow(ps.getDiscountFactor(), current_node.getStep()-1)
                                *(nextNode.getPos() - current_node.getParent().getPos())+rand.nextDouble()*epsilon);

        FSM = UPDATE;
    }

    // Update scores of states in tree
    private void UPDATE() {
        // Give this score to all the parents to backprop
        double score = current_node.getLastScore();
        while(!current_node.isStart) {
            current_node = current_node.getParent();
            current_node.newScore(score);
        }
        FSM = SELECTION;
        iterations++;
    }
}
