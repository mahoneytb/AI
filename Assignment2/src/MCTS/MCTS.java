package MCTS;

import problem.*;
import simulator.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Random;

public class MCTS {

    private ProblemSpec ps;
    private Node current_node;
    private Action current_action;
    private static int SETUP = 0;
    private static int SELECTION = 1;
    private static int EXPANSION_SIMULATION = 2;
    private static int UPDATE = 3;
    private Random rand = new Random();
    private Simulate simulator;
    private static double epsilon = 1e-6;
    private boolean setupComplete = false;
    private double scoreCarry;

    // Initial state
    private int FSM;

    // Initialize array of methods calls to states
    interface ExecuteStateAction {
        void ExecuteState();
    }

    private ExecuteStateAction[] States = new ExecuteStateAction[] {
            new ExecuteStateAction() { public void ExecuteState() { SETUP(); } },
            new ExecuteStateAction() { public void ExecuteState() { SELECTION(); } },
            new ExecuteStateAction() { public void ExecuteState() { EXPANSION_SIMULATION(); } },
            new ExecuteStateAction() { public void ExecuteState() { UPDATE(); } },
    };

    private void ExecuteState(int index) {
        States[index].ExecuteState();
    }

    public MCTS(ProblemSpec ps, Simulate simulator) {
        this.ps = ps;
        current_node = new Node(1, false, false, ps.getFirstCarType(), ProblemSpec.FUEL_MAX,
                TirePressure.ONE_HUNDRED_PERCENT, ps.getFirstDriver(), ps.getFirstTireModel(), ps, 0);
        current_node.isStart = true;
        current_node.setAction(null);
        this.simulator = simulator;
        FSM = SETUP;
    }

    public MCTS(ProblemSpec ps, Simulate simulator, State state, int steps, Action topAction) {
        this.ps = ps;
        current_node = new Node(state.getPos(), state.isInSlipCondition(), state.isInBreakdownCondition(),
                state.getCarType(), state.getFuel(), state.getTirePressure(), state.getDriver(),
                state.getTireModel(), ps, steps);
        current_node.isStart = true;
        current_node.setAction(topAction);
        this.simulator = simulator;
        FSM = SETUP;
    }

    // Run the FSM
    public Action run() {
        // Iterate for 14 seconds
        long start = System.nanoTime();
        long current = System.nanoTime();
        while (current - start < 13*1e9) { //13
            ExecuteState(FSM);
            if (FSM == SELECTION) { current = System.nanoTime(); }
        }
        return bestAction();
    }

    // Return the final action
    private Action bestAction() {
        Node best_child = current_node.getChildren().get(0);
        for(Node child : current_node.getChildren()) {
            if (child.getMean() > best_child.getMean()) {
                best_child = child;
            }
        }
        Action best_action = comboAction(best_child);
        return best_action;
    }

    private void SETUP() {
        current_action = current_node.getNextUnexpanded();
        // if every action has been explored, pick the best scoring next
        if(current_action == null) {
            for (Node child : current_node.getChildren()) {
                child.computeUCB1(epsilon);
            }
            setupComplete = true;
            FSM = SELECTION;
            return;
        }
        FSM = EXPANSION_SIMULATION;
    }

    // Choose which state to expand from
    private void SELECTION() {
        // start from start node and put selection into current_node
        current_action = current_node.getNextUnexpanded();
        // if every action has been explored, pick the best scoring next
        while(current_action == null) {
            Node best_child = current_node.getChildren().get(0);
            for(Node child : current_node.getChildren()) {
                child.computeUCB1(0);
                double s = best_child.getScore();
                if (child.getScore() > s | Double.isNaN(s)) {
                    best_child = child;
                }
            }
            current_node = best_child;
            current_action = current_node.getNextUnexpanded();
        }
        //current_node = bestUCT();
        //current_action = current_node.getNextUnexpanded();
        FSM = EXPANSION_SIMULATION;
    }

    // Choose an action to expand
    private void EXPANSION_SIMULATION() {
        Node nextNode = simNextNode(current_node, current_action);
        nextNode.addParent(current_node);
        current_node.addChild(nextNode);
        nextNode.setAction(current_action);

        // from here we do not save the computed nodes to save space
        boolean win = false;
        while (!win & nextNode.getStep() < ps.getMaxT()) {
            current_action = nextNode.getNextUnexpanded();
            nextNode = simNextNode(nextNode, current_action);
            nextNode.setAction(current_action);
            win = nextNode.getPos() >= ps.getN();// (current_node.getPos() + ps.getN());
        }
        // Get the node back (we lost it above through the iterations)
        current_node = current_node.getChildren().get(current_node.getChildren().size()-1);
        // Its score is the max time allowed minus the time it too to win
        scoreCarry = (win) ? 1 : 0;
        // Update the current expanding node mean here
        current_node.newScore(scoreCarry);
        FSM = UPDATE;
    }

    // Update scores of states in tree
    private void UPDATE() {
        // Give this score to all the parents to backprop
        if(setupComplete) {
            while(!current_node.isStart) {
                current_node.computeUCB1(0);
                current_node = current_node.getParent();
                current_node.newScore(scoreCarry);
            }
        }
        else {
            while (!current_node.isStart) {
                current_node = current_node.getParent();
                current_node.newScore(scoreCarry);
            }
        }
        FSM = (setupComplete) ? SELECTION : SETUP;
    }

    private Node simNextNode(Node node, Action action) {
        if (action.getActionType() != ActionType.MOVE) { return simulator.next(node, action); }

        HashMap<Node, Integer> hmap = new HashMap<Node, Integer>();
        for (int i=0; i<20; i++) {
            Node sample = simulator.next(node, action);
            updateHash(hmap, sample);
        }

        Node mostNode = null;
        int mostCount = 0;
        for (Node n : hmap.keySet()) {
            if (hmap.get(n) > mostCount) {
                mostCount = hmap.get(n);
                mostNode = n;
            }
        }
        return mostNode;
    }

    private void updateHash(HashMap<Node, Integer> hmap, Node node) {
        for (Node n : hmap.keySet()) {
            if (n.getPos() == node.getPos() &
                n.isInBreakdownCondition() == node.isInBreakdownCondition() &
                n.isInSlipCondition() == node.isInSlipCondition() &
                n.getStep() == node.getStep()) {
                hmap.put(n, hmap.get(n) + 1);
                return;
            }
        }
        hmap.put(node, 1);
    }

    // Lets consider using combo actions (A7 and A8)
    private Action comboAction(Node node) {
        ActionType current = node.getAction().getActionType();
        // If its move there is no combo
        if (current == ActionType.MOVE | ps.getLevel().getLevelNumber() < 4) { return node.getAction(); }

        // If its car or driver, we can pair with a subsequent car or driver in the tree (A7)
        if (current == ActionType.CHANGE_CAR | current == ActionType.CHANGE_DRIVER) {
            Node child = node.copyNode();
            while (!child.getChildren().isEmpty()) {
                ActionType childAction = child.getAction().getActionType();
                // if the later action is move don't use combo
                if (childAction == ActionType.MOVE) { return node.getAction(); }
                // if the later move is pairable (ie. 2 and 3), use combo
                else if ((childAction == ActionType.CHANGE_CAR | childAction == ActionType.CHANGE_DRIVER) &
                            childAction != current) {
                    String c = (childAction == ActionType.CHANGE_CAR) ? child.getAction().getCarType() :
                            node.getAction().getCarType();
                    String d = (childAction == ActionType.CHANGE_DRIVER) ? child.getAction().getDriverType() :
                            node.getAction().getDriverType();
                    Action best = new Action(ActionType.CHANGE_CAR_AND_DRIVER, c, d);
                    return best;
                }
                child = bestChild(child);
            }
            return node.getAction();
        }

        if (ps.getLevel().getLevelNumber() != 5) { return node.getAction(); }

        // If A4 to A6, always refuel 9, search for subsequent pairings of 4 and 5 in tree (A8)
        if (current == ActionType.CHANGE_TIRES | current == ActionType.CHANGE_PRESSURE) {
            Node child = node.copyNode();
            while (!child.getChildren().isEmpty()) {
                ActionType childAction = child.getAction().getActionType();
                // if the later action is move might as well add some fuel
                if (childAction == ActionType.MOVE) {
                    Action plusFuel = new Action(ActionType.CHANGE_TIRE_FUEL_PRESSURE, node.getTireModel(), 9,
                            node.getTirePressure());
                    return plusFuel;
                }
                // if the later move is pairable (ie. 4 and 6), use combo and add minimum fuel
                else if ((childAction == ActionType.CHANGE_TIRES | childAction == ActionType.CHANGE_PRESSURE) &
                        childAction != current) {
                    Tire t = (childAction == ActionType.CHANGE_TIRES) ? child.getAction().getTireModel() :
                            node.getAction().getTireModel();
                    TirePressure p = (childAction == ActionType.CHANGE_PRESSURE) ? child.getAction().getTirePressure() :
                            node.getAction().getTirePressure();
                    Action best = new Action(ActionType.CHANGE_TIRE_FUEL_PRESSURE, t, 9, p);
                    return best;
                }
                child = bestChild(child);
            }
            return node.getAction();
        }
        return null;
    }

    private Node bestChild(Node node) {
        Node bestChild = node.getChildren().get(0);
        double bestScore = 0;
        for (Node c : node.getChildren()) {
            if (c.getMean() > bestScore) {
                bestScore = c.getMean();
                bestChild = c;
            }
        }
        return bestChild;
    }
}
