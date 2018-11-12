package MCTS;

import problem.*;
import simulator.*;

import java.util.HashMap;

public class MCTS {

    private ProblemSpec ps;
    private Node current_node;
    private Action current_action;
    private static int SETUP = 0;
    private static int SELECTION = 1;
    private static int EXPANSION_SIMULATION = 2;
    private static int UPDATE = 3;
    private Simulate simulator;
    private static double epsilon = 1e-6;
    private boolean setupComplete = false;
    private double scoreCarry;
    private double totalVisits = 0;

    // Initial state
    private int FSM;

    // Initialize array of methods calls to states
    interface ExecuteStateAction {
        void ExecuteState();
    }

    // State functions
    private ExecuteStateAction[] States = new ExecuteStateAction[] {
            new ExecuteStateAction() { public void ExecuteState() { SETUP(); } },
            new ExecuteStateAction() { public void ExecuteState() { SELECTION(); } },
            new ExecuteStateAction() { public void ExecuteState() { EXPANSION_SIMULATION(); } },
            new ExecuteStateAction() { public void ExecuteState() { UPDATE(); } },
    };

    private void ExecuteState(int index) {
        States[index].ExecuteState();
    }

    // MCTS start initial
    public MCTS(ProblemSpec ps, Simulate simulator) {
        this.ps = ps;
        current_node = new Node(1, false, false, ps.getFirstCarType(), ProblemSpec.FUEL_MAX,
                TirePressure.ONE_HUNDRED_PERCENT, ps.getFirstDriver(), ps.getFirstTireModel(), ps, 0);
        current_node.isStart = true;
        current_node.setAction(null);
        this.simulator = simulator;
        FSM = SETUP;
    }

    // MCTS continue initial
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
        // Run MCTS for 13 seconds
        while (current - start < 13*1e9) {
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

    // Populate the first action nodes
    private void SETUP() {
        current_action = current_node.getNextUnexpanded();
        // if every action has been explored, pick the best scoring next
        if(current_action == null) {
            for (Node child : current_node.getChildren()) {
                child.computeUCB1(epsilon, totalVisits);
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
                child.computeUCB1(0, totalVisits);
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
        Node nextNode = simNextNode(current_node, current_action);
        nextNode.addParent(current_node);
        current_node.addChild(nextNode);
        nextNode.setAction(current_action);

        // perform heavy rollout
        double win = rollout(nextNode);

        // Get the node back (we lost it above through the selection iterations)
        current_node = current_node.getChildren().get(current_node.getChildren().size()-1);
        // Its score is the max time allowed minus the time it took to win
        scoreCarry = win;
        // Update the current expanding node mean here
        current_node.newScore(scoreCarry);
        FSM = UPDATE;
    }

    // Update scores of states in tree
    private void UPDATE() {
        totalVisits++;
        // Give this score to all the parents to backprop
        if(setupComplete) {
            while(!current_node.isStart) {
                current_node.computeUCB1(0, totalVisits);
                current_node = current_node.getParent();
                current_node.newScore(scoreCarry);
            }
        }
        // Don't update if we haven't populated the first actions yet
        else {
            while (!current_node.isStart) {
                current_node = current_node.getParent();
                current_node.newScore(scoreCarry);
            }
        }
        FSM = (setupComplete) ? SELECTION : SETUP;
    }

    // Return the next node given current node and action
    private Node simNextNode(Node node, Action action) {
        if (action.getActionType() != ActionType.MOVE) { return simulator.next(node, action); }

        // If the action is a movement, return the most frequently occurring next state
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

    // Include new states in hmap, if not new increase count
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
            Action plusFuel = new Action(ActionType.CHANGE_TIRE_FUEL_PRESSURE, node.getTireModel(), 9,
                    node.getTirePressure());
            return plusFuel;//node.getAction();
        }
        return null;
    }

    // Return the child of node with the best score
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

    // Heavy playout
    private double rollout(Node nextNode) {
        double win = 0;
        Action action = nextNode.getAction();
        double steps = 0;

        // We only allow the occurance of one of each of the following actions during rollout
        boolean car = action.getActionType() == ActionType.CHANGE_CAR;
        boolean tire = action.getActionType() == ActionType.CHANGE_TIRES;
        boolean press = action.getActionType() == ActionType.CHANGE_PRESSURE;
        boolean driver = action.getActionType() == ActionType.CHANGE_DRIVER;

        while (win==0.0 && nextNode.getStep() < ps.getMaxT()) {
            current_action = nextNode.getNextUnexpanded();
            steps++;
            boolean isValid = false;
            // Check the validity of the action
            while (!isValid) {
                // Quit if movement
                if (current_action.getActionType() == ActionType.MOVE) {
                    break;
                }
                // If we've seen the action before in this rollout, try another action
                if ((car && (current_action.getActionType() == ActionType.CHANGE_CAR)) ||
                        tire && (current_action.getActionType() == ActionType.CHANGE_TIRES) ||
                        press && (current_action.getActionType() == ActionType.CHANGE_PRESSURE) ||
                        driver && (current_action.getActionType() == ActionType.CHANGE_DRIVER)) {
                    current_action = nextNode.getNextUnexpanded();
                } else if (current_action.getActionType() == ActionType.CHANGE_CAR) {
                    car = true;
                    isValid = true;
                } else if (current_action.getActionType() == ActionType.CHANGE_TIRES) {
                    tire = true;
                    isValid = true;
                } else if (current_action.getActionType() == ActionType.CHANGE_PRESSURE) {
                    press = true;
                    isValid = true;
                } else if (current_action.getActionType() == ActionType.CHANGE_DRIVER) {
                    driver = true;
                    isValid = true;
                } else {
                    // We should never get here
                    System.out.println("How tho");
                }
            }
            // Prepare for next rollout level
            nextNode = simNextNode(nextNode, current_action);
            nextNode.setAction(current_action);
            // Compute the score
            win = (1.0*nextNode.getPos() - 1.0*current_node.getPos())/steps;
            // If
            if (current_action.getActionType()==ActionType.MOVE) { return win; }
        }
        return win;
    }
}
