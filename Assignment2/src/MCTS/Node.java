package MCTS;

import problem.*;

import java.util.ArrayList;
import java.util.Random;

public class Node {

    private Node parent;
    private ArrayList<Node> children = new ArrayList();
    private ProblemSpec state;
    private boolean explored = false;
    private ArrayList<Double> scores;
    public boolean isStart = false;
    private Random rand = new Random();
    private ActionSpace actionSpace;
    private double explore_param = Math.sqrt(2);
    private ProblemSpec ps;
    private int step;
    private double UCB1;
    private Action action;

    /** The position of the car in terms of grid cell in environment **/
    private int pos;
    /** Whether the car is in a slip condition **/
    private boolean slip;
    /** Whether the car is broken down **/
    private boolean breakdown;
    /** The car type **/
    private String carType;
    /** Fuel remaining **/
    private int fuel;
    /** Tire pressure **/
    private TirePressure tirePressure;
    /** The driver **/
    private String driver;
    /** The tire model **/
    private Tire tireModel;

    public ArrayList<Node> getChildren() { return children; }

    public void setAction(Action action) {
        this.action = action;
    }

    public void addParent(Node parent) {
        this.parent = parent;
    }

    public Node getParent() { return this.parent; }

    public void addChild(Node child) {
        this.children.add(child);
    }

    public void setExplored() {
        this.explored = true;
    }

    public Action getAction() {
        return this.action;
    }

    public double getLastScore() { return scores.get(scores.size()-1); }

    // UCB1 function
    public double getScore() {
        return UCB1;
    }

    // Calculate the explore/exploit score. Handle the case where start parent does not have many tries.
    private void computeUCB1() {
        if (parent.getVisits() > 1) {
            UCB1 = mean(this.scores) + explore_param * Math.sqrt(Math.log(parent.getVisits()) / this.getVisits());
        } else {
            UCB1 = mean(this.scores) + explore_param * Math.sqrt(Math.log(2) / this.getVisits());
        }
    }

    public void newScore(double score) {
        this.scores.add(score);
        if(!isStart) { computeUCB1(); }
    }

    public int getVisits() {
        return scores.size();
    }

    public Action getNextUnexpanded() {
        return actionSpace.getNextUnexploredAction();
    }

    private double mean(ArrayList<Double> n_list) {
        int sum = 0;
        for (double n : n_list) {
            sum += n;
        }
        return sum/n_list.size();
    }

    public void setStep(int step) { this.step = step; }
    public int getStep() { return step; }


    //*****************************************from State for Simulate***********************************************

    /**
     * Construct a new state with the given parameter values
     *
     * @param pos cell index of car in environment
     * @param slip if car is in slip condition
     * @param breakdown if car is in breakdown condition
     * @param carType the type of car
     * @param fuel the amount of fuel
     *        (assumes ProblemSpec.FUEL_MIN <= fuel <= ProblemSpec.FUEL_MAX)
     * @param tirePressure the pressure of tires
     * @param driver the driver
     * @param tireModel the model of the tires
     */
    public Node(int pos, boolean slip, boolean breakdown, String carType,
                 int fuel, TirePressure tirePressure, String driver, Tire tireModel, ProblemSpec ps, int step) {
        this.pos = pos;
        this.slip = slip;
        this.breakdown = breakdown;
        this.carType = carType;
        this.fuel = fuel;
        this.tirePressure = tirePressure;
        this.driver = driver;
        this.tireModel = tireModel;
        this.actionSpace = new ActionSpace(ps, this);
        scores = new ArrayList();
        this.ps = ps;
        this.step = step;
    }

    /**
     * Return the next state after moving the cars position from current state.
     *
     * N.B. move distance should be in range:
     *      [ProblemSpec.CAR_MIN_MOVE, ProblemSpec.CAR_MAX_MOVE]
     *
     * @param move the distance to move
     * @param N the max position (i.e. the goal region)
     * @return the next state
     */
    public Node changePosition(int move, int N) {
        Node nexNode = copyNode();
        if (nexNode.pos + move > N) {
            nexNode.pos = N;
        } else if (nexNode.pos + move < 1) {
            // not zero indexed as per assignment spec
            nexNode.pos = 1;
        } else {
            nexNode.pos += move;
        }
        return nexNode;
    }

    /**
     * Return the next state after changing the slip condition of current state
     *
     * @param newSlip the new slip condition
     * @return the next state
     */
    public Node changeSlipCondition(boolean newSlip) {
        Node nextNode = copyNode();
        nextNode.slip = newSlip;
        return nextNode;
    }

    /**
     * Return the next state after changing the breakdown condition of current state
     *
     * @param newBreakdown the new breakdown condition
     * @return the next state
     */
    public Node changeBreakdownCondition(boolean newBreakdown) {
        Node nextNode = copyNode();
        nextNode.breakdown = newBreakdown;
        return nextNode;
    }

    /**
     * Return the next state after changing car type in current state.
     *
     * @param newCarType the new car type
     * @return the next state
     */
    public Node changeCarType(String newCarType) {
        Node nextNode = copyNode();
        nextNode.carType = newCarType;
        nextNode.fuel = ProblemSpec.FUEL_MAX;
        nextNode.tirePressure = TirePressure.ONE_HUNDRED_PERCENT;
        return nextNode;
    }

    /**
     * Return the next state after changing the driver in current state.
     *
     * @param newDriver the new driver
     * @return the next state
     */
    public Node changeDriver(String newDriver) {
        Node nextNode = copyNode();
        nextNode.driver = newDriver;
        return nextNode;
    }

    /**
     * Return the next state after changing the tire model in current state.
     *
     * @param newTire the new tire model
     * @return the next state
     */
    public Node changeTires(Tire newTire) {
        Node nextNode = copyNode();
        nextNode.tireModel = newTire;
        nextNode.tirePressure = TirePressure.ONE_HUNDRED_PERCENT;
        return nextNode;
    }

    /**
     * Return the next state after adding fuel to current state. Amount of fuel
     * in a state is capped at ProblemSpec.FUEL_MAX
     *
     * @param fuelToAdd amount of fuel to add
     * @return the next state
     */
    public Node addFuel(int fuelToAdd) {
        if (fuelToAdd < 0) {
            throw new IllegalArgumentException("Fuel to add must be positive");
        }

        Node nextNode = copyNode();
        if (nextNode.fuel + fuelToAdd > ProblemSpec.FUEL_MAX) {
            nextNode.fuel = ProblemSpec.FUEL_MAX;
        } else {
            nextNode.fuel += fuelToAdd;
        }
        return nextNode;
    }

    /**
     * Return the next state after consuming fuel in current state.
     *
     * @param fuelConsumed amount of fuel consumed
     * @return the next state
     */
    public Node consumeFuel(int fuelConsumed) {
        if (fuelConsumed < 0) {
            throw new IllegalArgumentException("Fuel consumed must be positive");
        }
        Node nextNode = copyNode();
        nextNode.fuel -= fuelConsumed;
        if (nextNode.fuel < ProblemSpec.FUEL_MIN) {
            throw new IllegalArgumentException("Too much fuel consumed: "
                    + fuelConsumed);
        }
        return nextNode;
    }

    /**
     * Return the next state after changing the tire pressure in current state.
     *
     * @param newTirePressure the new tire pressure
     * @return the next state
     */
    public Node changeTirePressure(TirePressure newTirePressure) {
        Node nextNode = copyNode();
        nextNode.tirePressure = newTirePressure;
        return nextNode;
    }

    /**
     * Return the next state after changing the car type and the driver in
     * current state.
     *
     * @param newCarType the new car type
     * @param newDriver the new driver
     * @return the next state
     */
    public Node changeCarAndDriver(String newCarType, String newDriver) {
        Node nextNode = copyNode();
        nextNode.carType = newCarType;
        nextNode.driver = newDriver;
        nextNode.fuel = ProblemSpec.FUEL_MAX;
        nextNode.tirePressure = TirePressure.ONE_HUNDRED_PERCENT;
        return nextNode;
    }

    /**
     * Return the next state after changing the tire model, adding fuel and
     * changing the tire pressure
     *
     * @param newTireModel new tire model
     * @param fuelToAdd the amount of fuel to add
     * @param newTirePressure the new tire pressure
     * @return the next state
     */
    public Node changeTireFuelAndTirePressure(Tire newTireModel, int fuelToAdd,
                                               TirePressure newTirePressure) {
        Node nextNode = addFuel(fuelToAdd);
        nextNode.tireModel = newTireModel;
        nextNode.tirePressure = newTirePressure;
        return nextNode;
    }

    /**
     * Copy this state, returning a deep copy
     *
     * @return deep copy of current state
     */
    public Node copyNode() {
        return new Node(pos, slip, breakdown, carType, fuel, tirePressure, driver,
                tireModel, ps, step);
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Score=").append(UCB1).append(" | ");
        sb.append("Node: [ ");
        sb.append("Pos=").append(pos).append(" | ");
        sb.append("Car=").append(carType).append(" | ");
        sb.append("Driver=").append(driver).append(" | ");
        sb.append("Tire=").append(tireModel.toString()).append(" | ");
        sb.append("Pressure=").append(tirePressure.asString()).append(" | ");
        sb.append("Fuel=").append(fuel).append(" ]\n");
        return sb.toString();
    }

    public int getPos() {
        return pos;
    }

    public boolean isInSlipCondition() {
        return slip;
    }

    public boolean isInBreakdownCondition() {
        return breakdown;
    }

    public String getCarType() {
        return carType;
    }

    public int getFuel() {
        return fuel;
    }

    public TirePressure getTirePressure() {
        return tirePressure;
    }

    public String getDriver() {
        return driver;
    }

    public Tire getTireModel() {
        return tireModel;
    }

}
