package MCTS;

import problem.*;

/**
 * This class is the simulator for the problem.
 * The simulator takes in an action and returns the next state.
 */
public class Simulate {
    

    /** Problem spec for the current problem **/
    private ProblemSpec ps;
    /** The current state of the environment **/
    private Node currentNode;
    /** The number of steps taken **/
    private int steps;


    /**
     * Construct a new simulator instance from the given problem spec
     *
     * @param ps the ProblemSpec
     */
    public Simulate(ProblemSpec ps) {
        this.ps = ps;
    }

    /**
     * Perform an action against environment and receive the next state.
     *
     * @param a the action to perform
     * @return the next state or null if max time steps exceeded for problem
     */
    public Node next(Node currentNode, Action a) throws IllegalArgumentException {

        Node nextNode;
        this.currentNode = currentNode.copyNode();

        if (!actionValidForLevel(a)) {
            throw new IllegalArgumentException("ActionType A"
                    + a.getActionType().getActionNo()
                    + " is an invalid action for problem level "
                    + ps.getLevel());
        }

        switch(a.getActionType().getActionNo()) {
            case 1:
                nextNode = performA1();
                break;
            case 2:
                nextNode = performA2(a);
                break;
            case 3:
                nextNode = performA3(a);
                break;
            case 4:
                nextNode = performA4(a);
                break;
            case 5:
                nextNode = performA5(a);
                break;
            case 6:
                nextNode = performA6(a);
                break;
            case 7:
                nextNode = performA7(a);
                break;
            default:
                nextNode = performA8(a);
        }

        // handle slip and breakdown cases, we do this now so we can generate
        // correct output format
        if (nextNode.isInSlipCondition()) {
            // remain in same state but certain number of steps pass
            // -1 since we add 1 later
            steps += ps.getSlipRecoveryTime() - 1;
            nextNode = nextNode.changeSlipCondition(false);
        } else if (nextNode.isInBreakdownCondition()) {
            steps += ps.getRepairTime() - 1;
            nextNode = nextNode.changeBreakdownCondition(false);
        }
        steps += 1;

        return nextNode;
    }

    /**
     * Checks if given action is valid for the current problem level
     *
     * @param a the action being performed
     * @return True if action is value, False otherwise
     */
    private boolean actionValidForLevel(Action a) {
        return ps.getLevel().isValidActionForLevel(a.getActionType());
    }

    /**
     * Perform CONTINUE_MOVING action
     *
     * @return the next state
     */
    private Node performA1() {

        Node nextNode;

        // check there is enough fuel to make move in current state
        int fuelRequired = getFuelConsumption();
        int currentFuel = currentNode.getFuel();
        if (fuelRequired > currentFuel) {
            return currentNode;
        }

        // Sample move distance
        int moveDistance = sampleMoveDistance();

        // handle slip and breakdown cases, addition of steps handled in step method
        if (moveDistance == ProblemSpec.SLIP) {
            nextNode = currentNode.changeSlipCondition(true);
        } else if (moveDistance == ProblemSpec.BREAKDOWN) {
            nextNode = currentNode.changeBreakdownCondition(true);
        } else {
            nextNode = currentNode.changePosition(moveDistance, ps.getN());
        }

        // handle fuel usage for level 2 and above
        if (ps.getLevel().getLevelNumber() > 1) {
            nextNode = nextNode.consumeFuel(fuelRequired);
        }

        return nextNode;
    }

    /**
     * Return the move distance by sampling from conditional probability
     * distribution.
     *
     * N.B. this formula is not at all optimized for performance, so be wary if
     * trying to use it for finding a policy
     *
     * @return the move distance in range [-4, 5] or SLIP or BREAKDOWN
     */
    private int sampleMoveDistance() {

        double[] moveProbs = getMoveProbs();

        double p = Math.random();
        double pSum = 0;
        int move = 0;
        for (int k = 0; k < ProblemSpec.CAR_MOVE_RANGE; k++) {
            pSum += moveProbs[k];
            if (p <= pSum) {
                move = ps.convertIndexIntoMove(k);
                break;
            }
        }
        return move;
    }

    /**
     * Calculate the conditional move probabilities for the current state.
     *
     *          P(K | C, D, Ti, Te, Pressure)
     *
     * @return list of move probabilities
     */
    private double[] getMoveProbs() {

        // get parameters of current state
        Terrain terrain = ps.getEnvironmentMap()[currentNode.getPos() - 1];
        int terrainIndex = ps.getTerrainIndex(terrain);
        String car = currentNode.getCarType();
        String driver = currentNode.getDriver();
        Tire tire = currentNode.getTireModel();

        // calculate priors
        double priorK = 1.0 / ProblemSpec.CAR_MOVE_RANGE;
        double priorCar = 1.0 / ps.getCT();
        double priorDriver = 1.0 / ps.getDT();
        double priorTire = 1.0 / ProblemSpec.NUM_TYRE_MODELS;
        double priorTerrain = 1.0 / ps.getNT();
        double priorPressure = 1.0 / ProblemSpec.TIRE_PRESSURE_LEVELS;

        // get probabilities of k given parameter
        double[] pKGivenCar = ps.getCarMoveProbability().get(car);
        double[] pKGivenDriver = ps.getDriverMoveProbability().get(driver);
        double[] pKGivenTire = ps.getTireModelMoveProbability().get(tire);
        double pSlipGivenTerrain = ps.getSlipProbability()[terrainIndex];
        double[] pKGivenPressureTerrain = convertSlipProbs(pSlipGivenTerrain);

        // use bayes rule to get probability of parameter given k
        double[] pCarGivenK = bayesRule(pKGivenCar, priorCar, priorK);
        double[] pDriverGivenK = bayesRule(pKGivenDriver, priorDriver, priorK);
        double[] pTireGivenK = bayesRule(pKGivenTire, priorTire, priorK);
        double[] pPressureTerrainGivenK = bayesRule(pKGivenPressureTerrain,
                (priorTerrain * priorPressure), priorK);

        // use conditional probability formula on assignment sheet to get what
        // we want (but what is it that we want....)
        double[] kProbs = new double[ProblemSpec.CAR_MOVE_RANGE];
        double kProbsSum = 0;
        double kProb;
        for (int k = 0; k < ProblemSpec.CAR_MOVE_RANGE; k++) {
            kProb = magicFormula(pCarGivenK[k], pDriverGivenK[k],
                    pTireGivenK[k], pPressureTerrainGivenK[k], priorK);
            kProbsSum += kProb;
            kProbs[k] = kProb;
        }

        // Normalize
        for (int k = 0; k < ProblemSpec.CAR_MOVE_RANGE; k++) {
            kProbs[k] /= kProbsSum;
        }

        return kProbs;
    }

    /**
     * Convert the probability of slipping on a given terrain with 50% tire
     * pressure into a probability list, of move distance versus current
     * terrain and tire pressure.
     *
     * @param slipProb probability of slipping on current terrain and 50%
     *                 tire pressure
     * @return list of move probabilities given current terrain and pressure
     */
    private double[] convertSlipProbs(double slipProb) {

        // Adjust slip probability based on tire pressure
        TirePressure pressure = currentNode.getTirePressure();
        if (pressure == TirePressure.SEVENTY_FIVE_PERCENT) {
            slipProb *= 2;
        } else if (pressure == TirePressure.ONE_HUNDRED_PERCENT) {
            slipProb *= 3;
        }
        // Make sure new probability is not above max
        if (slipProb > ProblemSpec.MAX_SLIP_PROBABILITY) {
            slipProb = ProblemSpec.MAX_SLIP_PROBABILITY;
        }

        // for each terrain, all other action probabilities are uniform over
        // remaining probability
        double[] kProbs = new double[ProblemSpec.CAR_MOVE_RANGE];
        double leftOver = 1 - slipProb;
        double otherProb = leftOver / (ProblemSpec.CAR_MOVE_RANGE - 1);
        for (int i = 0; i < ProblemSpec.CAR_MOVE_RANGE; i++) {
            if (i == ps.getIndexOfMove(ProblemSpec.SLIP)) {
                kProbs[i] = slipProb;
            } else {
                kProbs[i] = otherProb;
            }
        }

        return kProbs;
    }

    /**
     * Apply bayes rule to all values in cond probs list.
     *
     * @param condProb list of P(B|A)
     * @param priorA prior probability of parameter A
     * @param priorB prior probability of parameter B
     * @return list of P(A|B)
     */
    private double[] bayesRule(double[] condProb, double priorA, double priorB) {

        double[] swappedProb = new double[condProb.length];

        for (int i = 0; i < condProb.length; i++) {
            swappedProb[i] = (condProb[i] * priorA) / priorB;
        }
        return swappedProb;
    }

    /**
     * Conditional probability formula from assignment 2 sheet
     *
     * @param pA P(A | E)
     * @param pB P(B | E)
     * @param pC P(C | E)
     * @param pD P(D | E)
     * @param priorE P(E)
     * @return numerator of the P(E | A, B, C, D) formula (still need to divide
     *      by sum over E)
     */
    private double magicFormula(double pA, double pB, double pC, double pD,
                                double priorE) {
        return pA * pB * pC * pD * priorE;
    }

    /**
     * Get the fuel consumption of moving given the current state
     *
     * @return move fuel consumption for current state
     */
    private int getFuelConsumption() {

        // get parameters of current state
        Terrain terrain = ps.getEnvironmentMap()[currentNode.getPos() - 1];
        String car = currentNode.getCarType();
        TirePressure pressure = currentNode.getTirePressure();

        // get fuel consumption
        int terrainIndex = ps.getTerrainIndex(terrain);
        int carIndex = ps.getCarIndex(car);
        int fuelConsumption = ps.getFuelUsage()[terrainIndex][carIndex];

        if (pressure == TirePressure.FIFTY_PERCENT) {
            fuelConsumption *= 3;
        } else if (pressure == TirePressure.SEVENTY_FIVE_PERCENT) {
            fuelConsumption *= 2;
        }
        return fuelConsumption;
    }

    /**
     * Perform CHANGE_CAR action
     *
     * @param a a CHANGE_CAR action object
     * @return the next state
     */
    private Node performA2(Action a) {

        if (currentNode.getCarType().equals(a.getCarType())) {
            // changing to same car type does not change state but still costs a step
            // no cheap refill here, muhahaha
            return currentNode;
        }

        return currentNode.changeCarType(a.getCarType());
    }

    /**
     * Perform CHANGE_DRIVER action
     *
     * @param a a CHANGE_DRIVER action object
     * @return the next state
     */
    private Node performA3(Action a) { return currentNode.changeDriver(a.getDriverType()); }

    /**
     * Perform the CHANGE_TIRES action
     *
     * @param a a CHANGE_TIRES action object
     * @return the next state
     */
    private Node performA4(Action a) {
        return currentNode.changeTires(a.getTireModel());
    }

    /**
     * Perform the ADD_FUEL action
     *
     * @param a a ADD_FUEL action object
     * @return the next state
     */
    private Node performA5(Action a) {
        // calculate number of steps used for refueling (minus 1 since we add
        // 1 in main function
        int stepsRequired = (int) Math.ceil(a.getFuel() / (float) 10);
        steps += (stepsRequired - 1);
        return currentNode.addFuel(a.getFuel());
    }

    /**
     * Perform the CHANGE_PRESSURE action
     *
     * @param a a CHANGE_PRESSURE action object
     * @return the next state
     */
    private Node performA6(Action a) {
        return currentNode.changeTirePressure(a.getTirePressure());
    }

    /**
     * Perform the CHANGE_CAR_AND_DRIVER action
     *
     * @param a a CHANGE_CAR_AND_DRIVER action object
     * @return the next state
     */
    private Node performA7(Action a) {

        if (currentNode.getCarType().equals(a.getCarType())) {
            // if car the same, only change driver so no sneaky fuel exploit
            return currentNode.changeDriver(a.getDriverType());
        }
        return currentNode.changeCarAndDriver(a.getCarType(),
                a.getDriverType());
    }

    /**
     * Perform the CHANGE_TIRE_FUEL_PRESSURE action
     *
     * @param a a CHANGE_TIRE_FUEL_PRESSURE action object
     * @return the next state
     */
    private Node performA8(Action a) {
        return currentNode.changeTireFuelAndTirePressure(a.getTireModel(),
                a.getFuel(), a.getTirePressure());
    }

    /**
     * Check whether a given state is the goal state or not
     *
     * @param s the state to check
     * @return True if s is goal state, False otherwise
     */
    public Boolean isGoalNode(Node s) {
        if (s == null) {
            return false;
        }
        return s.getPos() >= ps.getN();
    }

    /**
     * Get the current number of steps taken in latest simulation
     *
     * @return steps taken in latest simulation
     */
    public int getSteps() {
        return steps;
    }

}
