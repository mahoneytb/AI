package MCTS;

import problem.*;

import java.util.ArrayList;
import java.util.Random;

// The list of actions for any given node
public class ActionSpace {
    private ProblemSpec ps;
    private boolean move;
    private ArrayList<Boolean> car_type;
    private ArrayList<Boolean> driver;
    private ArrayList<Boolean> tire_type;
    private boolean fuel;
    private ArrayList<Boolean> pressure;
    private Random rand = new Random();
    private int n_cars;
    private int n_drivers;
    private int n_tires;
    private ArrayList<Integer> set_pressures;

    public ActionSpace(ProblemSpec ps) {
        this.ps = ps;
        car_type = new ArrayList();
        driver = new ArrayList();
        tire_type = new ArrayList();
        pressure = new ArrayList();

        n_cars = ps.getCT();
        n_drivers = ps.getDT();
        n_tires = ps.getTireOrder().size();

        move = false;
        fuel = false;

        for (int c = 0; c < n_cars; c++) {
            car_type.add(false);
        }
        for (int d = 0; d < n_drivers; d++) {
            driver.add(false);
        }
        for (int t = 0; t < n_tires; t++) {
            tire_type.add(false);
        }
        for (int p = 0; p < 3; p++) {
            pressure.add(false);
        }

        set_pressures = new ArrayList();
        set_pressures.add(50);
        set_pressures.add(75);
        set_pressures.add(100);
    }

    // Return a random action which is yet to be explored
    public Action getNextUnexploredAction() {
        ArrayList<Integer> actions = new ArrayList<>();
        if (!move)                      { actions.add(0); }
        if (car_type.contains(false))   { actions.add(1); }
        if (driver.contains(false))     { actions.add(2); }
        if (tire_type.contains(false))  { actions.add(3); }
        if (!fuel)                      { actions.add(4); }
        if (pressure.contains(false))   { actions.add(5); }
        if (actions.isEmpty()) {
            return null;
        }

        int next = actions.get(rand.nextInt(actions.size()));
        int r;

        switch(next) {
            case 0:
                move = true;
                return new Action(ActionType.MOVE);
            case 1:
                r = rand.nextInt(n_cars);
                while (car_type.get(r)) {
                    r = rand.nextInt(n_cars);
                }
                String c = ps.getCarOrder().get(r);
                car_type.set(r, true);
                return new Action(ActionType.CHANGE_CAR, c);
            case 2:
                r = rand.nextInt(n_drivers);
                while (driver.get(r)) {
                    r = rand.nextInt(n_drivers);
                }
                String d = ps.getDriverOrder().get(r);
                driver.set(r, true);
                return new Action(ActionType.CHANGE_DRIVER, d);
            case 3:
                r = rand.nextInt(n_tires);
                while (tire_type.get(r)) {
                    r = rand.nextInt(n_tires);
                }
                Tire t = ps.getTireOrder().get(r);
                tire_type.set(r, true);
                return new Action(ActionType.CHANGE_TIRES, t);
            case 4:
                fuel = true;
                return new Action(ActionType.ADD_FUEL, 10);
            case 5:
                r = rand.nextInt(3);
                while (pressure.get(r)) {
                    r = rand.nextInt(3);
                }
                pressure.set(r, true);

                if (r==0) { return new Action(ActionType.CHANGE_PRESSURE, TirePressure.FIFTY_PERCENT); }
                if (r==1) { return new Action(ActionType.CHANGE_PRESSURE, TirePressure.SEVENTY_FIVE_PERCENT); }
                if (r==2) { return new Action(ActionType.CHANGE_PRESSURE, TirePressure.ONE_HUNDRED_PERCENT); }
        }
        return null;
    }


}
