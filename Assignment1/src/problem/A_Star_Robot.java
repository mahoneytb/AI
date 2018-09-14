package problem;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.awt.geom.Point2D;
import java.util.LinkedList;
import java.util.Queue;

public class A_Star_Robot {

    private RobotConfig start;
    private ArrayList<RobotConfig> goals = new ArrayList<>();
    public ArrayList<RobotConfig> pathway = new ArrayList<>();

    public A_Star_Robot(RobotConfig start, ArrayList<RobotConfig> goals){
        for (RobotConfig g : goals) {
            g.cost = 0;
            this.goals.add(g);
        }
        this.start = start;
    }

    // Perform A* search
    public boolean compute(){
        if (goals.isEmpty()) { return false; }
        // Check if goal is reached
        if(goals.contains(this.start)){
            return true;
            // what do we return?
        }

        // Initialise priority queue and explored list
        PriorityQueue<RobotConfig> queue = new PriorityQueue<>(5, new checkCost());
        ArrayList<RobotConfig> explored = new ArrayList<>();
        ArrayList<RobotConfig> gen1 = getChildren(start);
        Heuristic(gen1, start);
        for (RobotConfig g : gen1) {g.parent = start;}
        queue.addAll(gen1);
        explored.add(start);

        // Begin search
        while(queue.peek() != null){
            // Pop highest priority from queue
            RobotConfig current = queue.poll();
            // Check if goal is reached and print out path taken
            if(this.goals.contains(current)) {
                createPath(current);
                return true;
            }
            // Otherwise expand the node and put the child nodes in the queue
            else{
                ArrayList<RobotConfig> children = getChildren(current);
                Heuristic(children, current);
                if(children.isEmpty()) {
                    explored.add(current);
                    continue;
                } else {
                    // only add if not yet explored
                    for (RobotConfig child : children) {
                        boolean done = false;
                        for (RobotConfig e_vs : explored) {
                            // if already explored...
                            if (child.equals(e_vs)) {
                                // add only if cost is lower
                                //if (child.cost >= e_vs.cost) {
                                    done = true;
                                //}
                            }
                        }
                        // otherwise add
                        if (!done) {
                            child.parent = current;
                            queue.add(child);
                        }
                    }
                }
            }
            explored.add(current);
        }
        return false;
    }

    // Comparator compares cost of priorityQueue elements
    class checkCost implements Comparator<RobotConfig>
    {
        public int compare(RobotConfig r1, RobotConfig r2)
        {
            return r1.cost - r2.cost;
        }
    }

    // Calculate heuristic from current to goal node
    private void Heuristic(ArrayList<RobotConfig> children, RobotConfig current) {

        for (RobotConfig child : children) {
            //RobotConfig p = child.parent;
            Point2D g = goals.get(0).getPos();

            if (goals.contains(child)) {
                child.cost = 0;
            } else if (current.getPos().getY() == g.getY() || child.getPos().getX() == g.getX()) {
                child.cost = current.g + 1;
            } else {
                child.cost = current.g + 10;
            }
        }
    }

    // Get the children and assign direction and update g cost
    private ArrayList<RobotConfig> getChildren(RobotConfig v) {
        ArrayList<RobotConfig> children = new ArrayList<RobotConfig>();
        for (RobotConfig n : v.neighbours) {
            // If 1st gen, add 1 to cost
            if (v.equals(start)) {
                n.g = 1;
            }
            // Otherwise, compare direction with parent and update cost: +1 if straight, +10 if turn
            else {
                Point2D g = goals.get(0).getPos();
                if (goals.contains(n)) {
                    n.g = v.g + 0;
                } else if (v.getPos().distance(g) < n.getPos().distance(g)) {
                    n.g = v.g + 1;
                } else {
                    n.g = v.g + 2;
                }
            }
            children.add(n);
        }
        return children;
    }

    // Construct pathway only including corner vertices
    private void createPath(RobotConfig r) {
        while (!r.equals(this.start)) {
            RobotConfig p = r.parent;
            pathway.add(r);
            r = p;
        }
        pathway.add(start);
    }
}
