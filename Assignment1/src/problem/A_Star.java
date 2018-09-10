package problem;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Comparator;
import java.util.PriorityQueue;

/**
 * A*
 */
public class A_Star {

    private Vertex start;
    private Vertex goal;
    public ArrayList<Vertex> pathway = new ArrayList<Vertex>();
    private Double grade;

    public A_Star(Vertex start, Vertex goal, double grade){
        this.goal = goal;
        goal.cost=0;
        this.start = start;
        this.grade = grade;
    }

    // Perform A* search
    public boolean compute(){

        // Check if goal is reached
        if(this.start.isEqual(goal)){
            return true;
            // what do we return?
        }

        // Initialise priority queue and explored list
        PriorityQueue<Vertex> queue =
                new PriorityQueue<Vertex>(5, new checkCost());
        ArrayList<Vertex> explored = new ArrayList<>();
        ArrayList<Vertex> gen1 = getChildren(start);
        Heuristic(gen1);
        queue.addAll(gen1);
        explored.add(start);

        // Begin search
        while(queue.peek() != null){
            // Pop highest priority from queue
            Vertex current = queue.poll();
            // Check if goal is reached and print out path taken
            if(current.isEqual(this.goal)) {
                createPath(current);
                return true;
            }
            // Otherwise expand the node and put the child nodes in the queue
            else{
                ArrayList<Vertex> children = getChildren(current);
                Heuristic(children);
                if(children.isEmpty()) {
                    explored.add(current);
                    continue;
                } else {
                    // only add if not yet explored
                    for (Vertex child : children) {
                        boolean done = false;
                        for (Vertex e_vs : explored) {
                            // if already explored...
                            if (child.isEqual(e_vs)) {
                                // add only if cost is lower
                                if (child.cost >= e_vs.cost) {
                                    done = true;
                                }
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
    class checkCost implements Comparator<Vertex>
    {
        public int compare(Vertex v1, Vertex v2)
        {
            return v1.cost - v2.cost;
        }
    }

    // Calculate heuristic from current to goal node
    // Number of different cells divided by 2
    private void Heuristic(ArrayList<Vertex> children) {
        double gX = goal.getPos().getX();
        double gY = goal.getPos().getY();
        for (Vertex child : children) {
            double cX = child.getPos().getX();
            double cY = child.getPos().getY();
            char d = child.direction;
            boolean shortEdge = false;
            // Check if shorter distance
            if ((Math.abs(cX-gX) < 0.001) || (Math.abs(cY-gY) < 0.001)) { shortEdge = true; }

            boolean toward =    (cX<gX) && (d=='r') ||  // Child to the left of goal moving right
                                (cX>gX) && (d=='l') ||  // Child to the right of goal moving left
                                (cY<gY) && (d=='u') ||  // Child below the goal moving up
                                (cY>gY) && (d=='d');    // Child above the goal moving down

            if (toward || shortEdge)    { child.cost = child.g + 1; }
            //else if (toward)            { child.cost = child.g + 1; }
            else                        { child.cost = child.g + 5; }
        }
    }

    // Get the children and assign direction and update g cost
    private ArrayList<Vertex> getChildren(Vertex v) {
        ArrayList<Vertex> children = new ArrayList<Vertex>();
        for (Vertex n : v.getNeighbours()) {
            if (!v.queryConnect(n)) { continue; }
            if (n.getPos().getX() < v.getPos().getX()) {
                n.direction = 'l';
            } else if (n.getPos().getX() > v.getPos().getX()) {
                n.direction = 'r';
            } else if (n.getPos().getY() > v.getPos().getY()) {
                n.direction = 'u';
            } else if (n.getPos().getY() < v.getPos().getY()) {
                n.direction = 'd';
            }
            // If 1st gen, add 1 to cost
            if (v.isEqual(start)) {
                n.g = 1;
            }
            // Otherwise, compare direction with parent and update cost: +1 if straight, +10 if turn
            else {
                if (v.direction == n.direction) {
                    n.g = v.g + 1;
                } else {
                    n.g = v.g + 10;
                }
            }
            children.add(n);
        }
        return children;
    }

    // Construct pathway only including corner vertices
    private void createPath(Vertex v) {
        boolean finished = false;
        pathway.add(v);
        while (!finished) {
            Vertex p;

            if (v.parent == null) {
                p = start;
                finished = true;
            } else {
                p = v.parent;
            }

            // Recompute direction
            if (p.getPos().getX() < v.getPos().getX()) {
                p.direction = 'r';
            } else if (p.getPos().getX() > v.getPos().getX()) {
                p.direction = 'l';
            } else if (p.getPos().getY() > v.getPos().getY()) {
                p.direction = 'd';
            } else if (p.getPos().getY() < v.getPos().getY()) {
                p.direction = 'u';
            }

            // If a corner add to pathway
            if (p.direction != v.direction) {
                pathway.add(v);
            }
            v = p;
        }
        pathway.add(start);
    }
}