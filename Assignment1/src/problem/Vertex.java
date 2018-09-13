package problem;

import java.awt.geom.Point2D;
import java.util.*;

// A vertex in C space with neighbours
public class Vertex {
    public Point2D pos;
    private HashMap<Vertex, Boolean> m = new HashMap<Vertex, Boolean>();
    public boolean collisionFree = true;
    public boolean outOfPathway = false;
    public Vertex parent;
    public int cost;
    public int g;
    public char direction;
    public boolean sameAsParent = false;

    public Vertex(Point2D pos) {
        this.pos = pos;
    }

    // Add a neighbour
    public void addNeighbour(Vertex n) {
        m.put(n, true); // Connected until proven false
    }

    // Getters
    public Point2D getPos() { return pos; }
    public Set<Vertex> getNeighbours() { return m.keySet();}

    // Check the connection status of neighbour
    public boolean queryConnect(Vertex neighbour) { return m.get(neighbour); }

    // Set the connection status of neighbour
    public void setConnect(Vertex neighbour, boolean con) {
        m.replace(neighbour, con);
    }

    // See if vertex is in the same position
    public boolean isEqual(Vertex v) {
        if (pos.equals(v.getPos())) { return true; }
        return false;
    }
}
