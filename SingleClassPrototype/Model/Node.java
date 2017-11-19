package Model;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

/**
 * Created by Nibbla on 26.09.2017.
 */
public class Node{
    int id;
    static int idcount;
    Node parent;
    Map<Node,Double> neighbours = new HashMap<>(8);
    ArrayList<Double> weights = new ArrayList<>(8);
    LinkedList<Node> shortestPath = new LinkedList<>();
    int x;
    int y;
    private double distance;


    public Node(int x, int y){
        this.x = x;
        this.y = y;

    }

    public void addNeighbour(Node n, double weight){
        this.neighbours.put(n,weight);

    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    public double getDistance() {
        return distance;
    }

    public LinkedList<Node> getShortestPath() {
        return shortestPath;
    }

    public void setShortestPath(LinkedList<Node> shortestPath) {
        this.shortestPath = shortestPath;
    }

    public Map<Node, Double> getAdjacentNodes() {
        return neighbours;
    }

    public int getX() {
        return this.x;
    }

    public int getY() {
        return this.y;
    }

    public String toString() {
        return "[x: " + y + ", y: " + x + "]";
    }
}
