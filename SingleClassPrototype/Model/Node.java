package Model;

import Interfaces.ObjectType;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by Nibbla on 26.09.2017.
 */
public class Node{
    int id;
    static int idcount;
    Node parent;
    ArrayList<Node> neighbours = new ArrayList<>(8);
    ArrayList<Double> weights = new ArrayList<>(8);

    int x;
    int y;


    public Node(int x, int y){
        this.x = x;
        this.y = y;

    }

    public void addNeighbour(Node n, double weight){
        this.neighbours.add(n);
        this.weights.add(weight);
    }
}
