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
    LinkedList<Line> shortestPathLines = new LinkedList<>();
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

    private void resolveShortestPathLines() {
        if (!shortestPathLines.isEmpty()) shortestPathLines.clear();
        //if (shortestPath.size() < 3){
         //   System.out.println("Path too small");

         //   Line tmp = new Line(shortestPath.get(0), shortestPath.get(1));
          //  shortestPathLines.add(tmp);

       // } else {
            ArrayList<Node> aPath = new ArrayList<>(shortestPath);

            Node start = null;
            Line tmp = null;

            for (int i = 0; i < aPath.size() - 2; i++) {
                Node p1 = aPath.get(i);
                Node p2 = aPath.get(i+1);
                Node p3 = aPath.get(i+2);

                if (i == 0) start = p1;

                //x, y interchanged

                double left = (p2.x - p1.x) * (p3.y - p2.y);
                double right = (p3.x - p2.x) * (p2.y - p1.y);

                if (left == right) {
                    tmp = null;
                } else {
                    tmp = new Line(start, p2);
                    start = p2;
                }

                if (tmp != null) shortestPathLines.add(tmp);
            }

            tmp = new Line(start, aPath.get(aPath.size() - 1));
            shortestPathLines.add(tmp);
        //}

    }

    public LinkedList<Line> getShortestPathLines() {
        //System.out.println("orig no of points: " + shortestPath.size());
        resolveShortestPathLines();
        //System.out.println("no of lines: " + shortestPathLines.size());
        return shortestPathLines;
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

    /*@Override
    public boolean equals(Object o){
        return (this.x == ((Node)o).getX() && this.y == ((Node)o).getY());
    }*/

    public double getDistanceTo(Node o){
        return Math.sqrt((Math.pow((this.x - o.x),2) + Math.pow((this.y - o.y),2)));
    }

    public static void main(String[] args) {
        Node a = new Node(1, 1);
        Node b = new Node(2, 2);
        Node c = new Node(3, 3);
        Node e  = new Node(0, 0);

        LinkedList<Node> lel = new LinkedList<>();
        lel.add(a);
        lel.add(b);
        lel.add(c);
        e.setShortestPath(lel);
        System.out.println(e.getShortestPath());
        System.out.println(e.getShortestPathLines());
    }
}
