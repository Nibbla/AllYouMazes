package Model;

import Interfaces.ObjectType;

import javafx.util.Pair;
import view.PixelObjectType;

import java.awt.*;
import java.util.*;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.stream.Stream;

/**
 * Created by Nibbla on 06.10.2017.
 */
public class SpecialGraph{
    private final int left;
    private final int right;
    private final int top;
    private final int botom;
    private Point start;
    Node[][] temp;
    private Point end;


    public SpecialGraph(PixelObjectType[][] source){

         left = 0;
         right = source.length;
         top = 0;
         botom = source[0].length;

        temp = new Node[right][botom];


       // nodes = new ArrayList<>(source.length*source[0].length);
        for (int x = 0; x < source.length; x++) {
            for (int y = 0; y < source[0].length; y++) {

                if (source[x][y].getSelectedClass()!= ObjectType.wall){
                    Node current = new Node(x,y);
                   //nodes.add(current);
                    temp[x][y] = current;
                }





            }
        }

        for (int x = 0; x < source.length; x++) {
            for (int y = 0; y < source[0].length; y++) {
                if (temp[x][y] == null) continue;



                for (int dx = x - 1; dx < x + 1; dx++) {
                    for (int dy = y - 1; dy < y + 1; dy++) {
                        if (dx < 0 || dx >= right || dy < 0 || dy >= botom) {
                            continue;

                        }
                        if (temp[dx][dy] == null) continue;

                        if (dx == x && dy == y) continue;
                        if (temp[dx][dy] == null) continue;
                        double dist = 1.;
                        if (dx != x && dy != y) dist = Math.sqrt(2);

                        temp[x][y].addNeighbour(temp[dx][dy],dist);

                    }
                }
            }
        }

        
    }
   


    public  void  calculatePathway(Pair<Double, Double> start) {
        ArrayList<Node> path = new ArrayList<>(30000);

        ArrayList<Node> unvisitedSet = new ArrayList<>(400000);

        Set<Node> settledNodes = new HashSet<>();
        Set<Node> unsettledNodes = new HashSet<>();

        int sX = (int) start.getKey().doubleValue();
        int sY = (int) start.getValue().doubleValue();



        if (temp[sX][sY] == null || temp[sX][sY] == null){
            return;
        }

        for (int x = 0; x <right; x++) {
            for (int y = 0; y < botom; y++) {
                if (temp[x][y] == null) continue;
                if (x == sX && y == sY) temp[x][y].setDistance(0);
                else {
                    temp[x][y].setDistance(Integer.MAX_VALUE);
                }

                unvisitedSet.add(temp[x][y]);

            }
        }

            while (!unvisitedSet.isEmpty()) {
                Node n = getLowestDistanceNode(unvisitedSet);
                unvisitedSet.remove(n);
                 System.out.println(unvisitedSet.size());
                if (n  == null){
                    settledNodes.add(n);
                    continue;
                }

                if (n.getAdjacentNodes()  == null){
                    settledNodes.add(n);
                    continue;
                }

                if (n.getAdjacentNodes().size() == 0){
                    settledNodes.add(n);
                    continue;
                }
                Set<Map.Entry<Node, Double>> entrySet = n.getAdjacentNodes().entrySet();
                for (Map.Entry<Node, Double> adjacencyPair :
                        entrySet) {
                    Node adjacentNode = adjacencyPair.getKey();
                    double edgeWeight = adjacencyPair.getValue();
                    if (!settledNodes.contains(adjacentNode)) {
                        calculateMinimumDistance(adjacentNode, edgeWeight, n);

                        unsettledNodes.add(adjacentNode);
                    }
                }
                settledNodes.add(n);

            }





    }

    private static void calculateMinimumDistance(Node evaluationNode,
                                                 double edgeWeigh, Node sourceNode) {
        double sourceDistance = sourceNode.getDistance();
        if (sourceDistance + edgeWeigh < evaluationNode.getDistance()) {
            evaluationNode.setDistance(sourceDistance + edgeWeigh);
            LinkedList<Node> shortestPath = new LinkedList<>(sourceNode.getShortestPath());
            shortestPath.add(sourceNode);
            evaluationNode.setShortestPath(shortestPath);
        }
    }


        private static Node getLowestDistanceNode(ArrayList<Node> unsettledNodes){
            Node lowestDistanceNode = null;
            double lowestDistance = Integer.MAX_VALUE;
            for (Node node: unsettledNodes) {
                double nodeDistance = node.getDistance();
                if (nodeDistance < lowestDistance) {
                    lowestDistance = nodeDistance;
                    lowestDistanceNode = node;
                }
            }
            if (lowestDistanceNode == null) {
                double lowestDistance2 = Integer.MAX_VALUE;
                for (Node node: unsettledNodes) {
                    double nodeDistance = node.getDistance();
                    if (nodeDistance < lowestDistance2) {
                        lowestDistance2 = nodeDistance;
                        lowestDistanceNode = node;
                    }
                }
            }
            return lowestDistanceNode;
        }
}
