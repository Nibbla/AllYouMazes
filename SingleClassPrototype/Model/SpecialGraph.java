package Model;

import Interfaces.ObjectType;

import view.View;

import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.*;

/**
 * Created by Nibbla on 06.10.2017.
 */
public class SpecialGraph{
    private final int left;
    private final int right;
    private final int top;
    private final int botom;
    private final BufferedImage astarGridRepresentation;
    private JFrame frame = null;
    private Point start;
    Node[][] Grid;
    private Point end;


    public SpecialGraph(ObjectType[][] source, int imageType, RoboPos roboPos){

         left = 0;
         right = source.length;
         top = 0;
         botom = source[0].length;

        Grid = new Node[right][botom];

        double radious = roboPos.getRadious();

        int dialate = (int) Math.ceil(radious);

        for (int i = 0; i < dialate; i++) {
            source = View.dialate(source,ObjectType.wall,right,botom);
        }

       // nodes = new ArrayList<>(source.length*source[0].length);
        for (int x = 0; x < source.length; x++) {
            for (int y = 0; y < source[0].length; y++) {

                if (source[x][y]!= ObjectType.wall){
                    Node current = new Node(x,y);
                   //nodes.add(current);
                    Grid[x][y] = current;
                }





            }
        }

        for (int x = 0; x < source.length; x++) {
            for (int y = 0; y < source[0].length; y++) {
                if (Grid[x][y] == null) continue;



                for (int dx = x - 1; dx <= x + 1; dx++) {
                    for (int dy = y - 1; dy <= y + 1; dy++) {
                        if (dx < 0 || dx >= right || dy < 0 || dy >= botom) {
                            continue;

                        }
                        if (Grid[dx][dy] == null) continue;

                        if (dx == x && dy == y) continue;
                        if (Grid[dx][dy] == null) continue;
                        double dist = 1.;
                        if (dx != x && dy != y) dist = Math.sqrt(2);

                        Grid[x][y].addNeighbour(Grid[dx][dy],dist);

                    }
                }
            }
        }


        astarGridRepresentation = new BufferedImage(right,botom,imageType);
        for (int x = 0; x < right; x++) {
            for (int y = 0; y < botom; y++) {
                //if (ot[x][y]!=ObjectType.floor)astarGridRepresentation.setRGB(x,y,ot4[x][y].getColor());
                //else {

                    astarGridRepresentation.setRGB(x, y, source[x][y].getColor());
                //}
            }

        }

        for (int x = 0; x < Grid.length; x++) {
            for (int y = 0; y < Grid[0].length; y++) {
                //if (ot[x][y]!=ObjectType.floor)astarGridRepresentation.setRGB(x,y,ot4[x][y].getColor());
                //else {
                Node n = Grid[x][y];
                if (n != null) astarGridRepresentation.setRGB(n.x, n.y, Color.cyan.getRGB());
                //}
            }

        }

        int x = (int)roboPos.getX();
        int y = (int)roboPos.getY();
        double r = (int) roboPos.getRadious();

        for (int i = (int) (x-r); i <= x+r; i++) {
            for (int j = (int) (y-r); j < y+r; j++) {
                if (Math.sqrt((i-x)*(i-x)+(j-y)*(j-y))<r)
                astarGridRepresentation.setRGB(i, j, Color.BLACK.getRGB());

            }
        }

        if (frame!=null) frame.setVisible(false);
        frame = new JFrame();
        frame.getContentPane().setLayout(new FlowLayout());
        frame.getContentPane().add(new JLabel(new ImageIcon(astarGridRepresentation)));
        // frame.getContentPane().add(new JLabel(new ImageIcon(b2)));

        frame.pack();
        frame.setVisible(true);
        
    }
   


    public  ArrayList<Node>  calculatePathway(RoboPos roboPos, int goalX, int goalY) {
        ArrayList<Node> path = new ArrayList<>(30000);

        ArrayList<Node> unvisitedSet = new ArrayList<>(400000);

        Set<Node> settledNodes = new HashSet<>();
        Set<Node> unsettledNodes = new HashSet<>();

        int sX = (int) roboPos.getX();
        int sY = (int) roboPos.getY();



        if (Grid[sX][sY] == null || Grid[sX][sY] == null){
            return path;
        }

        for (int x = 0; x <right; x++) {
            for (int y = 0; y < botom; y++) {
                if (Grid[x][y] == null) continue;
                if (x == sX && y == sY) Grid[x][y].setDistance(0);
                else {
                    Grid[x][y].setDistance(Integer.MAX_VALUE);
                }

                unvisitedSet.add(Grid[x][y]);

            }
        }
        int count = 0;
            while (!unvisitedSet.isEmpty()) {
                Node n = getLowestDistanceNode(unvisitedSet);
                unvisitedSet.remove(n);
                 System.out.println(unvisitedSet.size());
                if (n  == null){
                    settledNodes.add(n);
                    repaint(roboPos);
                    break;
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

                astarGridRepresentation.setRGB(n.x, n.y, Color.RED.getRGB());
                settledNodes.add(n);

                count++;
                if (count%1000==0) repaint(roboPos);

            }


        for (Node n : Grid[goalX][goalY].shortestPath){
            astarGridRepresentation.setRGB(n.x, n.y, Color.GREEN.getRGB());
            for (int dx = n.x - 2; dx <= n.x + 2; dx++) {
                for (int dy = n.y - 2; dy <= n.y + 2; dy++) {
                    try {
                        astarGridRepresentation.setRGB(dx, dy, Color.GREEN.getRGB());
                    }catch (java.lang.ArrayIndexOutOfBoundsException f){

                    }

                }
            }
        }



        repaint(roboPos);


        return path;
    }

    private void repaint(RoboPos roboPos) {
        int x = (int)roboPos.getX();
        int y = (int)roboPos.getY();
        double r = (int) roboPos.getRadious();
        for (int i = (int) (x-r); i <= x+r; i++) {
            for (int j = (int) (y-r); j < y+r; j++) {
                if (Math.sqrt((i-x)*(i-x)+(j-y)*(j-y))<r)
                    astarGridRepresentation.setRGB(i, j, Color.BLACK.getRGB());

            }
        }

        frame.repaint();
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
