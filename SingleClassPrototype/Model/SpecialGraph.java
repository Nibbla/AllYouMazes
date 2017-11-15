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
    private final int smallRight;
    private final int top;
    private final int smallBotom;
    private final BufferedImage astarGridRepresentation;
    private final int graphSkip;
    private JFrame frame = null;
    private Point start;
    Node[][] Grid;
    private Point end;
    private boolean visible;


    public SpecialGraph(ObjectType[][] source, int imageType, RoboPos roboPos, int graphSkip, boolean showPathway){

         left = 0;
         int right = source.length;
         top = 0;
         int botom = source[0].length;
        this.graphSkip = graphSkip;

         smallRight = right/graphSkip;
        smallBotom = botom/graphSkip;

        Grid = new Node[smallRight][smallBotom];

        double radious = roboPos.getRadious();

        int dialate = (int) Math.ceil(radious);

        for (int i = 0; i < dialate; i++) {
            source = View.dialate(source,ObjectType.wall,right,botom);
        }

       // nodes = new ArrayList<>(source.length*source[0].length);
        for (int x = 0; x < smallRight; x++) {
            for (int y = 0; y < smallBotom; y++) {

                if (source[x*graphSkip][y*graphSkip]!= ObjectType.wall){
                    Node current = new Node(x*graphSkip,y*graphSkip);
                   //nodes.add(current);
                    Grid[x][y] = current;
                }





            }
        }

        for (int x = 0; x < smallRight; x+=1) {
            for (int y = 0; y < smallBotom; y+=1) {
                if (Grid[x][y] == null) continue;



                for (int dx = x - 1; dx <= x + 1; dx+=1) {
                    for (int dy = y - 1; dy <= y + 1; dy+=1) {
                        if (dx < 0 || dx >= smallRight || dy < 0 || dy >= smallBotom) {
                            continue;

                        }
                        if (Grid[dx][dy] == null) continue;

                        if (dx == x && dy == y) continue;
                        if (Grid[dx][dy] == null) continue;
                        double dist = graphSkip;
                        if (dx != x && dy != y) dist = Math.sqrt(2*graphSkip*graphSkip);

                        Grid[x][y].addNeighbour(Grid[dx][dy],dist);

                    }
                }
            }
        }

        astarGridRepresentation = new BufferedImage(smallRight,smallBotom,imageType);
       if (showPathway) showPathway(source, imageType, roboPos, graphSkip);


    }

    private void showPathway(ObjectType[][] source, int imageType, RoboPos roboPos, int graphSkip) {

        for (int x = 0; x < smallRight; x++) {
            for (int y = 0; y < smallBotom; y++) {
                //if (ot[x][y]!=ObjectType.floor)astarGridRepresentation.setRGB(x,y,ot4[x][y].getColor());
                //else {

                    astarGridRepresentation.setRGB(x, y, source[x*graphSkip][y*graphSkip].getColor());
                //}
            }

        }

        for (int x = 0; x < Grid.length; x+=1) {
            for (int y = 0; y < Grid[0].length; y+=1) {
                //if (ot[x][y]!=ObjectType.floor)astarGridRepresentation.setRGB(x,y,ot4[x][y].getColor());
                //else {
                Node n = Grid[x][y];
                try {
                    if (n != null) astarGridRepresentation.setRGB(n.x/graphSkip, n.y/graphSkip, Color.cyan.getRGB());
                }catch (ArrayIndexOutOfBoundsException a){
                    System.out.println("baa");
                }

                //}
            }

        }

        int x = (int)roboPos.getX()/graphSkip;
        int y = (int)roboPos.getY()/graphSkip;
        double r = (int) roboPos.getRadious()/graphSkip;

        for (int i = (int) (x-r); i <= x+r; i++) {
            for (int j = (int) (y-r); j < y+r; j++) {
                if (i < 0 || i >= smallRight || j < 0 || j >= smallBotom) {
                    continue;
                }

                if (Math.sqrt((i-x)*(i-x)+(j-y)*(j-y))<r)
                astarGridRepresentation.setRGB(i, j, Color.BLACK.getRGB());

            }
        }


        frame = new JFrame();
        frame.getContentPane().setLayout(new FlowLayout());
        frame.getContentPane().add(new JLabel(new ImageIcon(astarGridRepresentation)));
        // frame.getContentPane().add(new JLabel(new ImageIcon(b2)));

        frame.pack();
        frame.setVisible(true);
    }


    public ArrayList<Node> calculatePathway(RoboPos roboPos, int goalX, int goalY, boolean showAstar) {
        ArrayList<Node> path = new ArrayList<>(30000);

        ArrayList<Node> unvisitedSet = new ArrayList<>(400000);

        Set<Node> settledNodes = new HashSet<>();
        Set<Node> unsettledNodes = new HashSet<>();

        int sX = (int) roboPos.getX()/graphSkip;
        int sY = (int) roboPos.getY()/graphSkip;



        if (Grid[sX][sY] == null || Grid[sX][sY] == null){
            return path;
        }

        for (int x = 0; x <smallRight; x++) {
            for (int y = 0; y < smallBotom; y++) {
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

               if(showAstar)  astarGridRepresentation.setRGB(n.x/graphSkip, n.y/graphSkip, Color.RED.getRGB());
                settledNodes.add(n);

                count++;
                if (count%1000==0) repaint(roboPos);

            }


        for (Node n : Grid[goalX][goalY].shortestPath){
            int nx = n.x / graphSkip;
            int ny = n.y / graphSkip;
            if(showAstar) astarGridRepresentation.setRGB(nx, ny, Color.GREEN.getRGB());
            for (int dx = nx - 2; dx <= nx + 2; dx++) {
                for (int dy = ny - 2; dy <= ny + 2; dy++) {
                    try {
                        if(showAstar) astarGridRepresentation.setRGB(dx, dy, Color.GREEN.getRGB());
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
                    astarGridRepresentation.setRGB(i/graphSkip, j/graphSkip, Color.BLACK.getRGB());

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

    public void setVisible(boolean visible) {
        if (frame!=null){
            frame.setVisible(false);
            frame=null;
        }
    }
}
