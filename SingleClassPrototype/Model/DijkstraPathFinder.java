package Model;

import Interfaces.ObjectType;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import view.View;

import javax.swing.*;
import java.awt.*;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.util.*;

/**
 * Created by Nibbla on 06.10.2017.
 */
public class DijkstraPathFinder {
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


    public DijkstraPathFinder(ObjectType[][] source, int imageType, RoboPos roboPos, int graphSkip, boolean showPathway){

         left = 0;
         int right = source.length;
         top = 0;
         int botom = source[0].length;
        this.graphSkip = graphSkip;

         smallRight = right/graphSkip;
        smallBotom = botom/graphSkip;

        Grid = new Node[smallRight][smallBotom];

        double radious = roboPos.getRadius();

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



    //This method is a total mess and will be cleaned up soon
    public static  Node[][] retrieveDijcstraGrid(Mat gray, MatOfPoint2f contour, double goalX, double goalY, int stepSize, org.opencv.core.Point[] optionalTabooAreaCenter, double[] optionalTabooAreaRadiusSquared, Rect[] optionalTabooArea) {
        int sRows = gray.rows() / stepSize;
        int sCols = gray.cols() / stepSize;
        Node[][] grid = new Node[sRows][sCols];

        for (int x = 0; x < sRows; x++) {
            for (int y = 0; y < sCols; y++) {
                //test the flipping of x and y one day
                if (optionalTabooAreaCenter!=null) {if (insideTabooArea(y,x,optionalTabooAreaCenter,optionalTabooAreaRadiusSquared,optionalTabooArea,stepSize)){

                    continue;
                }}
                double t = Imgproc.pointPolygonTest(contour, new org.opencv.core.Point(y * stepSize, x * stepSize), false);
                if (t == -1) {
                    grid[x][y] = new Node(x * stepSize, y * stepSize);
                }
            }
        }

        for (int x = 0; x < sRows; x++) {
            for (int y = 0; y < sCols; y++) {
                if (grid[x][y] == null) continue;

                for (int dx = x - 1; dx <= x + 1; dx += 1) {
                    for (int dy = y - 1; dy <= y + 1; dy += 1) {
                        if (dx < 0 || dx >= sRows || dy < 0 || dy >= sCols) continue;
                        if (grid[dx][dy] == null) continue;
                        if (dx == x && dy == y) continue;

                        double dist = stepSize;
                        if (dx != x && dy != y) dist = Math.sqrt(2 * stepSize * stepSize);
                        grid[x][y].addNeighbour(grid[dx][dy], dist);
                    }
                }
            }
        }

        ArrayList<Node> unvisitedSet = new ArrayList<>(400000);

        Set<Node> settledNodes = new HashSet<>();
        Set<Node> unsettledNodes = new HashSet<>();

        int sX = (int) (goalX / stepSize);
        int sY = (int) (goalY / stepSize);

        while (sX % stepSize != 0) {
            sX++;
        }

        while (sY % stepSize != 0) {
            sY++;
        }

        if (grid[sY][sX] == null) System.out.println("No pathway possible");



        for (int x = 0; x < sRows; x++) {
            for (int y = 0; y < sCols; y++) {
                if (grid[x][y] == null) continue;

                if (x == sY && y == sX) grid[x][y].setDistance(0);
                else grid[x][y].setDistance(Integer.MAX_VALUE);

                unvisitedSet.add(grid[x][y]);
            }
        }
        
        //boolean stop = false;

        while (!unvisitedSet.isEmpty()) {
            Node n = getLowestDistanceNode(unvisitedSet);
            unvisitedSet.remove(n);

            //if (n.getX() == 0 && n.getY() == 0) stop = true;

            if (n == null) {
                settledNodes.add(n);
                break;
            }

            if (n.getAdjacentNodes() == null) {
                settledNodes.add(n);
                continue;
            }

            if (n.getAdjacentNodes().size() == 0) {
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

            //if (stop) break;
        }

//        System.out.println(unvisitedSet.size());
//


        return grid;
    }

    public static boolean insideTabooArea(int x, int y, org.opencv.core.Point[] optionalTabooAreaCenter, double[] optionalTabooAreaRadiusSquared, Rect[] optionalTabooArea, int stepSize) {
        //the x y exchange shouldnt make a differenz as we handle a square and a circle
        for (int i = 0; i < optionalTabooAreaCenter.length; i++) {
            Rect r = optionalTabooArea[i];
            org.opencv.core.Point center = optionalTabooAreaCenter[i];
            if (x >= r.x && x <= r.x+r.width && y >= r.y && y <= r.y + r.height){
                //System.out.println(x+"_"+y + " is inside taboo sqaure");
                //check if inside circle
                double d = (center.x - x)*(center.x - x)+(center.y-y)*(center.y-y);

                if (d <= optionalTabooAreaRadiusSquared[i]){
                   //System.out.println(x+"_"+y + " is inside taboo area");
                    return true;
                }
            }
        }

        return false;
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
        double r = (int) roboPos.getRadius()/graphSkip;

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
        frame.setFocusableWindowState(false);


        frame.setVisible(true);
        frame.setFocusableWindowState(true);
    }

    public static LinkedList<Node> getShortestPathFromGrid(Node[][] grid, RoboPos rb, int stepsize) {

        return grid[(int) (rb.getX()/stepsize)][(int) (rb.getY()/stepsize)].shortestPath;
    }

    public static LinkedList<Line> getShortestPathFromGridLine(Node[][] grid, RoboPos rb, int stepsize) {

        return grid[(int) (rb.getX()/stepsize)][(int) (rb.getY()/stepsize)].getShortestPathLines();
    }

    public LinkedList<Node> calculatePathway(RoboPos roboPos, int goalX, int goalY, boolean showAstar) {


        ArrayList<Node> unvisitedSet = new ArrayList<>(400000);

        Set<Node> settledNodes = new HashSet<>();
        Set<Node> unsettledNodes = new HashSet<>();

        int sX = (int) roboPos.getX()/graphSkip;
        int sY = (int) roboPos.getY()/graphSkip;



        if (Grid[sX][sY] == null || Grid[sX][sY] == null){
            return null;
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
                //System.out.println(unvisitedSet.size());
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


        return Grid[goalX][goalY].shortestPath;
    }

    private void repaint(RoboPos roboPos) {
        int x = (int)roboPos.getX();
        int y = (int)roboPos.getY();
        double r = (int) roboPos.getRadius();
        for (int i = (int) (x-r); i <= x+r; i++) {
            for (int j = (int) (y-r); j < y+r; j++) {
                if (Math.sqrt((i-x)*(i-x)+(j-y)*(j-y))<r)
                    astarGridRepresentation.setRGB(i/graphSkip, j/graphSkip, Color.BLACK.getRGB());

            }
        }

        frame.repaint();
    }

    public static void calculateMinimumDistance(Node evaluationNode,
                                                double edgeWeigh, Node sourceNode) {
        double sourceDistance = sourceNode.getDistance();
        if (sourceDistance + edgeWeigh < evaluationNode.getDistance()) {
            evaluationNode.setDistance(sourceDistance + edgeWeigh);
            LinkedList<Node> shortestPath = new LinkedList<>(sourceNode.getShortestPath());
            shortestPath.add(sourceNode);
            evaluationNode.setShortestPath(shortestPath);
        }
    }


        public static Node getLowestDistanceNode(ArrayList<Node> unsettledNodes){
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


    public static LinkedList<Node> reverseLinkedList(LinkedList<Node> shortestPath) {
        LinkedList<Node> linky = new LinkedList<>();
        while (!shortestPath.isEmpty()){
            linky.add(shortestPath.getLast());
            shortestPath.removeLast();
        }
        return linky;
    }

    public static LinkedList<Line> reverseLinkedListLine(LinkedList<Line> shortestPath) {
        LinkedList<Line> linky = new LinkedList<>();
        while (!shortestPath.isEmpty()){
            linky.add(shortestPath.getLast());
            shortestPath.removeLast();
        }
        return linky;
    }


    public static void invertAAndBs(LinkedList<Line> shortestPathToObject) {
        for (Line l : shortestPathToObject){
            l.reverseNodes();
        }
    }
}
