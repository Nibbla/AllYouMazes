package Model;

import Interfaces.ObjectType;

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
    private Point start;
    public ArrayList<Node> nodes;


    public SpecialGraph(PixelObjectType[][] source){

        int left = 0;
        int right = source.length;
        int top = 0;
        int botom = source[0].length;

        Node[][] temp = new Node[right][botom];


        nodes = new ArrayList<>(source.length*source[0].length);
        for (int x = 0; x < source.length; x++) {
            for (int y = 0; y < source[0].length; y++) {

                if (source[x][y].getSelectedClass()!= ObjectType.wall){
                    Node current = new Node(x,y);
                   nodes.add(current);
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
        System.out.println(nodes.size());
        
    }
   
    public void setStart(Point start) {
        this.start = start;
    }

    public void setGoal(Point end) {

    }

    public Path calculatePathway() {
        return null;
    }
}
