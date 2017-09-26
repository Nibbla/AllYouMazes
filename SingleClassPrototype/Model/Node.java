package Model;

import java.util.List;

/**
 * Created by Nibbla on 26.09.2017.
 */
public abstract class Node{
    Node parent;
    List<Node> children;

    public int length(){
        return 0;
    }
}
