package Model;

/**
 * Created by Jordy on 19-11-2017.
 */

import java.util.LinkedList;

public class TraversalHandler {

    private TraversalStatus state;
    private Node agent;
    private LinkedList<Node> path;
    private int i = 0;
    private boolean firstT;
    private boolean firstE;

    //Need to edit this variable (for multiple e-pucks)
    private double maxSeparationDistance = 0;

    //TraversalStatus.START
    
    public TraversalHandler(LinkedList<Node> path, Node agent) {
        this.path = path;
        this.agent = agent;
        setState(TraversalStatus.INIT);
        firstT = true;
        firstE = true;
    }

    public void changePath(LinkedList<Node> path, int state) {
        path.clear();
        this.path = path;
        setState(TraversalStatus.INIT);
        firstT = true;
        firstE = true;
    }

    public String getState() {
        return state.name();
    }
    
    public void setState(TraversalStatus state) {
        this.state = state;
    }

    //update agent and retrieve next location
    public Node getNext(Node agent) {
        this.agent = agent;

        if (path.isEmpty()) {
            System.out.println("PATH IS EMPTY!");
            return null;
        }
        
        if (i >= path.size()) {
            System.out.println("I OUT OF BOUNDS");
        }

        //Edit with actual agents
        if (!checkCollission(null)) {
            if (firstT) {
                setState(TraversalStatus.TRAVERSING);
            } else if (state == TraversalStatus.EVADING) {
                //Shit hit the fan, this means we don't need to evade anymore, but we need to construct the shortest path anew
                //need SpecialGraph for this? To be implemented

                setState(TraversalStatus.TRAVERSING);
            }

            Node n = path.get(i);
            i++;
            return n;
        } else {
            return collissionAvoidance(null);
        }

        //Another option if we don't care about the path we already travelled:
        //Node n = path.get(0);
        //path.remove(0);
        //return n;
    }
    
    //IMPLEMENT EQUALS METHOD FOR NODE CLASS IF WE USE THIS FOR AGENT
    public boolean checkCollission(LinkedList<Node> agents) {
        for (Node otherAgent: agents) {
            double dist = Math.sqrt(Math.pow(otherAgent.getX() - agent.getX(), 2) + Math.pow(otherAgent.getY() - agent.getY(), 2));
            if (dist != 0) {
                return true;
            }
        }
        
        return false;
    }

    public Node collissionAvoidance(LinkedList<Node> agents) {
        //To be used with multiple e-pucks
        //Need to check out of bounds!!
        //We might run into trouble if they both 'evade' into walls, think about this!

        if (firstE || state == TraversalStatus.TRAVERSING) {
            setState(TraversalStatus.EVADING);
        }

        double dX = 0;
        double dY = 0;

        for (Node otherAgent: agents) {
            double dist = Math.sqrt(Math.pow(otherAgent.getX() - agent.getX(), 2) + Math.pow(otherAgent.getY() - agent.getY(), 2));

            if (dist <= maxSeparationDistance) {
                dX += otherAgent.getX() - agent.getX();
                dY += otherAgent.getY() - agent.getY();
            }
        }

        if (!agents.isEmpty()) {
            dX = -dX / agents.size();
            dY = -dY / agents.size();
        }

        return new Node((int)dX, (int)dY);
        //Use this to calculate the angle!
    }

}
