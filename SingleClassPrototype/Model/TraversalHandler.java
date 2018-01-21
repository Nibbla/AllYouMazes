package Model;

/**
 * Created by Jordy on 19-11-2017.
 */

import java.util.LinkedList;

public class TraversalHandler {

    private TraversalStatus state;
    private Node agent;
    private LinkedList<Line> path;
    private int i = 0;
    private boolean firstT;
    private boolean firstE;

    //Need to editFields this variable (for multiple e-pucks)
    private double maxSeparationDistance = 0;

    //TraversalStatus.START
    
    public TraversalHandler(LinkedList<Line> path, Node agent) {
        this.path = path;
        this.agent = agent;
        setState(TraversalStatus.INIT);
        firstT = true;
        firstE = true;
    }

    public void changePath(LinkedList<Line> path, int state) {
        //this.path.clear();
        this.path = path;
        Line n = path.getLast();
        this.path.add(new Line(n.getA(),n.getA()));
		//System.out.println("change: " + path);
	    //System.out.println("change: " + this.path);
        setState(TraversalStatus.INIT);
        firstT = true;
        firstE = true;
		i = 0;
    }

    public void step() {
        if (i < (this.path.size() - 1)){
            i++;
        }else{
            setState(TraversalStatus.FINISH);
        }
    }

    public int getIndex() {
        return i;
    }

    public Line getLine(int index) {
		//System.out.println("get: " + this.path);
        return path.get(index);
    }

	public int size() {
		return path.size();
	}

    public String getStateName() {
        return state.name();
    }

    public TraversalStatus getState(){
        return state;
    }
    
    public void setState(TraversalStatus state) {
        this.state = state;
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
