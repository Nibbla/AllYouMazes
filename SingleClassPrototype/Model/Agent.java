package Model;

public class Agent {


    private RoboPos currentPosition;
    private TraversalHandler handler;

    // TODO: make use of ROS_ID in controller. this is to anticipate multiple epucks.
    private int ROS_ID;

    public Agent(int ROS_ID, RoboPos currentPosition, TraversalHandler handler){
        this.ROS_ID = ROS_ID;
        this.currentPosition = currentPosition;
        this.handler = handler;
    }


    public RoboPos getCurrentPosition() {
        return currentPosition;
    }

    public void setCurrentPosition(RoboPos currentPosition) {
        this.currentPosition = currentPosition;
    }

    public TraversalHandler getHandler() {
        return handler;
    }

    public void setHandler(TraversalHandler handler) {
        this.handler = handler;
    }

    public int getROS_ID() {
        return ROS_ID;
    }

    public void setROS_ID(int ROS_ID) {
        this.ROS_ID = ROS_ID;
    }

}
