package Model;

public class Agent {


    private RoboPos currentPosition;
    private RoboPos lastPosition = new RoboPos(0,0,0,0);
    private TraversalHandler handler;
    private Node currentGoal;
    private double rotationCoefficient = 0, linearCoefficient = 0, lastRotationCoefficient = 0;
    private boolean turning, moving, alreadyTurning, alreadyMoving, stopped;

    private final double PROXIMITY = 20;
    private final double ROTATIONERROR = 20;

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

    public void update(RoboPos newPosition, Node rotationKP){
        lastPosition.setPosition(currentPosition.getX(), currentPosition.getY());
        lastPosition.setRadius(currentPosition.getRadius());
        lastPosition.setDirection(currentPosition.getDirection());

        currentPosition = newPosition;
        currentPosition.setDirection(currentPosition.getAngleTo(rotationKP));

        Node currentPathPosition = handler.getNode(handler.getIndex());
        Node nextPathPosition = handler.getNode(handler.getIndex() + 1);

        // TODO: swap Y and X as soon as path returns as X and Y
        double correctAngle = currentPosition.getAngleTo(new Node(nextPathPosition.getY(), nextPathPosition.getX()));

        // calculate the needed rotation-distance
        double distance = (Math.toDegrees(correctAngle) - Math.toDegrees(currentPosition.getDirection())) % 360;

        if (distance < -180) {
            distance += 360;
        } else if (distance > 179) {
            distance -= 360;
        }

        // debug output for angle calculation
                    /*
                    System.out.println("desired angle: " + Math.toDegrees(correctAngle));
                    System.out.println("needed rotation: " + distance);
                    System.out.println("robot position: " + agent.getCurrentPosition().getX() + " | " + agent.getCurrentPosition().getY());
                    System.out.println("current goal: " + n.getX() + " | " + n.getY());
                    System.out.println("----------");
                    */

        // check if needed angle is within allowed range (might depend on delay) and determine rotation direction.
        if (Math.abs(distance) >= ROTATIONERROR) {
            turning = true;
            moving = false;
            if (distance > 0) {
                rotationCoefficient = -1;
            } else {
                rotationCoefficient = 1;
            }
        } else {
            turning = false;
            moving = true;
            linearCoefficient = 1;
        }

        int x = (int) (currentPosition.getX());
        int y = (int) (currentPosition.getY());

        // while loop in order to .step() the path as long as the next nodes are too close.
        // TODO: maybe rework current tracing of path as it is not that 'Closed-loopish'
        // TODO: swap Y and X as soon as path returns X and Y
        while(Math.abs(currentPathPosition.getX() - y) <= (PROXIMITY) && Math.abs(x - currentPathPosition.getY()) <= (PROXIMITY)){
            handler.step();
            if ((alreadyTurning || alreadyMoving) && !(lastPosition.equals(getCurrentPosition()))){
                linearCoefficient = 0;
                rotationCoefficient = 0;
                alreadyTurning = false;
                alreadyMoving = false;
                stopped = true;
            }
            turning = false;
            moving = false;
            currentPathPosition = handler.getNode(handler.getIndex());
        }
    }
    
    public boolean doesNotMove(){
        return currentPosition.equals(lastPosition);
    }
    
    public boolean isAlreadyTurning(){
        return alreadyTurning;
    }
    
    public boolean isAlreadyMoving(){
        return alreadyMoving;
    }

    public boolean isMoving() {
        return moving;
    }

    public boolean isTurning() {
        return turning;
    }

    public double getRotationCoefficient(){
        return rotationCoefficient;
    }

    public double getLinearCoefficient(){
        return linearCoefficient;
    }

    public void setAlreadyTurning(boolean alreadyTurning){
        this.alreadyTurning = alreadyTurning;
    }

    public void setAlreadyMoving(boolean alreadyMoving){
        this.alreadyMoving = alreadyMoving;
    }

    public double getLastRotationCoefficient() {
        return lastRotationCoefficient;
    }

    public void setLastRotationCoefficient(double lastRotationCoefficient) {
        this.lastRotationCoefficient = lastRotationCoefficient;
    }

    public boolean isStopped() {
        return stopped;
    }

    public void setStopped(boolean stopped) {
        this.stopped = stopped;
    }
}
