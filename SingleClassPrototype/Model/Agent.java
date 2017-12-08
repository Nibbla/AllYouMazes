package Model;

public class Agent {


    private RoboPos currentPosition;
    private RoboPos lastPosition = new RoboPos(0,0,0,0);
    private RoboPos estimatedPosition = new RoboPos(0,0,0,0);

    private TraversalHandler handler;
    private Node currentGoal;
    private double rotationCoefficient = 0, linearCoefficient = 0,prevLinearCoefficient = 0, prevRotationCoefficient = 0;

    private final double PROXIMITY = 25;
    private final double ROTATIONERROR = 5;

    // TODO: make use of ROS_ID in controller. this is to anticipate multiple epucks.
    private int ROS_ID;
    private boolean needsToTurn, canMove, isTurning, isMoving;

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

    public void update(RoboPos newPosition, RoboPos rotationPoint, double stepsize){
        needsToTurn = false;
        canMove = false;

        isTurning = false;
        isMoving = false;


        lastPosition.setPosition(currentPosition.getX(), currentPosition.getY());
        lastPosition.setRadius(currentPosition.getRadius());
        lastPosition.setDirection(currentPosition.getDirection());

        prevRotationCoefficient = rotationCoefficient;
        prevLinearCoefficient = linearCoefficient;

        currentPosition = newPosition;
        currentPosition.setDirection(rotationPoint.getAngleTo(new Node((int)(currentPosition.getX()), (int)(currentPosition.getY()))));

        determineChanges();

        Node currentPathPosition = handler.getLine(handler.getIndex()).getB();

        int x = (int) (currentPosition.getX());
        int y = (int) (currentPosition.getY());

        // while loop in order to .step() the path as long as the next nodes are too close.
        // TODO: maybe rework current tracing of path as it is not that 'Closed-loopish'
        // TODO: swap Y and X as soon as path returns X and Y
        while(Math.abs(currentPathPosition.getX() - y) <= (PROXIMITY) && Math.abs(x - currentPathPosition.getY()) <= (PROXIMITY)){

            handler.step();
            currentPathPosition = handler.getLine(handler.getIndex()).getB();
        }

        // TODO: swap Y and X as soon as path returns as X and Y
        double correctAngle = currentPosition.getAngleTo(new Node(currentPathPosition.getY(), currentPathPosition.getX()));

        // calculate the needed rotation-distance
        double distance = ((Math.toDegrees(correctAngle) % 360 ) - (Math.toDegrees(currentPosition.getDirection()) % 360));

        if (distance < -180) {
            distance += 360;
        } else if (distance > 179) {
            distance -= 360;
        }

        // debug output for angle calculation
        /*
        System.out.println("current angle: " + Math.toDegrees(this.getCurrentPosition().getDirection()));
        System.out.println("estimated angle: " + Math.toDegrees(this.estimatedPosition.getDirection()));
        System.out.println("desired angle: " + Math.toDegrees(correctAngle));
        System.out.println("needed rotation: " + distance);
        System.out.println("robot position: " + this.getCurrentPosition().getX() + " | " + this.getCurrentPosition().getY());
        System.out.println("current goal: " + currentPathPosition.getY() + " | " + currentPathPosition.getX());
        */

        // check if needed angle is within allowed range (might depend on delay) and determine rotation direction.
        if (Math.abs(distance) >= ROTATIONERROR) {
            needsToTurn = true;
            canMove = false;
            if (distance > 0) {
                rotationCoefficient = -1;
                linearCoefficient = 0;
            } else {
                rotationCoefficient = 1;
                linearCoefficient = 0;
            }
        } else {
            needsToTurn = false;
            canMove = true;
            rotationCoefficient = 0;
            linearCoefficient = 1;
        }

    }

    private void determineChanges() {
        if ((Math.abs(lastPosition.getX() - currentPosition.getX())) < 10 && Math.abs(lastPosition.getY() - currentPosition.getY()) < 10){
            isMoving = false;
        } else {
            isMoving = true;
        }
        if (Math.abs(lastPosition.getDirection() - currentPosition.getDirection()) < 0.02){
            isTurning = false;
        } else {
            isTurning = true;
        }
    }

    public double getRotationCoefficient(){
        return rotationCoefficient;
    }

    public double getLinearCoefficient(){
        return linearCoefficient;
    }

    public boolean canMove() {
        return canMove;
    }

    public boolean needsToTurn() {
        return needsToTurn;
    }

    public boolean isMoving() {
        return isMoving;
    }

    public boolean isTurning() {
        return isTurning;
    }

    public double getPrevRotationCoefficient() {
        return prevRotationCoefficient;
    }

    public double getPrevLinearCoefficient() {
        return prevLinearCoefficient;
    }

    public boolean isStuck(){
        return lastPosition.equals(currentPosition);
    }
}