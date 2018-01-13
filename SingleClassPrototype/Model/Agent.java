package Model;

public class Agent {


    private RoboPos currentPosition;
    private RoboPos lastPosition = new RoboPos(0,0,0,0);
    private TraversalHandler handler;
    private Node currentGoal;
    private Node currentGoalA;
    private Node currentGoalB;
    private Node prevGoalA;
    private Node prevGoalB;
    private double rotationCoefficient = 0, linearCoefficient = 0,prevLinearCoefficient = 0, prevRotationCoefficient = 0, lastSentDirection = 0, correctAngle = 0, distance = 0;

    private final double PROXIMITY = 25;


    private final double ROTATIONERROR = 50;
	private final double DELAYANGLE = 0;


    private final double V2_ROTATIONERROR = 10;
    private final double V2_DELAYANGLE = 15;

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

	public void setLastDirection(double dir){
		lastSentDirection = dir;
	}

    public void update(RoboPos newPosition, RoboPos rotationPoint){


        lastPosition.setPosition(currentPosition.getX(), currentPosition.getY());
        lastPosition.setRadius(currentPosition.getRadius());
        lastPosition.setDirection(currentPosition.getDirection());

		
        prevRotationCoefficient = rotationCoefficient;
        if (rotationCoefficient != 0){
            lastSentDirection = rotationCoefficient;
        }

		prevLinearCoefficient = linearCoefficient;

        currentPosition = newPosition;
        currentPosition.setDirection(rotationPoint.getAngleTo(new Node((int)(currentPosition.getX()), (int)(currentPosition.getY()))));


        currentGoal = handler.getLine(handler.getIndex()).getA();

        int x = (int) (currentPosition.getX());
        int y = (int) (currentPosition.getY());

        // while loop in order to .step() the path as long as the next nodes are too close.
        // TODO: maybe rework current tracing of path as it is not that 'Closed-loopish'
        // TODO: swap Y and X as soon as path returns X and Y
        while(Math.abs(currentGoal.getX() - y) <= (PROXIMITY) && Math.abs(x - currentGoal.getY()) <= (PROXIMITY)){

			handler.step();
            currentGoal = handler.getLine(handler.getIndex()).getA();
        }

        needsToTurn = false;
        canMove = false;
        isTurning = false;
        isMoving = false;

        determineChanges();

        // TODO: swap Y and X as soon as path returns as X and Y
        double correctAngle = currentPosition.getAngleTo(new Node(currentGoal.getY(), currentGoal.getX()));

        // calculate the needed rotation-distance
       double distance = ((Math.toDegrees(correctAngle) + (prevRotationCoefficient * DELAYANGLE)) - Math.toDegrees(currentPosition.getDirection()))%360;

        if (distance < -180) {
            distance += 360;
        } else if (distance > 179) {
            distance -= 360;
        }

     
        // debug output for angle calculation
                    /*
					System.out.println("current angle: " + Math.toDegrees(this.getCurrentPosition().getDirection()));
                    System.out.println("desired angle: " + Math.toDegrees(correctAngle));
                    System.out.println("needed rotation: " + distance);
                    System.out.println("robot position: " + this.getCurrentPosition().getX() + " | " + this.getCurrentPosition().getY());
                    System.out.println("current goal: " + currentGoal.getY() + " | " + currentGoal.getX());
                    System.out.println("----------");
                    */

        // check if needed angle is within allowed range (might depend on delay) and determine rotation direction.
		double tmpError = ROTATIONERROR;

		if (isMoving){
			tmpError = 40;
		}


        if (Math.abs(distance) >= tmpError) {
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

    public void updateV2(RoboPos newPosition, RoboPos rotationPoint){
        lastPosition.setPosition(currentPosition.getX(), currentPosition.getY());
        lastPosition.setRadius(currentPosition.getRadius());
        lastPosition.setDirection(currentPosition.getDirection());


        prevRotationCoefficient = rotationCoefficient;
        if (rotationCoefficient != 0){
            lastSentDirection = rotationCoefficient;
        }

        prevLinearCoefficient = linearCoefficient;

        currentPosition = newPosition;
        currentPosition.setDirection(rotationPoint.getAngleTo(new Node((int)(currentPosition.getX()), (int)(currentPosition.getY()))));

        prevGoalA = currentGoalA;
        prevGoalB = currentGoalB;

        currentGoalA = handler.getLine(handler.getIndex()).getA();
        currentGoalB = handler.getLine(handler.getIndex()).getB();

        int x = (int) (currentPosition.getX());
        int y = (int) (currentPosition.getY());

        // while loop in order to .step() the path as long as the next nodes are too close.
        // TODO: maybe rework current tracing of path as it is not that 'Closed-loopish'
        // TODO: swap Y and X as soon as path returns X and Y
        while(Math.abs(currentGoalA.getX() - y) <= (PROXIMITY) && Math.abs(x - currentGoalA.getY()) <= (PROXIMITY)){
            handler.step();
            currentGoalA = handler.getLine(handler.getIndex()).getA();
            currentGoalB = handler.getLine(handler.getIndex()).getB();
        }

        // THE INTERESTING PART:
        // canMove means that at last step we were aligned (meaning within V2_ROTATIONERROR) with the current B node (endpoint) of the line.
        // But we only enter this case if the B node is still the same.
        // This might get dangerous when the line is very long, as our "+- V2_ROTATIONERROR/2" might accumulate, so in case this is working consider shorter lines.
        //
        // If we have a new endpoint or were previously were not aligned we resume with the usual loop.
        //
        // During the else-statement the angle we're aiming at is depending on the previous - hopefully sent - rotation coefficient, meaning it will aim a the "angle +- V2_ROTATIONERROR/2" if we were previously moving straight, and otherwise will aim at "angle +-(depending on direction) V2_DELAYANGLE +- V2_ROTATIONERROR/2".
        //
        // It can be tried to change the if-clause, as well as the calculation in line 216 ( --> correctAngle = [...]), to use currentGoalA instead of B, but since A is changed more often than B I considered starting with the later one.
        // Otherwise for the if-clause it can be tried (with either A/B) to use Node.getDistanceTo(other) to compare currentGoalA/B and previousGoalA/B and see that they are within a certain range of each other, close enough to allow us to stay 'on-course'.
        if(canMove && prevGoalB.equals(currentGoalB)){
            isTurning = false;
            isMoving = false;

            determineChanges();

            rotationCoefficient = 0;
            linearCoefficient = 1;
        } else {
            needsToTurn = false;
            canMove = false;
            isTurning = false;
            isMoving = false;

            determineChanges();

            // TODO: swap Y and X as soon as path returns as X and Y
            correctAngle = currentPosition.getAngleTo(new Node(currentGoalB.getY(), currentGoalB.getX()));

            // calculate the needed rotation-distance
            distance = ((Math.toDegrees(correctAngle) + (prevRotationCoefficient * V2_DELAYANGLE)) - Math.toDegrees(currentPosition.getDirection()))%360;

            if (distance < -180) {
                distance += 360;
            } else if (distance > 179) {
                distance -= 360;
            }


            // check if needed angle is within allowed range and determine rotation direction.
            if (Math.abs(distance) >= V2_ROTATIONERROR) {
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

        // debug output for angle calculation
    /*
        System.out.println("current angle: " + Math.toDegrees(this.getCurrentPosition().getDirection()));
        System.out.println("desired angle: " + Math.toDegrees(correctAngle));
        System.out.println("needed rotation: " + distance);
        System.out.println("robot position: " + this.getCurrentPosition().getX() + " | " + this.getCurrentPosition().getY());
        System.out.println("current goal: " + currentGoalB.getY() + " | " + currentGoalB.getX());
        System.out.println("----------");
    */
    }

    private void determineChanges() {
        if (Math.abs(lastPosition.getX() - currentPosition.getX()) <= 2 && Math.abs(lastPosition.getY() - currentPosition.getY()) <= 2){
            isMoving = false;
        } else {
            isMoving = true;
        }
        if (Math.abs(lastPosition.getDirection() - currentPosition.getDirection()) <= 0.01){
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
