package Model;

public class RoboPos{

    private double x;
    private double y;
    private double radius;
    private double direction;

    public RoboPos(double x, double y, double radius) {
        this.x = x;
        this.y = y;
        this.radius = radius;
        this.direction = 0;
    }

    public RoboPos(double x, double y, double radius, double direction) {
        this.x = x;
        this.y = y;
        this.radius = radius;
        this.direction = direction;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getRadius() {
        return this.radius;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public double getDirection() {
        return direction;
    }

    public void setDirection(double direction) {
        this.direction = direction;
    }

    public void setPosition(double x, double y){
        this.x = x;
        this.y = y;
    }

    /**
     * Method for calculating the angle (in radians) from the robot to the target. If everything is correct this method was adapted to deal with a coordinate system where the y axis is inverted.
     * @param target target node, needs to have an x and y coordinate
     * @return the angle from the robot to the target in an y-inverted coordinate system. 0 being "north", and pi/2 being "east".
     */
    public double getAngleTo(Node target){
        return Math.atan2(target.getX()-this.x, -1 * (target.getY()-this.y));
    }

    /**
     * Method for calculating the distance from the current position to a goal x,y.
     * @param targetX x-coordinate of the goal
     * @param targetY y-coordinate of the goal
     * @return euclidean distance to the target
     */
    public double getDistanceTo(double targetX, double targetY){
        return Math.sqrt((Math.pow((this.x - targetX),2) + Math.pow((this.y - targetY),2)));
    }
}