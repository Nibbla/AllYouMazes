package Model;

public class RoboPos{

    private final double x;
    private final double y;

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    private final double r;


    public RoboPos(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public double getRadious() {
        return r;
    }

    /**
     * Method for calculating the angle (in radians) from the robot to the target. If everything is correct this method was adapted to deal with a coordinate system where the y axis is inverted.
     * @param target target node, needs to have an x and y coordinate
     * @return the angle from the robot to the target in an y-inverted coordinate system. 0 being "north", and pi/2 being "east".
     */
    public double getAngleTo(Node target){
        return Math.atan2(target.getX()-this.x, -1 * (target.getY()-this.y));
    }
}