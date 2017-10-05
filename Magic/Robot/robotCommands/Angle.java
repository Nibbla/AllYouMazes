package Magic.Robot.robotCommands;

/**
 * Represent an angle in degrees
 *
 * Created by nik on 03/03/17.
 */
public class Angle
{
    private double angle;

    public Angle()
    {
        angle = 0;
    }

    public Angle(int degrees)
    {
        angle = degrees;
    }

    public Angle(double degrees)
    {
        angle = degrees;
    }

    public Angle(Direction direction)
    {
        rotateTo(direction);
    }

    /**
     * the angle between two agents
     * @param first
     * @param second
     */
    public Angle(AbstractAgent first, AbstractAgent second) {
        double deltaX =  second.getLocation().getX() - first.getLocation().getX();
        double deltaY =  second.getLocation().getY() - first.getLocation().getY();

        angle = Math.toDegrees(Math.atan2(deltaY,deltaX));

    }

    public Angle clone(){
        return new Angle(angle);
    }
    public double getAngle()
    {
        return angle;
    }

    public double getRadians()
    {
        return toRadians(angle);
    }

    public static double toRadians(double degree)
    {
        return (degree % 360) * (Math.PI / 180);
    }

    public void rotate(int degree)
    {
        angle = (angle + degree) % 360;
    }

    public void rotateTo(Direction direction)
    {
        switch(direction)
        {
            case Direction.EAST:
                angle = 0;
                break;
            case Direction.NORTH_EAST:
                angle = 45;
                break;
            case Direction.NORTH:
                angle = 90;
                break;
            case Direction.NORTH_WEST:
                angle = 135;
                break;
            case Direction.WEST:
                angle = 180;
                break;
            case Direction.SOUTH_WEST:
                angle = 225;
                break;
            case Direction.SOUTH:
                angle = 270;
                break;
            case Direction.SOUTH_EAST:
                angle = 315;
                break;
        }
    }

    public double distance(Angle otherAngle)
    {
        return Math.abs(angle  - otherAngle.getAngle());
    }

    public double distance(int otherAngle)
    {
        return Math.abs(angle - (otherAngle % 360));
    }

    public double distance(double otherAngle)
    {
        return Math.abs(angle - (otherAngle % 360));
    }

    @Override
    public String toString() {
        return Double.toString(angle);
    }
}


