package Magic.golf;


import Game.Actors.Move;
import Game.Actors.Player;
import Game.Config;

import java.util.ArrayList;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Ball {



    private final Player player;

    public double radius = Config.getBallRadius();

    public double previousX, previousY, previousZ;



    public double x, y, z;
    public double aX, aY, aZ;
    private double[] surfaceX;
    private double[] surfaceY;
    private double[] surfaceZ;
    private double[] surfaceXBig;
    private double[] surfaceYBig;
    private double[] surfaceZBig;
    private int[][] surfaceColection;
    private int[][] surfaceColectionBig;

    public boolean isMoving = false;
    public boolean inHole = false;
    public boolean inPlay = true;

    private int stopcounter;



    private boolean pregame = true;

    private Ball(Player player){
        this.player = player;
    }

    public Player getPlayer() {
        return player;
    }

    public Ball clone(){
        Ball b = new Ball(this.player);
        b.x = x;
        b.y = y;
        b.z = z;
        b.previousX = previousX;
        b.previousY = previousY;
        b.previousZ = previousZ;
        b.aX = aX; b.aY = aY; b.aZ = aZ;
        b.surfaceX = surfaceX;
        b.surfaceY = surfaceY;
        b.surfaceZ = surfaceZ;

        b.surfaceXBig = surfaceXBig;
        b.surfaceYBig = surfaceYBig;
        b.surfaceZBig = surfaceZBig;

        b.isMoving = isMoving;
        b.inHole = inHole;
        b.inPlay = inPlay;
        b.pregame = pregame;
        b.stopcounter = stopcounter;

        b.surfaceColection = new int[surfaceX.length][3];
        b.surfaceColectionBig = new int[surfaceXBig.length][3];
        return b;
    }


    /**
     * constructor of the class Ball
     * @param radius the radius of the ball
     * @param p the owner of the ball
     */
    public Ball(double radius, Player p) {
        this.player = p;
        this.radius = radius;

        calculateSurfacePoints();
    }

    public boolean isPregame() {
        return pregame;
    }

    /**cant work. stupid paper
     * https://www.cmu.edu/biolphys/deserno/pdf/sphere_equi.pdf
     */
    private void calculateSurfacePointsWrong() {

        int N = Config.BALLRESOLUTION;
        ArrayList<Double> xses = new ArrayList<Double>((int) (N*1.3));
        ArrayList<Double> yses = new ArrayList<Double>((int)(N*1.3));
        ArrayList<Double> zses = new ArrayList<Double>((int)(N*1.3));
        ArrayList<Double> xses2 = new ArrayList<Double>((int) (N*1.3));
        ArrayList<Double> yses2 = new ArrayList<Double>((int)(N*1.3));
        ArrayList<Double> zses2 = new ArrayList<Double>((int)(N*1.3));


        int Ncount = 0;
        double a = (4*Math.PI*radius*radius)/N;
        double d = Math.sqrt(a);
        double mLamba = Math.round(d/Math.PI);
        double dLamba = Math.PI/mLamba;
        double dPhi = a/dLamba;
        for (int m = 0; m < mLamba; m++) {
            double lambda = Math.PI*(m+0.5)/mLamba;
            double mPhi = Math.round(2*Math.PI* sin(lambda/dPhi));
            for (int n = 0; n < mPhi; n++) {
                double phi = 2 * Math.PI * n/mPhi;
                double x = new Double(radius* sin(lambda)* cos(phi));
                double y = new Double(radius* sin(lambda)* sin(phi));
                double z = new Double(radius* cos(lambda));
                xses.add(x);
                yses.add(y);
                zses.add(z);
                xses2.add(x*1.2);
                yses2.add(y*1.2);
                zses2.add(z*1.2);
                Ncount++;
            }
        }
        System.out.println("Points on surface" +Ncount);
        surfaceX = new double[xses.size()];
        surfaceY = new double[yses.size()];
        surfaceZ = new double[zses.size()];
        surfaceXBig = new double[xses.size()];
        surfaceYBig = new double[yses.size()];
        surfaceZBig = new double[zses.size()];

        for (int i = 0; i < xses.size(); i++) {
            surfaceX[i] = xses.get(i);
            surfaceY[i] = yses.get(i);
            surfaceZ[i] = zses.get(i);
            surfaceXBig[i] = xses2.get(i);
            surfaceYBig[i] = yses2.get(i);
            surfaceZBig[i] = zses2.get(i);
        }

        surfaceColection = new int[xses.size()][3];
        surfaceColectionBig = new int[xses.size()][3];

    }

    private void calculateSurfacePoints() {

        int N = Config.BALLRESOLUTION;
        ArrayList<Double> xses = new ArrayList<Double>((int) (N*1.3));
        ArrayList<Double> yses = new ArrayList<>((int) (N * 1.3));
        ArrayList<Double> zses = new ArrayList<Double>((int)(N*1.3));
        ArrayList<Double> xses2 = new ArrayList<Double>((int) (N*1.3));
        ArrayList<Double> yses2 = new ArrayList<Double>((int)(N*1.3));
        ArrayList<Double> zses2 = new ArrayList<Double>((int)(N*1.3));


        int Ncount=0;
        for (double theta = 0; theta <= 2*Math.PI; theta=theta+ (2*Math.PI/16.)) {

            for (double arpha = 0; arpha <= Math.PI; arpha=arpha+(Math.PI/8.)) {


                double x = radius*cos(theta)*sin(arpha);
                double y = radius*sin(theta)*sin(arpha);
                double z = radius* cos(arpha);
                xses.add(x* Config.collitionSurfacePointRatio);
                yses.add(y* Config.collitionSurfacePointRatio);
                zses.add(z* Config.collitionSurfacePointRatio);
                xses2.add(x* Config.hoverSurfacePointRatio);
                yses2.add(y* Config.hoverSurfacePointRatio);
                zses2.add(z* Config.hoverSurfacePointRatio);
                Ncount++;
            }
        }
        System.out.println("Points on surface" +Ncount);
        surfaceX = new double[xses.size()];
        surfaceY = new double[yses.size()];
        surfaceZ = new double[zses.size()];
        surfaceXBig = new double[xses.size()];
        surfaceYBig = new double[yses.size()];
        surfaceZBig = new double[zses.size()];

        for (int i = 0; i < xses.size(); i++) {
            surfaceX[i] = xses.get(i);
            surfaceY[i] = yses.get(i);
            surfaceZ[i] = zses.get(i);
            surfaceXBig[i] = xses2.get(i);
            surfaceYBig[i] = yses2.get(i);
            surfaceZBig[i] = zses2.get(i);
        }

        surfaceColection = new int[xses.size()][3];
        surfaceColectionBig = new int[xses.size()][3];

    }

    /**
     * Checks if the ball is moving.
     * If ball is moving, return true
     * @return isMoving
     */
    public boolean isMoving() {
        return isMoving;
    }

    public double getPreviousX() {
        return previousX;
    }

    public double getPreviousY() {
        return previousY;
    }

    public double getPreviousZ() {
        return previousZ;
    }

    /**
     * Check if ball is shot
     */
    public double getSign(double number){
        if(number>=0)
            return 1;
        else{
            return -1;
        }
    }

    public void shootBall(double impulsX, double impulsY, double impulsZ) {
        if (!isMoving && inPlay()) {
            previousX-=impulsX* Config.STEPSIZE;
            previousY-=impulsY* Config.STEPSIZE;
            previousZ-=impulsZ* Config.STEPSIZE;
            isMoving = true;
        }
    }

    public void shootBall(double angleRadians, double power) {
        shootBall(Math.cos(angleRadians)*power,Math.sin(angleRadians)*power,0);
    }



    /**
     * ??
     * @return true
     */
    public boolean inPlay() {
        return inPlay;
    }

    public boolean isInHole() {
        return inHole;
    }

    public void setInHole(boolean inHole) {
        this.inHole = inHole;
        this.inPlay = !inHole;
    }

    /**
     * getter to get the speed in the X-axis
     * @return speedX
     */
    public double getSpeedX() {
        return this.x - this.previousX;
    }

    /**
     * getter to get the speed in the Y-axis
     * @return vY
     */
    public double getSpeedY() {
        return this.y - this.previousY;
    }

    /**
     * getter to get the speed in the Z-axis
     * @return speedZ
     */
    public double getSpeedZ() {
        return this.z - this.previousZ;
    }

    /**
     * getter to get the radius
     * @return radius
     */
    public double getRadius() {
        return radius;
    }


    /**
     * getter to get the coordinate of the ball of the Coordinate class
     * @return coordinate
     */
    public Coordinate getCoordinate() {
        return new Coordinate(x,y,z);
    }




    /**
     * check if ball is stopped, ball is stopped under following condition
     * if ball is stopped, isMoving is false
     */
    public boolean checkBallStopped(){
        if(Math.abs(getSpeedX())<=0.02 && Math.abs(getSpeedY())<=0.02 && Math.abs(getSpeedZ())<=0.01){

            stopcounter++;
            if (stopcounter>=10){
                isMoving=false;
                //System.out.println("ballStopped");
                previousX = x;
                previousY = y;
                previousZ = z;
                return true;
            }


        }else{
            stopcounter=0;
        }
        if(inHole) return true;
        isMoving=true;
        return false;

    }


    /**
     * method that prints the information about the ball
     */
    public void printBallInfo(){
/*
        System.out.println("Name:" + player.getName());
        System.out.println("X: " +x);
        System.out.println("Y: " +y);
        System.out.println("Z: " +z);
        System.out.println("previousX: " +previousX);
        System.out.println("previousY: " +previousY);
        System.out.println("previousZ: " +previousZ);

       // System.out.println("SpeedX: " + getSpeedX());
       // System.out.println("SpeedY: " + getSpeedY());
       // System.out.println("SpeedZ: " + getSpeedZ());
        System.out.println("ball radius: " + getRadius());
        if(!isMoving)
            System.out.println("ballStopped");
            */
    }

    /**
     * getter to get the speed of the ball
     * @return formula to get the speed (with Pythagoras)
     */
    public double getSpeed() {
       double speedX = getSpeedX();
        double speedY = getSpeedY();
        double speedZ = getSpeedZ();

        return Math.sqrt(speedX*speedX+speedY*speedY+speedZ*speedZ);
    }



    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }



    public double getaX() {
        return aX;
    }

    public void setaX(double aX) {
        this.aX = aX;
    }

    public double getaY() {
        return aY;
    }

    public void setaY(double aY) {
        this.aY = aY;
    }

    public double getaZ() {
        return aZ;
    }

    public void setaZ(double aZ) {
        this.aZ = aZ;
    }

    public int[][] getSurfacePoints() {
        for (int i = 0; i < surfaceX.length; i++) {
            surfaceColection[i][0] = (int) (surfaceX[i]+x);
            surfaceColection[i][1] = (int) (surfaceY[i]+y);
            surfaceColection[i][2] = (int) (surfaceZ[i]+z);

        }
        return surfaceColection;
    }

    public int[][] getSurfacePointsBig() {
        for (int i = 0; i < surfaceXBig.length; i++) {
            surfaceColectionBig[i][0] = (int) (surfaceXBig[i]+x);
            surfaceColectionBig[i][1] = (int) (surfaceYBig[i]+y);
            surfaceColectionBig[i][2] = (int) (surfaceZBig[i]+z);

        }
        return surfaceColectionBig;
    }

    public void setPreGame(boolean pregame) {
        this.pregame = pregame;
    }


    public void shootBall(Move move) {
        shootBall(move.getX(),move.getY(),move.getZ());
    }

}