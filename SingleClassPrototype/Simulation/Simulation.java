package Simulation;

import Control.RobotControl;
import Interfaces.IControl;
import Model.*;
import SpecialSettingsEtc.Settings;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import view.Camera;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.concurrent.*;

/**
 * Created by Jyr on 11/20/2017.
 */

//Assuming continuous simulation for now
//Probably need to add discrete later
//Need some agent class, new Node() all the time is ugly
//ONE ROBOT!

public class Simulation {

    public final static int TIME_STEP = 500;
    private static final double ROTATIONERROR = 25;

    private Agent agent;
    private String pathToPicture;
    private MatOfPoint contour;
    private ScheduledExecutorService scheduler;
    private Node n;
    private boolean turning;
    private boolean moving;
    private boolean detected;
    private boolean alreadyTurning;
    private double lastRotationCoefficient = 0;
    private boolean alreadyMoving;
    private RoboPos lastPosition = new RoboPos(0,0,0,0);

    public IControl control;
    private static RobotControl factoryControl = new RobotControl();

    /**
     * Method to create an initial scene (requires the robot to be detected, will fail otherwise)
     * @param picture the string path to the input picture. Will probably be removed due to BGS-Computervision
     */
    public Simulation(String picture) {
        this.pathToPicture = picture;

        // previous approach to start a raspistill process in order to caprute images every 0.5s. NOTE: this is image is still not 'scaled down' (this can be achieved by halving the pixel values).
        /*
        Camera camera = new Camera();
        camera.startCamera(60, 2, 1300, 2000, 75, (Settings.getInputPath()), 0.075, 0.1, 0.8, 0.8);


        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
         */

        // current image recognition. to be replaced with data from BGS
        Mat gray = ComputerVision.grayScale(ComputerVision.resize(picture));
        KeyPoint[] kps = ComputerVision.robotv2(ComputerVision.resize(pathToPicture), 0, 0);
        
        if (kps[0] == null || kps[1] == null) {
            System.out.println("oh god no");
            detected = false;
        } else {
            detected = true;
        }

        // extract robot position and radius from computervision
        int robotX = (int)kps[0].pt.x;
        int robotY = (int)kps[0].pt.y;
        int robotR = (int)kps[0].size/2;

        // create RoboPos vaiable to be passed to the angent
        RoboPos rp = new RoboPos(robotX, robotY, robotR);
        rp.setDirection(rp.getAngleTo(new Node((int)kps[1].pt.x, (int)kps[1].pt.y)));

        // scan contours of the maze
        this.contour = ComputerVision.retrieveContour(gray, rp);

        // create shorted path based on contours (the underlaying method still has to pe improved)
        // TODO: currently the 'Nodes' returned in the ArrayList shortest-path have X and Y swapped. When change also adapt input parameters for angle calculations, see below.
        LinkedList<Node> shortestPath = ComputerVision.retrievePath(gray, new MatOfPoint2f(contour.toArray()), rp, 8);

        // init a traversalHandler based on the shortest path, to be passed to the agent
        TraversalHandler traversalHandler = new TraversalHandler(shortestPath, new Node((int) rp.getX(), (int) rp.getY()));

        // create an agent with ROS_ID, roboPos and handler. TODO: implement ROS_ID in RobotControl
        this.agent = new Agent(0,rp, traversalHandler);

        try{
            this.control = factoryControl.getInstance();
        } catch(Exception e){
            e.printStackTrace();
        }

        if (agent == null || contour == null || shortestPath == null) System.out.println("null pointer constructing simulation");

        // start connection to the epuck (init ROS)
        connect();

        // start controlling thread
        schedule();
    }

    private void connect() {
        control.startConnection();
    }

    private void schedule() {
        //scheduler = Executors.newScheduledThreadPool(1);
        scheduler =  Executors.newSingleThreadScheduledExecutor();

        final Runnable robotDetector = new Runnable() {
            @Override
            public void run() {
                // set the current goal node. TODO: X and Y are swapped
                n = agent.getHandler().getNode(agent.getHandler().getIndex());

                // TODO: check if the gal was reached
                //boolean finished = agent.getHandler().getIndex()+1 > agent.getHandler().length();

                // these values well be adapted depending on the current position and goal of the robot.
                double rotationCoefficient = 0;
                double linearCoefficient = 0;

                // input from the Computervision
                KeyPoint[] kps = null;

                // change method call depending on the last known robot position
                if (detected) {
                    kps = ComputerVision.robotv2(ComputerVision.resize(pathToPicture), (int)agent.getCurrentPosition().getX(), (int)agent.getCurrentPosition().getY(), (int)(agent.getCurrentPosition().getRadius()));
                } else {
                    kps = ComputerVision.robotv2(ComputerVision.resize(pathToPicture), 0, 0);
                }


                if (kps[0] == null || kps[1] == null) {
                    System.out.println("oh god no");
                    detected = false;
                } else {
                    detected = true;
                }

                // perform the following steps if the robot was detectd. otherwise wait for next computervision-result
                if (detected) {

                    // extract the robots position and radius
                    int robotX = (int) kps[0].pt.x;
                    int robotY = (int) kps[0].pt.y;
                    int robotR = (int) kps[0].size / 2;

                    // store the current values of the robot for 'stuff'
                    lastPosition.setPosition(agent.getCurrentPosition().getX(), agent.getCurrentPosition().getY());
                    lastPosition.setRadius(agent.getCurrentPosition().getRadius());
                    lastPosition.setDirection(agent.getCurrentPosition().getDirection());

                    // update the fields in the agent with the new values
                    agent.getCurrentPosition().setPosition(robotX, robotY);
                    agent.getCurrentPosition().setRadius(robotR);
                    agent.getCurrentPosition().setDirection(agent.getCurrentPosition().getAngleTo(new Node((int) kps[1].pt.x, (int) kps[1].pt.y)));
                    // TODO: maybe use different approach for calculating the rotation
                    // agent.getCurrentPosition().setDirection(lastPosition.getAngleTo(new Node((int) agent.getCurrentPosition().getX(),(int) agent.getCurrentPosition().getY())));

                    // calculate the angle to the current goal. TODO: revert X,Y swap in method call.
                    double correctAngle = agent.getCurrentPosition().getAngleTo(new Node(agent.getHandler().getNode(agent.getHandler().getIndex() + 1).getY(), agent.getHandler().getNode(agent.getHandler().getIndex() + 1).getX()));

                    // calculate the needed rotation-distance
                    double distance = (Math.toDegrees(correctAngle) - Math.toDegrees(agent.getCurrentPosition().getDirection())) % 360;

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



                    int x = (int) (agent.getCurrentPosition().getX());
                    int y = (int) (agent.getCurrentPosition().getY());

                    // while loop in order to .step() the path as long as the next nodes are too close.
                    // TODO: maybe rework current tracing of path as it is not that 'Closed-loopish'
                    while(Math.abs(n.getX() - y) <= (ComputerVision.PROXIMITY * ComputerVision.STEP_SIZE) && Math.abs(x - n.getY()) <= (ComputerVision.PROXIMITY * ComputerVision.STEP_SIZE)){
                        agent.getHandler().step();
                        if ((alreadyTurning || alreadyMoving) && !(lastPosition.equals(agent.getCurrentPosition()))){
                            control.sendCommand(0, 0);
                            alreadyTurning = false;
                            alreadyMoving = false;
                        }
                        turning = false;
                        moving = false;
                        n = agent.getHandler().getNode(agent.getHandler().getIndex());
                    }


                    // for switching between moving/turning. A new command will only be sent in case there was no previous command sent or the robot is not moving/rotating (due to no command being sent. it happens).
                    if (turning && !moving) {
                        if (!alreadyTurning || (rotationCoefficient != lastRotationCoefficient || lastRotationCoefficient == 0) || (lastPosition.equals(agent.getCurrentPosition()))) {
                            control.sendCommand(linearCoefficient, rotationCoefficient);
                            alreadyTurning = true;
                            alreadyMoving = false;
                            lastRotationCoefficient = rotationCoefficient;
                        }
                    } else if (moving && !turning) {
                        if (!alreadyMoving  || (lastPosition.equals(agent.getCurrentPosition()))) {
                            control.sendCommand(linearCoefficient, rotationCoefficient);
                            alreadyTurning = false;
                            alreadyMoving = true;
                        }
                    }
                }
            }
        };

        final ScheduledFuture<?> robotDetectorHandle = scheduler.scheduleAtFixedRate(robotDetector, 500, Simulation.TIME_STEP, TimeUnit.MILLISECONDS);
    }

    private void finish() {
        scheduler.shutdown();
        System.exit(0);
    }

    public static void main(String[] args) {
        Simulation sim = new Simulation(Settings.getInputPath());
    }

}
