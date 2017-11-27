package Simulation;

import Control.RobotControl;
import Interfaces.IControl;
import Model.*;
import SpecialSettingsEtc.Settings;
import Util.ImgWindow;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import sun.rmi.server.Util;

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

    private boolean detected;


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
        Mat backgroundImage = ComputerVision.readImg(picture);
        Mat grayBackground = ComputerVision.grayScale(ComputerVision.resize(backgroundImage));
        Mat newImage = ComputerVision.readImg(picture);
        Mat grayNew = ComputerVision.grayScale(ComputerVision.resize(backgroundImage));
        KeyPoint[] kps = ComputerVision.robotv2(ComputerVision.resize(backgroundImage), ComputerVision.resize(newImage), 0, 0);
        
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
        this.contour = ComputerVision.retrieveContour(grayBackground, rp);

        // create shorted path based on contours (the underlaying method still has to pe improved)
        // TODO: currently the 'Nodes' returned in the ArrayList shortest-path have X and Y swapped. When change also adapt input parameters for angle calculations, see below.
        int stepsize = 8;
        Node[][] grid = DijkstraPathFinder.retrieveDijcstraGrid(grayBackground, new MatOfPoint2f(contour.toArray()), 0,0, stepsize);
        LinkedList<Node> shortestPath = DijkstraPathFinder.getShortestPathFromGrid(grid,rp,stepsize);
        shortestPath = DijkstraPathFinder.reverseLinkedList(shortestPath);  //to not mess with code. it should now be upside down, as the dijkstra starts from the goal and not the robot.
        for (Node no : shortestPath) {
            Imgproc.circle(grayBackground, new org.opencv.core.Point(no.getY(), no.getX()), 1, new Scalar(255), 1);
        }
        ImgWindow.newWindow(grayBackground);


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

                // TODO: check if the goal was reached
                //boolean finished = agent.getHandler().getIndex()+1 > agent.getHandler().length();


                // input from the Computervision
                KeyPoint[] kps = null;

                // change method call depending on the last known robot position
                // TODO: adapt to BGS method, this is only improvised to make Simulation compilable
                Mat backgroundImage = ComputerVision.readImg(pathToPicture);
                Mat newImage = ComputerVision.readImg(pathToPicture);
                if (detected) {
                    kps = ComputerVision.robotv2(ComputerVision.resize(backgroundImage), (int)agent.getCurrentPosition().getX(), (int)agent.getCurrentPosition().getY(), (int)(agent.getCurrentPosition().getRadius()));
                } else {
                    kps = ComputerVision.robotv2(ComputerVision.resize(backgroundImage),ComputerVision.resize(newImage), 0, 0);
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

                    agent.update(new RoboPos(robotX, robotY, robotR), new Node((int) (kps[1].pt.x), (int) (kps[1].pt.y)));

                    // for switching between moving/turning. A new command will only be sent in case there was no previous command sent or the robot is not moving/rotating (due to no command being sent. it happens).
                    if (agent.isTurning() && !agent.isMoving()) {
                        if (!agent.isAlreadyTurning() || (agent.getRotationCoefficient() != agent.getLastRotationCoefficient() || agent.getLastRotationCoefficient() == 0) || agent.doesNotMove()) {
                            control.sendCommand(agent.getLinearCoefficient(), agent.getRotationCoefficient());
                            agent.setAlreadyTurning(true);
                            agent.setAlreadyMoving(false);
                            agent.setLastRotationCoefficient(agent.getRotationCoefficient());
                        }
                    } else if (agent.isMoving() && !agent.isTurning()) {
                        if (!agent.isAlreadyMoving()  || (lastPosition.equals(agent.getCurrentPosition())) || agent.doesNotMove()) {
                            control.sendCommand(agent.getLinearCoefficient(), agent.getRotationCoefficient());
                            agent.setAlreadyTurning(false);
                            agent.setAlreadyMoving(true);
                        }
                    } else if (agent.isStopped()){
                        control.sendCommand(agent.getLinearCoefficient(), agent.getRotationCoefficient());
                        agent.setStopped(false);
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
