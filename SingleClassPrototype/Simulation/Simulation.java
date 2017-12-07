package Simulation;

import Control.RobotControl;
import Interfaces.IControl;
import Model.*;
import Util.ImgWindow;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

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

    public final static int TIME_STEP = 333;
    private static final double ROTATIONERROR = 25;

    private Agent agent;
    private String pathToPicture;
    private MatOfPoint contour;
    private ScheduledExecutorService scheduler;
    private Node n;

    private boolean detected;
	private boolean prevStuck;


    private RoboPos lastPosition = new RoboPos(0,0,0,0);

    public IControl control;
    private static RobotControl factoryControl = new RobotControl();

    private ImageRecognition cv = new ImageRecognition();

	public static Node[][] grid;
	private LinkedList<Node> shortestPath;

	private double lastSendLinearSpeed = 0;
	private double lastSentAngularSpeed = 0;

	private double LINEARSENSITIVITY = 0.1;
	private double ANGULARSENSITIVITY = 0.05;

    /**
     * Method to create an initial scene (requires the robot to be detected, will fail otherwise)
     */
    public Simulation() {
        cv.initCamera(1000,1000,3000);

        // current image recognition. to be replaced with data from BGS

		cv.findRobotPosition();
        Point currentPosition = cv.getCenter();
        
        if (currentPosition == null) {
            System.out.println("oh god no");
            detected = false;
        } else {
            detected = true;
        }

        // extract robot position and radius from computervision
        int robotX = (int)(currentPosition.x);
        int robotY = (int)(currentPosition.y);
        int robotR = (int)(cv.getRadius()/2);

        // create RoboPos vaiable to be passed to the angent
        RoboPos rp = new RoboPos(robotX, robotY, robotR);

        cv.findAnglePosition();
        Point anglePosition = cv.getAngle();

        RoboPos ap = new RoboPos(anglePosition.x, anglePosition.y, 0);
        rp.setDirection(ap.getAngleTo(new Node((int)(rp.getX()), (int)(rp.getY()))));

        // scan contours of the maze
        contour = cv.getCurrentContours();

        // create shorted path based on contours (the underlaying method still has to pe improved)
        // TODO: currently the 'Nodes' returned in the ArrayList shortest-path have X and Y swapped. When change also adapt input parameters for angle calculations, see below.
        Mat currentFrame = cv.getFrame();

        int stepsize = 8;
        grid = DijkstraPathFinder.retrieveDijcstraGrid(currentFrame, new MatOfPoint2f(contour.toArray()), 0,0, stepsize);
        shortestPath = DijkstraPathFinder.getShortestPathFromGrid(grid,new RoboPos(rp.getY(), rp.getX(), 0,0),stepsize);
        shortestPath = DijkstraPathFinder.reverseLinkedList(shortestPath);  //to not mess with code. it should now be upside down, as the dijkstra starts from the goal and not the robot.
        for (Node no : shortestPath) {
            Imgproc.circle(currentFrame, new org.opencv.core.Point(no.getY(), no.getX()), 1, new Scalar(255), 1);
        }
        //ImgWindow.newWindow(currentFrame);


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

/*
        control.sendCommand(1,0);

		try{Thread.sleep(3000);}catch(Exception e){}
*/
        final Runnable robotDetector = new Runnable() {
            @Override
            public void run() {
                // set the current goal node. TODO: X and Y are swapped
                // n = agent.getHandler().getNode(agent.getHandler().getIndex());

                // TODO: check if the goal was reached
                //boolean finished = agent.getHandler().getIndex()+1 > agent.getHandler().length();


                // input from the Computervision
				cv.findRobotPosition();
                Point currentPosition = cv.getCenter();

                cv.findAnglePosition();
                Point anglePosition = cv.getAngle();


                if (currentPosition == null) {
                    System.out.println("oh god no");
                    detected = false;
                } else {
                    detected = true;
                }

                // perform the following steps if the robot was detectd. otherwise wait for next computervision-result
                if (detected) {
        			


                    boolean needToSend = false;

                    // extract the robots position and radius
                    int robotX = (int) (currentPosition.x);
                    int robotY = (int) (currentPosition.y);
                    int robotR = (int) (cv.getRadius()/2);

					//shortestPath = DijkstraPathFinder.getShortestPathFromGrid(grid,new RoboPos(robotY, robotX, 0,0),8);
        			//shortestPath = DijkstraPathFinder.reverseLinkedList(shortestPath);
					//agent.getHandler().changePath(shortestPath, 0);



                    agent.update(new RoboPos(robotX, robotY, robotR), new RoboPos((int)(anglePosition.x), (int)(anglePosition.y),0));

                    // for switching between moving/turning. A new command will only be sent in case there was no previous command sent or the robot is not moving/rotating (due to no command being sent. it happens).
                    if (agent.canMove() && !agent.isMoving()){
                        needToSend = true;
                    } else if (agent.needsToTurn() && (!agent.isTurning() || agent.getPrevRotationCoefficient() != agent.getRotationCoefficient())){
                        needToSend = true;
                    } 

                    double linearDifference = Math.abs(agent.getLinearCoefficient() - lastSendLinearSpeed);
                    double rotationDifference = Math.abs(agent.getRotationCoefficient() - lastSentAngularSpeed);

                    if ((linearDifference <= LINEARSENSITIVITY && linearDifference > 0) || (rotationDifference <= ANGULARSENSITIVITY && rotationDifference > 0)){
                        needToSend = false;
                    }

                    if (needToSend){
						if ((agent.getPrevRotationCoefficient() != agent.getRotationCoefficient()) || (agent.getPrevLinearCoefficient() != agent.getLinearCoefficient()) || (agent.isStuck() && !prevStuck)){
							System.out.println(agent.getLinearCoefficient()+ " | "+agent.getRotationCoefficient());
							lastSendLinearSpeed = agent.getLinearCoefficient();
							lastSentAngularSpeed = agent.getRotationCoefficient();
                        	control.sendCommand(agent.getLinearCoefficient(),agent.getRotationCoefficient());
							if (prevStuck){
								prevStuck = false;
							} else {
								prevStuck = true;
								}
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
        Simulation sim = new Simulation();
    }

}
