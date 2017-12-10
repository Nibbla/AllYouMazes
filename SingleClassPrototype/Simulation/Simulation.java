package Simulation;

import Util.ImgWindow;
import Control.RobotControl;
import Interfaces.IControl;
import Model.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;

import java.util.LinkedList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/**
 * Created by Jyr on 11/20/2017.
 */

//Assuming continuous simulation for now
//Probably need to add discrete later
//Need some agent class, new Node() all the time is ugly
//ONE ROBOT!

public class Simulation {

    public final static int TIME_STEP = 70;
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
    private LinkedList<Line> shortestPath;

    private double lastSendLinearSpeed = 0;
    private double lastSentAngularSpeed = 0;

    private double LINEARSENSITIVITY = 0.1;
    private double ANGULARSENSITIVITY = 0.2;

    private ImgWindow pathWindow = ImgWindow.newWindow();

    /**
     * Method to create an initial scene (requires the robot to be detected, will fail otherwise)
     */
    public Simulation() {
		System.out.println("Init Camera...");
        cv.initCamera(400,600,1000,300);

        // current image recognition. to be replaced with data from BGS
	Mat currentFrame = cv.getFrame();
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

        Imgcodecs.imwrite("help.jpg", currentFrame);

        // scan contours of the maze
        contour = cv.getCurrentContours();

        // create shorted path based on contours (the underlaying method still has to pe improved)
        // TODO: currently the 'Nodes' returned in the ArrayList shortest-path have X and Y swapped. When change also adapt input parameters for angle calculations, see below.

		System.out.println("Calculate Path...");
        int stepsize = 4;
        grid = DijkstraPathFinder.retrieveDijcstraGrid(currentFrame, new MatOfPoint2f(contour.toArray()), 0,0, stepsize);
        shortestPath = DijkstraPathFinder.getShortestPathFromGridLine(grid,new RoboPos(rp.getY(), rp.getX(), 0,0),stepsize);
        shortestPath = DijkstraPathFinder.reverseLinkedListLine(shortestPath);  //to not mess with code. it should now be upside down, as the dijkstra starts from the goal and not the robot.
        //System.out.println(shortestPath.size());
        currentFrame = currentFrame.submat(cv.backgroundRect(currentFrame));
        for (Line no : shortestPath) {
            //System.out.println(no);
            Imgproc.line(currentFrame, new org.opencv.core.Point(no.getA().getY(), no.getA().getX()), new org.opencv.core.Point(no.getB().getY(), no.getB().getX()), new Scalar(255), 3);
        }
        pathWindow.setImage(currentFrame);
        Imgcodecs.imwrite("help2.jpg", currentFrame);
	cv.releaseFrame();
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
        //schedule();

		startSimulation();
    }

    private void connect() {
        control.startConnection();
    }



	private void startSimulation(){


		long start = System.currentTimeMillis();
		long end = System.currentTimeMillis();
	while(true){
		long diff = (end - start);
		
		try{
			if (diff < TIME_STEP){
				Thread.sleep((TIME_STEP - diff));
			}
		}catch (Exception e){
			System.out.println("unable to wait, wtf");
		}

		start = System.currentTimeMillis();
		end = 0;

		Mat test = cv.getFrame();

		end = System.currentTimeMillis();
        System.out.println("Until taking picture took " + (end-start) + " ms");

        cv.findRobotPosition();
        Point currentPosition = cv.getCenter();

		end = System.currentTimeMillis();
        System.out.println("Until determining Robot " + (end-start) + " ms");

        cv.findAnglePosition();
        Point anglePosition = cv.getAngle();
		
		end = System.currentTimeMillis();
        System.out.println("Until determining Rotation " + (end-start) + " ms");

		if (currentPosition == null) {
                    System.out.println("oh god no");
                    detected = false;
                } else {
                    detected = true;
                }

		System.out.println(detected);
		if (detected) {



                    boolean needToSend = false;

                    // extract the robots position and radius
                    int robotX = (int) (currentPosition.x);
                    int robotY = (int) (currentPosition.y);
                    int robotR = (int) (cv.getRadius()/2);


					try{
                    shortestPath = DijkstraPathFinder.getShortestPathFromGridLine(grid,new RoboPos(robotY, robotX, 0,0), 4);
                    shortestPath = DijkstraPathFinder.reverseLinkedListLine(shortestPath);
                    agent.getHandler().changePath(shortestPath, 0);
					}catch (Exception e){
					System.out.println("No path retrievable. Robot possibly within Contours");
					}

					end = System.currentTimeMillis();
        			System.out.println("Until changing path took " + (end-start) + " ms");

                    //System.out.println("updating agent");
                    agent.update(new RoboPos(robotX, robotY, robotR), new RoboPos((int)(anglePosition.x), (int)(anglePosition.y),0));
                    //System.out.println("done updating");
				

					end = System.currentTimeMillis();
        			System.out.println("Until updating robot took " + (end-start) + " ms");


		/*
                    Imgproc.circle(test, currentPosition, 5, new Scalar(255,0,0), 3);
                    Imgproc.circle(test, anglePosition, 5,new Scalar(255,0,0), 3);
                    Imgproc.line(test, currentPosition, anglePosition, new Scalar(255,0,0), 3);
                    Imgproc.circle(test, new Point(agent.getHandler().getLine(agent.getHandler().getIndex()).getA().getY(), agent.getHandler().getLine(agent.getHandler().getIndex()).getA().getX()), 5,new Scalar(255,255,255), 3);

                    pathWindow.setImage(test);
		*/
		    		cv.releaseFrame();
                    // for switching between moving/turning. A new command will only be sent in case there was no previous command sent or the robot is not moving/rotating (due to no command being sent. it happens).
                    if (agent.canMove() && lastSendLinearSpeed != agent.getLinearCoefficient()){
                        //System.out.println("needs to move");
                        needToSend = true;
                    } else if (agent.needsToTurn() && lastSentAngularSpeed != agent.getRotationCoefficient()){
                        //System.out.println("needs to turn");
                        needToSend = true;
                    }



                    if (needToSend){
                            System.out.println(agent.getLinearCoefficient()+ " | "+agent.getRotationCoefficient());
                            control.sendCommand(agent.getLinearCoefficient(),agent.getRotationCoefficient());
                            lastSendLinearSpeed = agent.getLinearCoefficient();
                            lastSentAngularSpeed = agent.getRotationCoefficient();
                      }

					end = System.currentTimeMillis();
        			System.out.println("Until sending command took " + (end-start) + " ms");
                }
				end = System.currentTimeMillis();
                System.out.println("Complete update took " + (end-start) + " ms");
                //System.out.println("------------------");




		}


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
		long start = System.currentTimeMillis();
                // set the current goal node. TODO: X and Y are swapped
                // n = agent.getHandler().getNode(agent.getHandler().getIndex());

                // TODO: check if the goal was reached
                //boolean finished = agent.getHandler().getIndex()+1 > agent.getHandler().length();

                // input from the Computervision
		cv.getFrame();
		cv.releaseFrame();
		Mat test = cv.getFrame();
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


                    shortestPath = DijkstraPathFinder.getShortestPathFromGridLine(grid,new RoboPos(robotY, robotX, 0,0), 4);
                    shortestPath = DijkstraPathFinder.reverseLinkedListLine(shortestPath);
                    agent.getHandler().changePath(shortestPath, 0);



                    //System.out.println("updating agent");
                    agent.update(new RoboPos(robotX, robotY, robotR), new RoboPos((int)(anglePosition.x), (int)(anglePosition.y),0));
                    //System.out.println("done updating");

		/*
                    Imgproc.circle(test, currentPosition, 5, new Scalar(255,0,0), 3);
                    Imgproc.circle(test, anglePosition, 5,new Scalar(255,0,0), 3);
                    Imgproc.line(test, currentPosition, anglePosition, new Scalar(255,0,0), 3);
                    Imgproc.circle(test, new Point(agent.getHandler().getLine(agent.getHandler().getIndex()).getA().getY(), agent.getHandler().getLine(agent.getHandler().getIndex()).getA().getX()), 5,new Scalar(255,255,255), 3);

                    pathWindow.setImage(test);
		*/
		    cv.releaseFrame();
                    // for switching between moving/turning. A new command will only be sent in case there was no previous command sent or the robot is not moving/rotating (due to no command being sent. it happens).
                    if (agent.canMove() && lastSendLinearSpeed != agent.getLinearCoefficient()){
                        //System.out.println("needs to move");
                        needToSend = true;
                    } else if (agent.needsToTurn() && lastSentAngularSpeed != agent.getRotationCoefficient()){
                        //System.out.println("needs to turn");
                        needToSend = true;
                    }



                    if (needToSend){
                            System.out.println(agent.getLinearCoefficient()+ " | "+agent.getRotationCoefficient());
                            control.sendCommand(agent.getLinearCoefficient(),agent.getRotationCoefficient());
                            lastSendLinearSpeed = agent.getLinearCoefficient();
                            lastSentAngularSpeed = agent.getRotationCoefficient();
                      }
                }
		long end = System.currentTimeMillis();
                System.out.println("Update took " + (end-start) + " ms");
                //System.out.println("------------------");

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
        //ImageRecognition ir = new ImageRecognition();
        //ir.loop();
    }

}
