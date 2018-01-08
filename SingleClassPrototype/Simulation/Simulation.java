package Simulation;

import Control.RobotControl;
import Interfaces.IControl;
import Model.*;
import Util.ImgWindow;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.LinkedList;

/**
 * Created by Jyr on 11/20/2017.
 */

public class Simulation {

    public final static boolean DEBUG_DURATION = true;
    public final static boolean DEBUG_REAL_TIME_POSITION = false;
    public final static boolean DEBUG_CONTROLLER = true;
    public final static boolean DEBUG_CV_CONTOURS = false;
    public static boolean DEBUG_CV_ROBOT_ANGLE_DETECTION = false;

    public final static int TIME_STEP = 70;

    public static Node[][] grid;
    private static RobotControl factoryControl = new RobotControl();
    public IControl control;
    private LinkedList<Line> shortestPath;
    private int stepsize = 4;
    private Agent agent;
    private MatOfPoint contour;

    private boolean detected;

    private ImageRecognition cv = new ImageRecognition(true);

    private double lastSendLinearSpeed = 0;
    private double lastSentAngularSpeed = 0;    

    private ImgWindow pickWindow = ImgWindow.newWindow();


    private ImgWindow pathWindow;
    private ImgWindow debugWindow;

    /**
     * Method to create an initial scene (requires the robot to be detected, will fail otherwise)
     */
    public Simulation() {
        // One-time initialization of camera
        System.out.println("Init Camera...");
        cv.initCamera(400, 600, 1000, 300);

        // storing the current frame for later use
        Mat currentFrame = cv.getFrame();
        currentFrame = currentFrame.submat(Imgproc.boundingRect(cv.backgroundRect(currentFrame)));

        // store current frame (e.g. for inspection)
        Imgcodecs.imwrite("currentInitialImage.jpg", currentFrame);

        // determine the current position of the robot
        cv.findRobotPosition(false);
        Point currentPosition = cv.getCenter();

        if (currentPosition == null) {
            System.out.println("Robot not found in initial frame, program will crash.");
            detected = false;
        } else {
            detected = true;
        }

        // extract robot position and radius from computervision
        int robotX = (int) (currentPosition.x);
        int robotY = (int) (currentPosition.y);
        int robotR = (int) (cv.getRadius() / 2);

        // create RoboPos vaiable to be passed to the angent
        RoboPos rp = new RoboPos(robotX, robotY, robotR);

        // determine the current position of the angle and calculate rotation
        cv.findAnglePosition(false);
        Point anglePosition = cv.getAngle();
        RoboPos ap = new RoboPos(anglePosition.x, anglePosition.y, 0);
        rp.setDirection(ap.getAngleTo(new Node((int) (rp.getX()), (int) (rp.getY()))));

        // scan contours of the maze
        contour = cv.getCurrentContours();

	pickWindow.setImage(currentFrame);
	
	while(!pickWindow.isClicked()){
	    
	}
	System.out.println("X: " + pickWindow.mouseX + " | Y: " + pickWindow.mouseY);

        // TODO: around here the contours should be displayed in a window as well, s.t. a goal position can be extracted via click and passed as goalX, goalY below. Note that they have to be scaled onto the 'stepsize' grid,

        // create shorted path based on contours (the underlaying method still has to pe improved)
        // TODO: currently the 'Nodes' returned in the ArrayList shortest-path have X and Y swapped. When change also adapt input parameters for angle calculations, see below.

        System.out.println("Calculate Path...");
        grid = DijkstraPathFinder.retrieveDijcstraGrid(currentFrame, new MatOfPoint2f(contour.toArray()), pickWindow.mouseX, pickWindow.mouseY, stepsize);
        shortestPath = DijkstraPathFinder.getShortestPathFromGridLine(grid, new RoboPos(rp.getY(), rp.getX(), 0, 0), stepsize);
        shortestPath = DijkstraPathFinder.reverseLinkedListLine(shortestPath);  //to not mess with code. it should now be upside down, as the dijkstra starts from the goal and not the robot.

        // draw the path to the goal on the initial frame

        for (Line no : shortestPath) {
            Imgproc.line(currentFrame, new org.opencv.core.Point(no.getA().getY(), no.getA().getX()), new org.opencv.core.Point(no.getB().getY(), no.getB().getX()), new Scalar(255), 3);
        }
	pathWindow = ImgWindow.newWindow();
        pathWindow.setImage(currentFrame);

        // store the edited frame (e.g for inspection)
        Imgcodecs.imwrite("editedInitialFrame.jpg", currentFrame);

        // free all memory used by CV as soon as all information (contours, position, rotation) is extracted
        cv.releaseFrame();

        // init a traversalHandler based on the shortest path, to be passed to the agent
        TraversalHandler traversalHandler = new TraversalHandler(shortestPath, new Node((int) rp.getX(), (int) rp.getY()));

        // create an agent with ROS_ID, roboPos and handler. TODO: implement ROS_ID in RobotControl to send commands to different robots
        this.agent = new Agent(0, rp, traversalHandler);

        try {
            this.control = factoryControl.getInstance();
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (agent == null || contour == null || shortestPath == null)
            System.out.println("null pointer constructing simulation");

        // start connection to the epuck (init ROS)
        connect();

        // start controlling thread
        startSimulation();
    }

    private void connect() {
        control.startConnection();
    }

    /**
     * Method used to manage the execution of the Simulation.
     */
    private void startSimulation() {
		int counter = 0;
		boolean debug = false;

        if (DEBUG_REAL_TIME_POSITION) {
            debugWindow = ImgWindow.newWindow();
        }


        // Variables used for Timer (Scheduler)
        long start = System.currentTimeMillis();
        long end = System.currentTimeMillis();
        long diff = (end - start);

        // variable for current image
        Mat currentFrame;

        // TODO: Implement end-condition, e.g. check if the robot has reached the goal. This should be placed in the while loop instead of 'true'.
        while (true) {

			if(counter % 10 == 0){
				debug = true;
			}

            // calculate how long the thread needs to wait to reach the desired delay in execution-time.
            diff = (end - start);

            try {
                if (diff < TIME_STEP) {
                    Thread.sleep((TIME_STEP - diff));
                }
            } catch (Exception e) {
                System.out.println("unable to wait");
            }

            // used to measure execution time
            start = System.currentTimeMillis();

            // CV frame used during each simulation step
            currentFrame = cv.getFrame();

            if (DEBUG_DURATION) {
                end = System.currentTimeMillis();
                System.out.println("Taking picture took " + (end - start) + " ms");
            }

            // find the position of the robot
            cv.findRobotPosition(debug);
            Point currentPosition = cv.getCenter();

            if (DEBUG_DURATION) {
                System.out.println("Determining Robot " + (System.currentTimeMillis() - end) + " ms");
                end = System.currentTimeMillis();
            }

            // find the angle-position of the robot
            cv.findAnglePosition(debug);
            Point anglePosition = cv.getAngle();

            if (DEBUG_DURATION) {
                System.out.println("Determining Rotation " + (System.currentTimeMillis() - end) + " ms");
                end = System.currentTimeMillis();
            }

            // check if the robot has been found
            if (currentPosition == null) {
                System.out.println("Robot position not found, will skip frame.");
                detected = false;
            } else {
                detected = true;
            }

            // if robot has been found, perform further steps
            if (detected) {

                boolean needToSend = false;

                // extract the robots position and radius
                int robotX = (int) (currentPosition.x);
                int robotY = (int) (currentPosition.y);
                int robotR = (int) (cv.getRadius() / 2);

                // retrieve the newest shortest path from the grid and pass it to the handler
                try {
                    shortestPath = DijkstraPathFinder.getShortestPathFromGridLine(grid, new RoboPos(robotY, robotX, 0, 0), stepsize);
                    shortestPath = DijkstraPathFinder.reverseLinkedListLine(shortestPath);
                    agent.getHandler().changePath(shortestPath, 0);
                } catch (Exception e) {
                    System.out.println("No path retrievable. Robot possibly within Contours");
                }

                if (DEBUG_DURATION) {
                    System.out.println("Changing path took " + (System.currentTimeMillis() - end) + " ms");
                    end = System.currentTimeMillis();
                }

                // update representation of the agent, new position, new rotation.
                agent.update(new RoboPos(robotX, robotY, robotR), new RoboPos((int) (anglePosition.x), (int) (anglePosition.y), 0));
		
		System.out.println(agent.getCurrentPosition());

                if (DEBUG_DURATION) {
                    System.out.println("Updating robot took" + (System.currentTimeMillis() - end) + " ms");
                    end = System.currentTimeMillis();
                }


                if (DEBUG_REAL_TIME_POSITION) {
                    Imgproc.circle(currentFrame, currentPosition, 5, new Scalar(255, 0, 0), 3);
                    Imgproc.circle(currentFrame, anglePosition, 5, new Scalar(255, 0, 0), 3);
                    Imgproc.line(currentFrame, currentPosition, anglePosition, new Scalar(255, 0, 0), 3);
                    Imgproc.circle(currentFrame, new Point(agent.getHandler().getLine(agent.getHandler().getIndex()).getA().getY(), agent.getHandler().getLine(agent.getHandler().getIndex()).getA().getX()), 5, new Scalar(255, 255, 255), 3);

                    debugWindow.setImage(currentFrame);

                    System.out.println("Updating real-time debug window took " + (System.currentTimeMillis() - end) + " ms");
                    end = System.currentTimeMillis();
                }

                // release CV frame
                currentFrame.release();
                cv.releaseFrame();

                // for switching between moving/turning. A new command will only be sent in case there was no previous command sent or the robot is not moving/rotating (due to no command being sent. it happens).
                if (agent.canMove() && lastSendLinearSpeed != agent.getLinearCoefficient()) {

                    if (DEBUG_CONTROLLER) {
                        System.out.println("needs to move");
                    }

                    needToSend = true;
                } else if (agent.needsToTurn() && lastSentAngularSpeed != agent.getRotationCoefficient()) {

                    if (DEBUG_CONTROLLER) {
                        System.out.println("needs to turn");
                    }

                    needToSend = true;
                }


                if (needToSend) {
                    if (DEBUG_CONTROLLER) {
                        System.out.println("Sending command: " + agent.getLinearCoefficient() + " | " + agent.getRotationCoefficient());
                    }

                    // sending command and storing it for comparison in next frame
                    control.sendCommand(agent.getLinearCoefficient(), agent.getRotationCoefficient());
                    lastSendLinearSpeed = agent.getLinearCoefficient();
                    lastSentAngularSpeed = agent.getRotationCoefficient();
                }

                if (DEBUG_DURATION) {
                    System.out.println("Sending command took " + (System.currentTimeMillis() - end) + " ms");
                    end = System.currentTimeMillis();
                }
            }

            if (DEBUG_DURATION) {
                end = System.currentTimeMillis();
                System.out.println("Complete update took " + (end - start) + " ms");
            }

		debug = false;
        }
    }

    public static void main(String[] args) {
        Simulation sim = new Simulation();
    }
}
