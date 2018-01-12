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
    public final static boolean DEBUG_CV_ROBOT_ANGLE_DETECTION = false;

    public boolean debugEveryXFrames = true;
    public int debugFrames = 10;

    public final static int TIME_STEP = 70;

    public static Node[][] grid;
    private static RobotControl factoryControl = new RobotControl();
    public IControl control;
    private LinkedList<Line> shortestPath;
    private int stepsize = 4;
    private Agent agent;
    private MatOfPoint contour;

    private boolean detected;

    private ImageRecognition cv = new ImageRecognition(debugEveryXFrames);

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
        Mat currentFrame = cv.getSubFrame();


        // store current frame (e.g. for inspection)
        Imgcodecs.imwrite("currentInitialImage.jpg", currentFrame);

        // determine the current position of the robot
        cv.findRobotPosition(false);
        //cv.findObjectPostion(false);
        detected = cv.isRobotDetected();

        // extract robot position and radius from computervision
        // create RoboPos vaiable to be passed to the angent
        RoboPos rp = cv.getRoboPosFromCurrentPositionAndSetAngle(false);


        // scan contours of the maze
        contour = cv.getCurrentContours();

        pickWindow.setImage(currentFrame);

        while(!pickWindow.isClicked()){ }
        System.out.println("X: " + pickWindow.mouseX + " | Y: " + pickWindow.mouseY);

        // TODO: around here the contours should be displayed in a window as well, s.t. a goal position can be extracted via click and passed as goalX, goalY below. Note that they have to be scaled onto the 'stepsize' grid,

        // create shorted path based on contours (the underlaying method still has to pe improved)
        // TODO: currently the 'Nodes' returned in the ArrayList shortest-path have X and Y swapped. When change also adapt input parameters for angle calculations, see below.

        cv.findObjectPostion(false);
        setGridAndShortestPath(rp,currentFrame);

        // draw the path to the goal on the initial frame
        drawPathOnWindowAndStoreFrame(currentFrame);



        // free all memory used by CV as soon as all information (contours, position, rotation) is extracted
        cv.releaseFrame();

        // init a traversalHandler based on the shortest path, to be passed to the agent
        TraversalHandler traversalHandler = new TraversalHandler(shortestPath, new Node((int) rp.getX(), (int) rp.getY()));

        // create an agent with ROS_ID, roboPos and handler. TODO: implement ROS_ID in RobotControl to send commands to different robots
        this.agent = new Agent(0, rp, traversalHandler);


        this.control = factoryControl.getInstance();


        if (agent == null || contour == null || shortestPath == null) {
            System.out.println("null pointer constructing simulation");
            return;
        }

        // start connection to the epuck (init ROS)
        connect();

        // start controlling thread
        startSimulation();
    }

    private void drawPathOnWindowAndStoreFrame(Mat currentFrame) {
        for (Line no : shortestPath) {
            Imgproc.line(currentFrame, new org.opencv.core.Point(no.getA().getY(), no.getA().getX()), new org.opencv.core.Point(no.getB().getY(), no.getB().getX()), new Scalar(255), 3);
        }
        pathWindow = ImgWindow.newWindow();
        pathWindow.setImage(currentFrame);

        // store the edited frame (e.g for inspection)
        Imgcodecs.imwrite("editedInitialFrame.jpg", currentFrame);
    }

    private void setGridAndShortestPath(RoboPos rp, Mat currentFrame) {
        System.out.println("Calculate Path...");
        grid = DijkstraPathFinder.retrieveDijcstraGrid(currentFrame, new MatOfPoint2f(contour.toArray()), pickWindow.mouseX, pickWindow.mouseY, stepsize);
        shortestPath = DijkstraPathFinder.getShortestPathFromGridLine(grid, new RoboPos(rp.getY(), rp.getX(), 0, 0), stepsize);
        shortestPath = DijkstraPathFinder.reverseLinkedListLine(shortestPath);  //to not mess with code. it should now be upside down, as the dijkstra starts from the goal and not the robot.

    }


    private void connect() {
        control.startConnection();
    }

    /**
     * Method used to manage the execution of the Simulation.
     */
    private void startSimulation() {
        int counter = 0;
        boolean debug=false;

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

            if(counter % debugFrames == 0 && debugEveryXFrames){
                debug = true;
            }else debug = false;

            counter++;

            // calculate how long the thread needs to wait to reach the desired delay in execution-time.
            diff = (end - start);

            waitMinimumTime(diff);


            // used to measure execution time
            start = System.currentTimeMillis();
            // CV frame used during each simulation step
            currentFrame = cv.getFrame();

            end = outputPictureTakingTime(start,end);


            // find the position of the robot

            cv.findRobotPosition(debug);
            Point currentPosition = cv.getCenter();
            end = outputRobotDetectionTime(end);

            // find the angle-position of the robot
            cv.findAnglePosition(debug);
            Point anglePosition = cv.getAngle();
            end = outputRotationDetectionTime(end);

            cv.findObjectPostion(debug);


            // check if the robot has been found
            checkIfRobotHasBeenFoundAndSetDetected(currentPosition);



            if (!detected) { //robot has not been detected, start a new loop
                outputWholeLoopDuration(start);
                continue;
            }

            // if robot has been found, perform further steps

            boolean needToSend = false;

            // extract the robots position and radius
            int robotX = (int) (currentPosition.x);
            int robotY = (int) (currentPosition.y);
            int robotR = (int) (cv.getRadius() / 2);

            // retrieve the newest shortest path from the grid and pass it to the handler
            retrieveNewestShortestPath(robotX,robotY,0);


            end = outputChangingPathDuration(end);


            // update representation of the agent, new position, new rotation.
            agent.update(new RoboPos(robotX, robotY, robotR), new RoboPos((int) (anglePosition.x), (int) (anglePosition.y), 0));

            System.out.println(agent.getCurrentPosition());

            end = outputUpdateRobotDuration(end);



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

            sendCommands(needToSend);


            end = outputCommandSendingDuration(end);


            outputWholeLoopDuration(start);


        }
    }

    private void sendCommands(boolean needToSend) {
        if (needToSend) {

            if (DEBUG_CONTROLLER) {
                System.out.println("Sending command: " + agent.getLinearCoefficient() + " | " + agent.getRotationCoefficient());
            }

            // sending command and storing it for comparison in next frame
            control.sendCommand(agent.getLinearCoefficient(), agent.getRotationCoefficient());
            lastSendLinearSpeed = agent.getLinearCoefficient();
            lastSentAngularSpeed = agent.getRotationCoefficient();
        }
    }

    private long outputUpdateRobotDuration(long end) {
        if (DEBUG_DURATION) {
            System.out.println("Updating robot took" + (System.currentTimeMillis() - end) + " ms");
            end = System.currentTimeMillis();
        }
        return end;
    }

    private void retrieveNewestShortestPath(int robotX, int robotY, int robotR) {
        try {
            shortestPath = DijkstraPathFinder.getShortestPathFromGridLine(grid, new RoboPos(robotY, robotX, robotR, 0), stepsize);
            shortestPath = DijkstraPathFinder.reverseLinkedListLine(shortestPath);
            agent.getHandler().changePath(shortestPath, 0);
        } catch (Exception e) {
            System.out.println("No path retrievable. Robot possibly within Contours");
        }
    }

    private long outputChangingPathDuration(long end) {
        if (DEBUG_DURATION) {
            System.out.println("Changing path took " + (System.currentTimeMillis() - end) + " ms");
            end = System.currentTimeMillis();
        }
        return end;
    }

    private long outputCommandSendingDuration(long end) {
        if (DEBUG_DURATION) {
            System.out.println("Sending command took " + (System.currentTimeMillis() - end) + " ms");
            end = System.currentTimeMillis();
        }
        return end;
    }

    private void outputWholeLoopDuration(long start) {
        if (DEBUG_DURATION) {
            long end = System.currentTimeMillis();
            System.out.println("Complete update took " + (end - start) + " ms");
        }
    }

    private void checkIfRobotHasBeenFoundAndSetDetected(Point currentPosition) {
        if (currentPosition == null) {
            System.out.println("Robot position not found, will skip frame.");
            detected = false;
        } else {
            detected = true;
        }
    }

    private long outputRotationDetectionTime(long end) {
        if (DEBUG_DURATION) {
            System.out.println("Determining Rotation " + (System.currentTimeMillis() - end) + " ms");
            end = System.currentTimeMillis();
        }
        return end;
    }

    private long outputRobotDetectionTime(long end) {
        if (DEBUG_DURATION) {
            System.out.println("Determining Robot " + (System.currentTimeMillis() - end) + " ms");
            end = System.currentTimeMillis();
        }
        return end;
    }

    private long outputPictureTakingTime(long start, long end) {
        if (DEBUG_DURATION) {
            end = System.currentTimeMillis();
            System.out.println("Taking picture took " + (end - start) + " ms");
        }
        return end;
    }

    private void waitMinimumTime(long diff) {
        try {
            if (diff < TIME_STEP) {
                Thread.sleep((TIME_STEP - diff));
            }
        } catch (Exception e) {
            System.out.println("unable to wait");
        }
    }

    public static void main(String[] args) {
        Simulation sim = new Simulation();
    }


}
