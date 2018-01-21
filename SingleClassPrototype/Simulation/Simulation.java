package Simulation;

import Control.RobotControl;
import Interfaces.IControl;
import Model.*;
import Util.ImgWindow;
import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import javax.swing.*;
import java.awt.*;
import java.io.File;
import java.util.*;

/**
 * Created by Jyr on 11/20/2017.
 */

public class Simulation {

    public final static boolean DEBUG_DURATION = false;
    public final static boolean DEBUG_REAL_TIME_POSITION = false;
    public final static boolean DEBUG_CONTROLLER = false;
    public final static boolean DEBUG_CV_CONTOURS = false;
    public final static boolean DEBUG_CV_ROBOT_ANGLE_DETECTION = false;
    public final static boolean DEBUG_CV_OBJECT = false;
    public final static boolean DEBUG_SHOW_GRID = false;
    public final static boolean DEBUG_PRINTOUT_PATH = false;
    public final static boolean DEBUG_STORE_EDITEDFRAME = false;
    public final static boolean DEBUG_ALLOWPATHWINDOWTOBEREDRAWN = true;

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
    private boolean byPassCamera = true; //set this to true in case you rather have different images selected
                                           //then using the camera. still needs open cv installed though.
    private boolean byPassObject = true;

    private double lastSendLinearSpeed = 0;
    private double lastSentAngularSpeed = 0;

    //private ImgWindow pathWindow = ImgWindow.newWindow();


    private ImgWindow pathWindow;
    private ImgWindow debugWindow;
    private int objectRadius = 20;
    private int goalX = 0;
    private int goalY = 0;
    private ImgWindow gridWindow;
    private double mouthaX;
    private double mouthaY;
    private double mouthbX;
    private double mouthbY;
    private double mouthcX;
    private double mouthcY;
    private double mouthdX;
    private double mouthdY;
    private Node[][] gridToRobotInvertingNeeded;
    private Point gridToRobotNoInvertingNeededObjectPosition;
    private double minimalAbstand = 25;
    private double minimalAbstandQuadrat = minimalAbstand*minimalAbstand;


    private ArrayList<java.awt.Point> possiblePickUpPoints;

    private Point[] optionalTabooAreaCenter = new Point[1];
    private double[] optionalTabooAreaRadiusSquared = new double[1];
    private Rect[] optionalTabooArea = new Rect[1];
    private double tabooRadius;
    private LinkedList<Line> shortestPathFromObject;
    private boolean goalChanged =false;
    private double maxRadius;
    private int latestPathWindowMouseX;
    private int latestPathWindowMouseY;
    private java.awt.Point currentSelectedPickUpPointStepsized;
    private double distanceObjectToPickup;
    private boolean allowFullStopRobot = false;



    /**
     * Method to create an initial scene (requires the robot to be detected, will fail otherwise)
     */
    public Simulation() {
        // One-time initialization of camera
        System.out.println("Init Camera...");
        if (byPassCamera){
            File[] files = getCammeraByPassImages();
            cv.setByPass(byPassCamera,files);

        }else {
            cv.initCamera(600, 400, 1000, 300);
        }


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
        pathWindow = ImgWindow.newWindow();
        pathWindow.setImage(currentFrame);

        while(!pathWindow.isClicked()){ }
        System.out.println("X: " + pathWindow.mouseX + " | Y: " + pathWindow.mouseY);

        // TODO: around here the contours should be displayed in a window as well, s.t. a goal position can be extracted via click and passed as goalX, goalY below. Note that they have to be scaled onto the 'stepsize' grid,

        // create shorted path based on contours (the underlaying method still has to pe improved)
        // TODO: currently the 'Nodes' returned in the ArrayList shortest-path have X and Y swapped. When change also adapt input parameters for angle calculations, see below.

        //cv.findObjectPostion(false);
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

    private File[] getCammeraByPassImages() {
        JFileChooser chooser = new JFileChooser();
        chooser.setMultiSelectionEnabled(true);
        chooser.showOpenDialog(null);
        return chooser.getSelectedFiles();
    }

    private void drawPathOnWindowAndStoreFrame(Mat currentFrame) {


        for (Line no : shortestPath) {
            Imgproc.line(currentFrame, new org.opencv.core.Point(no.getA().getY(), no.getA().getX()), new org.opencv.core.Point(no.getB().getY(), no.getB().getX()), new Scalar(255), 3);
        }
        if (pathWindow == null) pathWindow = ImgWindow.newWindow();
        Imgproc.circle( currentFrame, cv.getAnglePoint(), 5, new Scalar( 23, 65, 255 ), 2 );
        Imgproc.line(currentFrame, new org.opencv.core.Point(mouthbX, mouthbY), new org.opencv.core.Point(mouthaX, mouthaY), new Scalar(128), 2);
        Imgproc.line(currentFrame, new org.opencv.core.Point(mouthbX, mouthbY), new org.opencv.core.Point(mouthcX, mouthcY), new Scalar(128), 2);
        Imgproc.line(currentFrame, new org.opencv.core.Point(mouthaX, mouthaY), new org.opencv.core.Point(mouthdX, mouthdY), new Scalar(128), 2);

        if ( latestPathWindowMouseX != pathWindow.mouseX &&  latestPathWindowMouseY != pathWindow.mouseY){
            latestPathWindowMouseX = pathWindow.mouseX;
            latestPathWindowMouseY = pathWindow.mouseY;
            System.out.println("angle" + cv.getAnglePoint());

        if (!distanceAndAngleToObjectIsWrong(new Point(pathWindow.mouseX,pathWindow.mouseY))){
            System.out.println(pathWindow.mouseX + " " + pathWindow.mouseY + " are within robot mouth");
        }else{
            System.out.println(pathWindow.mouseX + " " + pathWindow.mouseY + " not within robot mouth");
        }

        }
        if (byPassObject) {
            Point object = pathWindow.getByPassObject();
            if (object!=null) Imgproc.circle( currentFrame, object, 5, new Scalar( 213, 125, 35 ), 2 );
        }
        pathWindow.setImage(currentFrame);

        // store the edited frame (e.g for inspection)
        if (DEBUG_STORE_EDITEDFRAME)Imgcodecs.imwrite("editedInitialFrame.jpg", currentFrame);
    }

    private void setGridAndShortestPath(RoboPos rp, Mat currentFrame) {
        System.out.println("Calculate Path...");
        goalX = pathWindow.mouseX;
        goalY = pathWindow.mouseY;
        grid = DijkstraPathFinder.retrieveDijcstraGrid(currentFrame, new MatOfPoint2f(contour.toArray()), goalX, goalY, stepsize, false, null, null, null);
        setShortestPathToGeal(rp, false);





    }

    private void setShortestPathToGeal(RoboPos rp, boolean checkIfInsideTaboo) {
        if (grid[(int) (rp.getY()/stepsize)][(int) ( rp.getX()/stepsize)] != null){
            shortestPath = DijkstraPathFinder.getShortestPathFromGridLine(grid, new RoboPos(rp.getY(), rp.getX(), 0, 0), stepsize);

        }else {
            // createTabooAreaFromObject(object,optionalTabooArea,optionalTabooAreaCenter,optionalTabooAreaRadiusSquared);
            tabooRadius = (cv.getRadius()+objectRadius)*1.1/ stepsize;
            java.awt.Point p = getAlternativeObjectPoint(cv.getCenter(),checkIfInsideTaboo);
            shortestPath = grid[p.y][p.x].getShortestPathLines();
        }

        shortestPath = DijkstraPathFinder.reverseLinkedListLine(shortestPath);
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
        while (agent.getHandler().getState() != TraversalStatus.FINISH) {

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
            // check if the robot has been found

            checkIfRobotHasBeenFoundAndSetDetected(currentPosition);
            if (!detected) { //robot has not been detected, start a new loop
                end = System.currentTimeMillis();
                outputWholeLoopDuration(start);
                continue;
            }
            // find the angle-position of the robot
            cv.findAnglePosition(debug);
            Point anglePosition = cv.getAnglePoint();
            end = outputRotationDetectionTime(end);

            cv.findObjectPostion(debug);








            // if robot has been found, perform further steps

            boolean needToSend = false;

            // extract the robots position and radius
            int robotX = (int) (currentPosition.x);
            int robotY = (int) (currentPosition.y);
            int robotR = (int) (cv.getRadius() / 2);

            // retrieve the newest shortest path from the grid and pass it to the handler
            //retrieveNewestShortestPath(robotX,robotY,0);
            checkIfGoalChangedAndSetGridNew(currentFrame);

            Point object;
            if (byPassObject){
                object = pathWindow.getByPassObject();
            }else {
                object = cv.getObject();
            }

            if (object!=null) {
                retrieveObjectShortestPath(object,currentFrame, robotX, robotY, 0);
            }else{
                setShortestPathToGeal(new RoboPos(robotX,robotY,robotR), false);
                agent.getHandler().changePath(shortestPath, 0);
            }

            end = outputChangingPathDuration(end);
            if (DEBUG_ALLOWPATHWINDOWTOBEREDRAWN) drawPathOnWindowAndStoreFrame(currentFrame);



            // update representation of the agent, new position, new rotation.
            agent.updateV2(new RoboPos(robotX, robotY, robotR), new RoboPos((int) (anglePosition.x), (int) (anglePosition.y), 0));
            if (debug) {
// System.out.println(agent.getCurrentPosition());
            }


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
            } else if (agent.getHandler().getState() == TraversalStatus.FINISH){

                if (DEBUG_CONTROLLER) {
                    System.out.println("goal reached");
                }

                needToSend = true;
            }


            try{
                sendCommands(needToSend);
            }catch (Exception e){
                e.printStackTrace();
            }



            end = outputCommandSendingDuration(end);
            end = System.currentTimeMillis();

            outputWholeLoopDuration(start);


        }
    }

    private void checkIfGoalChangedAndSetGridNew(Mat currentFrame) {
       if (goalX != pathWindow.mouseX || goalY != pathWindow.mouseY) {
           goalX = pathWindow.mouseX;
           goalY = pathWindow.mouseY;
           grid = DijkstraPathFinder.retrieveDijcstraGrid(currentFrame, new MatOfPoint2f(contour.toArray()), goalX, goalY, stepsize, false, null, null, null);
           goalChanged = true;
       }
    }

    /**
     * this method generates a complete path from robot, to object, to exit.
     *
     * Hereby first the path from object to exit will be generated
     *
     * then around the object (on the surface of a circle) several pick up points
     * will be generated. these pick up points will be evaluated if possible (i.e.
     * no wall between object and pick up point), by angle between point and object
     * in relation to first line of the object_to_exit path, and by distance to robot.
     *
     * after a pick up point is selected, the path from robot to pickup point,
     * will be generated. this path will be concatenated with the line between
     * pickup point and object. Finally this resulting path will be concatenated
     * with the original path
     *
     * @param currentFrame
     * @param robotX
     * @param robotY
     * @param robotR
     */
    private void retrieveObjectShortestPath( Point object,Mat currentFrame, int robotX, int robotY, int robotR) {
        try {

            //pfad vom object zum ausgang


            //findNodeThatRobotGetsObject
            boolean robotWihtinPickupRangePlus10Percent = robotWihinPickupRangePlus10Percent(object);
            boolean distanceAndAngleToObjectIsWrong= distanceAndAngleToObjectIsWrong(object);
            System.out.println("RobotWithinPickUpRange: " + robotWihtinPickupRangePlus10Percent);
            System.out.println("object in Pickup Mouth: " + !distanceAndAngleToObjectIsWrong);

            if (distanceAndAngleToObjectIsWrong&& !robotWihtinPickupRangePlus10Percent) {
                System.out.println("Calculating Path To Pickup Point and from there to Goal");

                //find path to object
                shortestPathFromObject = getShortestPathFromPointToGoal(object);
                LinkedList<Line> pathToPickup = null;
                if (gridToRobotInvertingNeeded == null || objectMoved(object) || goalChanged || shortestPathFromObject == null ) {
                    //object grid is recalculated
                    goalChanged = false;
                    System.out.println("Recalc Object Dyikstra Grid");
                    if (allowFullStopRobot)fullStopRobot();

                    createTabooAreaFromObject(object);
                    pathToPickup = calculatePickupPointAndCalculateGridFromPickupPoint(pathToPickup,currentFrame, object, robotX, robotY,true);
                }else {
                    pathToPickup = getPathToPickup(robotX, robotY,true);
                }



                shortestPathFromObject.add(0, new Line(shortestPathFromObject.getFirst().getB(), pathToPickup.getLast().getA()));
                shortestPathFromObject.addAll(0, pathToPickup);


             } else {
                System.out.println("Robot either close to object or within pickup range");
                if (robotWihtinPickupRangePlus10Percent&&distanceAndAngleToObjectIsWrong) {

                    System.out.println("Calculating Path within pickup range To Object and from there to Goal");
                    shortestPathFromObject = getShortestPathFromPointToGoal(object);

                    //addLineAtBeginning From Robot to the First Path Point
                    shortestPathFromObject.add(0, new Line(shortestPathFromObject.getFirst().getB(), new Node(robotY,robotX)));
                    Line l = shortestPathFromObject.get(0);

                    System.out.println(l);
                    System.out.println(shortestPathFromObject.get(1));

                }else {
                    System.out.println("Object supposed to be in mouth. Directly Drive to goal");
                    shortestPathFromObject = getShortestPathFromPointToGoal(new Point(robotX, robotY));

                }

			}

            if (shortestPathFromObject!=null){
                shortestPath = shortestPathFromObject;
                agent.getHandler().changePath(shortestPath, 0);

			} 
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("No path retrievable. Robot possibly within Contours");
        }
    }

    private void fullStopRobot() {
        System.out.println("Stop Robot whilst recalculating");
        agent.fullStop(0.,0.);
       control.sendCommand(0, 0);

    }

    private LinkedList<Line> calculatePickupPointAndCalculateGridFromPickupPoint(LinkedList<Line> pathToPickup, Mat currentFrame, Point object, int robotX, int robotY,boolean safty) {
        int objectXStepsized = (int) (object.x / stepsize);
        int objectYStepsited = (int) (object.y / stepsize);

        possiblePickUpPoints = getPickUpPoints(objectXStepsized, objectYStepsited, (int) (tabooRadius - 1), grid.length, grid[0].length, true);
        System.out.println("Possible pick up points");

        LinkedList<Double> possibleWayPointsAngleToObjectCenter = createPossibleWayPointsAngles(object, possiblePickUpPoints, shortestPathFromObject.getFirst());
        ArrayList<Double> possibleScores = settleScore(possiblePickUpPoints, possibleWayPointsAngleToObjectCenter);

        //addwhile stuff
        while(pathToPickup == null&&(possiblePickUpPoints.size()>0)) {
            int i = getBest(possibleScores, possiblePickUpPoints);
            java.awt.Point selectedPickUpPoint = possiblePickUpPoints.get(i);
           //if (doesHitWall(grid,shortestPathToObject.getLast().getB().getX()/stepsize,shortestPathToObject.getLast().getB().getY()/stepsize,shortestPathFromObject.getFirst().getA().getX()/stepsize, shortestPathFromObject.getFirst().getA().getY()/stepsize))
            if (safty&&doesHitWall(grid,selectedPickUpPoint.y,selectedPickUpPoint.x,objectYStepsited, objectXStepsized)){
                possibleScores.remove(i);
                possiblePickUpPoints.remove(i);
                continue;
            }

            gridToRobotInvertingNeeded = DijkstraPathFinder.retrieveDijcstraGrid(currentFrame, new MatOfPoint2f(contour.toArray()), selectedPickUpPoint.x * stepsize, selectedPickUpPoint.y * stepsize, stepsize, true, optionalTabooAreaCenter, optionalTabooAreaRadiusSquared, optionalTabooArea);
            distanceObjectToPickup = Math.sqrt(Math.pow((object.x - selectedPickUpPoint.x * stepsize), 2) + Math.pow((object.y - selectedPickUpPoint.y * stepsize), 2));

            System.out.println("Second Grid is new calculated");
            if (DEBUG_SHOW_GRID)drawGrid(gridToRobotInvertingNeeded, possiblePickUpPoints, currentFrame, stepsize);
            try {
                pathToPickup = getPathToPickup(robotX, robotY,true);
            }catch (java.lang.IndexOutOfBoundsException index){
                System.out.println("That pickup point is unfeasable");
                System.out.println("Continue Search for Pickup Point");
            }

            if (pathToPickup == null){
                possibleScores.remove(i);
                possiblePickUpPoints.remove(i);
            }
            currentSelectedPickUpPointStepsized = selectedPickUpPoint;
        }
        if (pathToPickup == null) {
            if (safty == false) return null;
            return calculatePickupPointAndCalculateGridFromPickupPoint(pathToPickup,  currentFrame,  object,  robotX,  robotY,!safty);
        }
        return pathToPickup;
    }

    private LinkedList<Line> getPathToPickup(int robotX, int robotY, boolean checkIfInsideTaboo) {
        LinkedList<Line> pathToPickup;
        int robotXStepsized = robotX / stepsize;
        int robotYStepsized = robotY / stepsize;
        if (grid[robotYStepsized][robotXStepsized] != null) {
            pathToPickup = gridToRobotInvertingNeeded[robotYStepsized][robotXStepsized].getShortestPathLines();
        } else {
            java.awt.Point p = getAlternativeObjectPoint(cv.getCenter(),checkIfInsideTaboo);
            pathToPickup = gridToRobotInvertingNeeded[p.y][p.x].getShortestPathLines();
        }

        pathToPickup = DijkstraPathFinder.reverseLinkedListLine(pathToPickup);
        return pathToPickup;
    }

    private LinkedList<Line> getShortestPathFromPointToGoal(Point startPoint) {
        if (grid[(int) (startPoint.y / stepsize)][(int) (startPoint.x / stepsize)] != null) {
            shortestPathFromObject = DijkstraPathFinder.getShortestPathFromGridLine(grid, new RoboPos(startPoint.y, startPoint.x, 0), stepsize);
        } else {
            tabooRadius = (cv.getRadius()+objectRadius)*1.1/ stepsize;
            java.awt.Point p = getAlternativeObjectPoint(startPoint,false);
            shortestPathFromObject = DijkstraPathFinder.getShortestPathFromGridLine(grid, new RoboPos(p.y * stepsize, p.x * stepsize, 0), stepsize);
        }
        shortestPathFromObject = DijkstraPathFinder.reverseLinkedListLine(shortestPathFromObject);
        return shortestPathFromObject;
    }

    private boolean robotWihinPickupRangePlus10Percent(Point object) {
        if (currentSelectedPickUpPointStepsized == null)return false;
        Point center = cv.getCenter();

        if (center == null)return false;
        if (object == null) return false;
        double distanceObjectToRobot = Math.sqrt(Math.pow((object.x - center.x),2) + Math.pow((object.y - center.y),2));

        System.out.println("Distance Object To Robot: " + distanceObjectToRobot);
        System.out.println("Distance Object To Pickup: " + distanceObjectToPickup*1.1);
        if (distanceObjectToRobot <= distanceObjectToPickup*1.1) return true;

        return false;
    }

    private java.awt.Point getAlternativeObjectPoint(Point object,boolean checkIfInsideTaboo) {
        for (int r = 0; r <= (int) (tabooRadius)+stepsize; r++) {
            ArrayList<java.awt.Point> altPoints = getPickUpPoints((int) object.x / stepsize, (int) object.y / stepsize, r, grid.length, grid[0].length,checkIfInsideTaboo);
            if (altPoints.size()!=0)return (altPoints.get(0));
        }

        return null;
    }

    private boolean objectMoved(Point object) {
        if (gridToRobotNoInvertingNeededObjectPosition != null) {
            ;
            double oldX = gridToRobotNoInvertingNeededObjectPosition.x;
            double oldY = gridToRobotNoInvertingNeededObjectPosition.y;
            double newX = object.x;
            double newY = object.y;
            if ((oldX - newX) * (oldX - newX) + (oldY - newY) * (oldY - newY) < minimalAbstandQuadrat) {
                System.out.println("Not Moved");
                return false;
            }
        }
        System.out.println("Moved");
        gridToRobotNoInvertingNeededObjectPosition = object;
        return true;
    }

    private void drawGrid(Node[][] gridToDraw, ArrayList<java.awt.Point> possiblePickUpPoints, Mat currentFrame, int stepSize) {
        if (gridWindow== null) gridWindow  = ImgWindow.newWindow();
        gridWindow.setTitle("GridWindow");
        Mat f = currentFrame.clone();


       // Imgproc.circle( f, new Point( object.x, object.y ), objectRadius, new Scalar( 128, 0, 128 ), 2 );
        for (java.awt.Point p : possiblePickUpPoints){
            Imgproc.circle( f, new Point( p.x*stepSize, p.y*stepSize ), 3, new Scalar( 90, 45, 128 ), 1 );
        }
        for (int x = 0;x < gridToDraw.length; x++) {
            for (int y = 0; y < gridToDraw[0].length; y++) {
               if (gridToDraw[x][y] != null){
                   Node n = gridToDraw[x][y];
                   Imgproc.circle( f, new Point( n.getY(), n.getX() ), 1, new Scalar( 0, 0, 255 ), 1 );
               }



            }
        }
        gridWindow.setImage(f);
    }

    private LinkedList<Line> selectBestPickupPointAndAddToPath(LinkedList<Line> shortestPathFromObject,Node[][] gridToRobotInvertingNeeded, ArrayList<java.awt.Point> possiblePickUpPoints, LinkedList<Double> possibleWayPointsAngleToObjectCenter, ArrayList<Double> possibleWayPointsLengths, ArrayList<Double> possibleScores, LinkedList<Line> shortestPathToObject, boolean solutionfound) {
        while(possiblePickUpPoints.size()>0&& solutionfound == false){
            int i = getBest(possibleScores,possiblePickUpPoints);

            java.awt.Point pointSelected = null;
            pointSelected = possiblePickUpPoints.get(i);
            shortestPathToObject = DijkstraPathFinder.getShortestPathFromGridLine(gridToRobotInvertingNeeded, new RoboPos(pointSelected.getY()*stepsize, pointSelected.getX()*stepsize, 0, 0), stepsize);
            //DijkstraPathFinder.invertAAndBs(shortestPathToObject);
            //Line line = new Line(shortestPathToObject.getLast().getB(), shortestPathFromObject.getFirst().getA());

            //check if flip of x and y nexxessary

            if (shortestPathToObject==null||doesHitWall(grid,shortestPathToObject.getLast().getB().getX()/stepsize,shortestPathToObject.getLast().getB().getY()/stepsize,shortestPathFromObject.getFirst().getA().getX()/stepsize, shortestPathFromObject.getFirst().getA().getY()/stepsize)){
                possiblePickUpPoints.remove(i);
                possibleWayPointsLengths.remove(i);
                possibleWayPointsAngleToObjectCenter.remove(i);
                solutionfound = false;
            }else{
                shortestPathFromObject.add( new Line(shortestPathToObject.getLast().getA(), shortestPathFromObject.getFirst().getB()));
                System.out.println("Point Selected: x" + pointSelected.x + " y" + pointSelected.y + " i" + i);
                solutionfound = true;
            }
        }
        return shortestPathToObject;
    }


    private LinkedList<Line> selectBestPickupPointAndAddToPathOld(LinkedList<Line> shortestPathFromObject, Node[][] gridToRobotNoInvertingNeeded, ArrayList<java.awt.Point> possiblePickUpPoints, LinkedList<Double> possibleWayPointsAngleToObjectCenter, ArrayList<Double> possibleWayPointsLengths, ArrayList<Double> possibleScores, LinkedList<Line> shortestPathToObject, boolean solutionfound) {
        while(possiblePickUpPoints.size()>0&& solutionfound == false){
            int i = getBest(possibleScores,possiblePickUpPoints);

            java.awt.Point pointSelected = null;
            pointSelected = possiblePickUpPoints.get(i);
            shortestPathToObject = DijkstraPathFinder.getShortestPathFromGridLine(gridToRobotNoInvertingNeeded, new RoboPos(pointSelected.getY()*stepsize, pointSelected.getX()*stepsize, 0, 0), stepsize);
           DijkstraPathFinder.invertAAndBs(shortestPathToObject);
            //Line line = new Line(shortestPathToObject.getLast().getB(), shortestPathFromObject.getFirst().getA());

        //check if flip of x and y nexxessary

            if (shortestPathToObject==null||doesHitWall(grid,shortestPathToObject.getLast().getB().getX()/stepsize,shortestPathToObject.getLast().getB().getY()/stepsize,shortestPathFromObject.getFirst().getA().getX()/stepsize, shortestPathFromObject.getFirst().getA().getY()/stepsize)){
                possiblePickUpPoints.remove(i);
                possibleWayPointsLengths.remove(i);
                possibleWayPointsAngleToObjectCenter.remove(i);
                solutionfound = false;
        }else{
             shortestPathFromObject.add( new Line(shortestPathToObject.getLast().getA(), shortestPathFromObject.getFirst().getB()));
                System.out.println("Point Selected: x" + pointSelected.x + " y" + pointSelected.y + " i" + i);
                solutionfound = true;
            }
        }
        return shortestPathToObject;
    }

    private void createTabooAreaFromObject(Point object) {
        tabooRadius = (cv.getRadius()+objectRadius)*1.1/ stepsize;
        //define Taboo Area which is around the object
        optionalTabooAreaCenter = new Point[1];
        optionalTabooAreaRadiusSquared = new double[1];
        optionalTabooArea = new Rect[1];

        for (int i = 0; i < optionalTabooAreaCenter.length; i++) {
            optionalTabooAreaCenter[i] = new Point(object.x / stepsize, object.y / stepsize);
            double radius = tabooRadius - stepsize;
            int left = (int) (optionalTabooAreaCenter[i].x - radius);
            int up = (int) (optionalTabooAreaCenter[i].y - radius);
            int diameter = (int) (2 * radius)+1;

            optionalTabooArea[i] = new Rect(left, up, diameter, diameter);
            optionalTabooAreaRadiusSquared[i] = radius * radius;
        }

    }

    /**
     * goes through all pick up points and assignes them two respectivly normalised scores between 0 and 1.
     * The first score is depending on the path length between robot and pick up point
     * the second score is depending on the angle between the line (pickuppoint, object) and the line (object, first step)
     * the smaller the path or the angle the better
     *
     * @param possibleWayPoints
     * @param possibleWayPointsAngleToObjectCenter
     * @return
     */
    private ArrayList<Double> settleScore(ArrayList<java.awt.Point> possibleWayPoints, LinkedList<Double> possibleWayPointsAngleToObjectCenter) {
        //quick and dirty
        int i = -1;

        int size = possibleWayPointsAngleToObjectCenter.size();
        ArrayList<Double> scoresOfPoints = new ArrayList<>(size);
        double bestL = Double.MAX_VALUE;
        double worstL = 0.;
        double bestAngle = Math.PI;
        double worstAngle = 0;
        //for (Double pwpL : possibleWayPointsLengths){
        //    if (pwpL < bestL) bestL = pwpL;
        //    if (pwpL > worstL) worstL = pwpL;
       // }
        for (Double pwpA : possibleWayPointsAngleToObjectCenter){
            if (pwpA < bestAngle) bestAngle = pwpA;
            if (pwpA > worstAngle) worstAngle = pwpA;
        }

        Iterator<Double> angleIterator = possibleWayPointsAngleToObjectCenter.iterator();
        for (int j = 0; j < size; j++) {
            double score1 =  1-( (angleIterator.next()-bestAngle)/ (worstAngle - bestAngle));
           // double score2 =  1-( (possibleWayPointsLengths.get(j)-bestL)/ (worstL - bestL));
            scoresOfPoints.add(score1);
            //scoresOfPoints.add(score2);
        }

       return scoresOfPoints;

    }

    /**
     * goes through all pick up points and assignes them two respectivly normalised scores between 0 and 1.
     * The first score is depending on the path length between robot and pick up point
     * the second score is depending on the angle between the line (pickuppoint, object) and the line (object, first step)
     * the smaller the path or the angle the better
     *
     * @param possibleWayPoints
     * @param possibleWayPointsLengths
     * @param possibleWayPointsAngleToObjectCenter
     * @return
     */
    private ArrayList<Double> settleScoreOld(ArrayList<java.awt.Point> possibleWayPoints, ArrayList<Double> possibleWayPointsLengths, LinkedList<Double> possibleWayPointsAngleToObjectCenter) {
        //quick and dirty
        int i = -1;

        int size = possibleWayPointsLengths.size();
        ArrayList<Double> scoresOfPoints = new ArrayList<>(size);
        double bestL = Double.MAX_VALUE;
        double worstL = 0.;
        double bestAngle = Math.PI;
        double worstAngle = 0;
        for (Double pwpL : possibleWayPointsLengths){
            if (pwpL < bestL) bestL = pwpL;
            if (pwpL > worstL) worstL = pwpL;
        }
        for (Double pwpA : possibleWayPointsAngleToObjectCenter){
            if (pwpA < bestAngle) bestAngle = pwpA;
            if (pwpA > worstAngle) worstAngle = pwpA;
        }

        Iterator<Double> angleIterator = possibleWayPointsAngleToObjectCenter.iterator();
        for (int j = 0; j < size; j++) {
            double score1 =  1-( (angleIterator.next()-bestAngle)/ (worstAngle - bestAngle));
            double score2 =  1-( (possibleWayPointsLengths.get(j)-bestL)/ (worstL - bestL));
            scoresOfPoints.add(3*score1+score2);
            //scoresOfPoints.add(score2);
        }

        return scoresOfPoints;

    }

    private int getBest(ArrayList<Double> scoresOfPoints, ArrayList<java.awt.Point> possibleWayPoints) {
        double best = 0;
        int size = possibleWayPoints.size();
        int bestIndex = -1;
        for (int j = 0; j < size; j++) {
            if (best < scoresOfPoints.get(j)){
                best = scoresOfPoints.get(j);
                bestIndex = j;
            }
        }


        return bestIndex;
    }

    /**
     * retrieves for every path from robot to pickupPoint the length
     * and stores it into an array
     * @param possibleWayPoints
     * @param gridToRobotNoInvertingNeeded
     * @return
     */
    private ArrayList<Double> createPossibleWayPointsLengths(ArrayList<java.awt.Point> possibleWayPoints, Node[][] gridToRobotNoInvertingNeeded) {
        ArrayList<Double> possibleWayPointsLengths = new ArrayList<>(possibleWayPoints.size());
        int size = possibleWayPoints.size();

        for (int i = 0; i < size; i++) {
            java.awt.Point pointSelected = possibleWayPoints.get(i);
            possibleWayPointsLengths.add(gridToRobotNoInvertingNeeded[pointSelected.y][ pointSelected.x].getDistance()); //see if need of flip
        }


        return possibleWayPointsLengths;
    }

    private LinkedList<Double> createPossibleWayPointsAngles(Point object, ArrayList<java.awt.Point> possibleWayPoints, Line objectFirstStep) {
        System.out.println("possibleWayPoints" + possibleWayPoints.size());
      //  LinkedList<Node>

        //double angle = getAngleBetweenTwoPoint();

        int maxI = possibleWayPoints.size();
        LinkedList<Double> possibleWayPointsAngleToObjectCenter = new LinkedList<>();
       // double angleFromObject = Math.atan2(objectFirstDirection.getB().getY()-objectFirstDirection.getA().getY(),objectFirstDirection.getB().getX()-objectFirstDirection.getA().getX());

        //apereantly x and y are exchanged. again! so ofdayX and ofday are to be flipped flipped
        //sensible
        //int ofday = objectFirstStep.getA().getY() - objectFirstStep.getB().getY();
        //int ofdax = objectFirstStep.getA().getX() - objectFirstStep.getB().getX();

        //real
        int ofdax = objectFirstStep.getA().getY() - objectFirstStep.getB().getY();
        int ofday = objectFirstStep.getA().getX() - objectFirstStep.getB().getX();

       // int ofday = objectFirstDirection.getA().getY() - objectFirstDirection.getB().getY(); a and b could be reversed
       // int ofdax = objectFirstDirection.getA().getX() - objectFirstDirection.getB().getX();

        int objecty = (int) (object.y/stepsize);
        int objectx = (int) (object.x/stepsize);


       // System.out.println("objectXY_div_Stepsize: " + objectx+ " " + objecty);

       // System.out.println("objectToGoal vector: " + ofdax+ " " + ofday);
        double atan1 =  Math.atan2(ofday, ofdax);
        for (int i = maxI-1; i >= 0; i--) {
            java.awt.Point pwp = possibleWayPoints.get(i);
            if (grid[pwp.y][pwp.x] == null) {possibleWayPoints.remove(i);continue;}

            int vectorFromPickUpToObjectY = (int) (objecty-pwp.getY());
            int vectorFromPickUpToObjectX = (int) (objectx-pwp.getX());;


            double atan2 =  Math.atan2(vectorFromPickUpToObjectY, vectorFromPickUpToObjectX);

            double angle = atan1  - atan2;
            if (angle < 0) angle += 2 * Math.PI;
            if (angle > Math.PI) angle = 2 * Math.PI-angle;

           //  angle =Math.abs(   (angle %(2 * Math.PI)- 2 * Math.PI)  %(2 * Math.PI));

            // if (angle < 0) angle += 2 * Math.PI;


            double calculatedAngleBetweenObjects = angle;
            possibleWayPointsAngleToObjectCenter.add(0,calculatedAngleBetweenObjects);
           // System.out.println("Angle between pickupPoint " +pwp.x + " " + pwp.y + " and path from object calculated: " + calculatedAngleBetweenObjects);
          // System.out.println("PickupToObject vector: " + vectorFromPickUpToObjectX+ " " + vectorFromPickUpToObjectY);
          // System.out.println("path from object vector: " + ofdax+ " " + ofday);
        }

       // Collections.sort(possibleWayPoints,c);
       // System.out.println("possibleWayPoints2" + possibleWayPoints.size());
        return possibleWayPointsAngleToObjectCenter;
    }

    public boolean distanceAndAngleToObjectIsWrong(Point object) {
        Point center = cv.getCenter();
        if (center == null)return false;
        if (object == null) return false;
        Point triangleTip = cv.getAnglePoint();
        System.out.println(triangleTip);
        double deltaTipToCenterX = center.x - triangleTip.x;
        double deltaTipToCenterY = center.y - triangleTip.y;

        double normOfTriangleTip = Math.sqrt(deltaTipToCenterX*deltaTipToCenterX+ deltaTipToCenterY*deltaTipToCenterY);
        double centerNormedDirectionX = deltaTipToCenterX/ normOfTriangleTip;
        double centerNormedDirectionY = deltaTipToCenterY/ normOfTriangleTip;

        if (maxRadius < cv.getRadius()){
            maxRadius = cv.getRadius();
        };
        double radius = maxRadius;

        double mouthBreite = radius*0.65;
        double pauerFactor = 0.9;

         mouthaX = -mouthBreite * centerNormedDirectionY;
         mouthaY = mouthBreite * centerNormedDirectionX;

         mouthbX = mouthBreite * centerNormedDirectionY;
         mouthbY = -mouthBreite * centerNormedDirectionX;

         mouthcX = mouthbX + centerNormedDirectionX * pauerFactor * radius;
         mouthcY = mouthbY + centerNormedDirectionY * pauerFactor * radius;

         mouthdX = mouthaX + centerNormedDirectionX * pauerFactor * radius;
         mouthdY = mouthaY + centerNormedDirectionY * pauerFactor * radius;

        this.mouthaX = mouthaX + center.x;
        this.mouthaY = mouthaY + center.y;

        this.mouthbX = mouthbX +  center.x;
        this.mouthbY = mouthbY  + center.y;

        this.mouthcX = mouthcX + center.x;
        this.mouthcY = mouthcY + center.y;

        this.mouthdX = mouthdX + center.x;
        this.mouthdY = mouthdY + center.y;

       int[] eckPunkteIX = {(int) mouthaX, (int) mouthbX, (int) mouthcX, (int) mouthdX};
        int[] eckPunkteIY = {(int) mouthaY, (int) mouthbY, (int) mouthcY, (int) mouthdY};
        Polygon poly = new Polygon(eckPunkteIX,eckPunkteIY,4);
        if (poly.contains(object.x,object.y))return false; else return true;

        //double width = Math.sqrt((mouthbX-mouthaX)*(mouthbX-mouthaX)+(mouthbY-mouthaY)*(mouthbY-mouthaY));
        //double height = Math.sqrt((mouthbX-mouthaX)*(mouthbX-mouthaX)+(mouthdY-mouthaY)*(mouthdY-mouthaY));

        //double f1 = (mouthbX-mouthaX)/width;
        //double f2= (mouthbY-mouthaY)/height;
        //double f3 =(mouthdX-mouthaX)/width;
        //double f4 = (mouthdY-mouthaY)/height;

        //double deltaAM_x = object.x-mouthaX;
        //double deltaAM_y = object.y-mouthaY;



        //double equation1 = deltaAM_x * f1 + deltaAM_y * f2;
        //double eqaution2= deltaAM_x * f3+ deltaAM_y * f4;

        //if (0<= equation1 && equation1<= width && 0<= eqaution2 && eqaution2<= height){
          //  return false;
        //}


        //return true;
    }
    public boolean distanceAndAngleToObjectIsWrong2(Point object) {

        Point center = cv.getCenter();
        if (center == null)return false;
        if (object == null) return false; //object is invsibile, therefore hidden in the epucs mouth (if that is still our scenerio)

        //uses https://de.wikipedia.org/wiki/Skalarprodukt
        //uses M  of coordinates (x,y) is inside the rectangle iff
        //https://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
        //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        //(scalar product of vectors)

        //we create a rectangle, where the base is the epuc diameter orientatiet ortoganal to the movement axis
        //the left point is A the right point is B
        //the left point far side of the rectangle is D
        Point triangleTip = cv.getAnglePoint();

        double deltaTipToCenterX = center.x - triangleTip.x;
        double deltaTipToCenterY = center.y - triangleTip.y;

        double normOfTriangleTip = Math.sqrt(deltaTipToCenterX*deltaTipToCenterX+ deltaTipToCenterY*deltaTipToCenterY);
        double centerNormedDirectionX = deltaTipToCenterX/ normOfTriangleTip;
        double centerNormedDirectionY = deltaTipToCenterY/ normOfTriangleTip;

        if (maxRadius < cv.getRadius()){
            maxRadius = cv.getRadius();
        };
        double radius = maxRadius;
        double mouthBreite = radius*0.8;
        double pauerFactor = 1.82;

        double mouthaX = mouthBreite * centerNormedDirectionY;
        double mouthaY = -mouthBreite * centerNormedDirectionX;

        double mouthbX = -mouthBreite * centerNormedDirectionY;
        double mouthbY = mouthBreite * centerNormedDirectionX;

        double mouthcX = mouthbX + centerNormedDirectionX * pauerFactor * radius;
        double mouthcY = mouthbY + centerNormedDirectionY * pauerFactor * radius;

        double mouthdX = mouthaX + centerNormedDirectionX * pauerFactor * radius;
        double mouthdY = mouthaY + centerNormedDirectionY * pauerFactor * radius;

        this.mouthaX = mouthaX + center.x;
        this.mouthaY = mouthaY + center.y;

        this.mouthbX = mouthbX +  center.x;
        this.mouthbY = mouthbY  + center.y;

        this.mouthcX = mouthcX + center.x;
        this.mouthcY = mouthcY + center.y;

        this.mouthdX = mouthdX + center.x;
        this.mouthdY = mouthdY + center.y;

        double mX =  object.x;
        double mY =  object.y;

        //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        double deltaAM_x = mX-mouthaX;
        double deltaAM_y = mY-mouthaY;
        double deltaAB_x = mouthbX - mouthaX;
        double deltaAB_y = mouthbY - mouthaY;
        double deltaAD_x = mouthdX - mouthaX;
        double deltaAD_y = mouthdY - mouthaY;

        double scalarAMAB = deltaAM_x*deltaAB_x+deltaAM_y*deltaAB_y;
        double scalarABAB = deltaAB_x*deltaAB_x+deltaAB_y*deltaAB_y;
        double scalarAMAD = deltaAM_x*deltaAD_x+deltaAM_y*deltaAD_y;
        double scalarADAD = deltaAD_x*deltaAD_x+deltaAD_y*deltaAD_y;
        if (scalarAMAB>=0)return true;
        if (scalarABAB>=scalarAMAB) return true;
        if (scalarAMAB>=0)return true;
        if (scalarADAD>=scalarAMAD) return true;


        return false;
    }




    private boolean doesHitWall(Node[][] grid, int x1, int y1, int x2, int y2) {
        // delta of exact value and rounded value of the dependent variable
        int d = 0;

        int dx = Math.abs(x2 - x1);
        int dy = Math.abs(y2 - y1);

        int dx2 = 2 * dx; // slope scaling factors to
        int dy2 = 2 * dy; // avoid floating point

        int ix = x1 < x2 ? 1 : -1; // increment direction
        int iy = y1 < y2 ? 1 : -1;

        int x = x1;
        int y = y1;

        if (dx >= dy) {
            while (true) {
                if (grid[x][y] == null) return true;
                if (x == x2)
                    break;
                x += ix;
                d += dy2;
                if (d > dx) {
                    y += iy;
                    d -= dx2;
                }
            }
        } else {
            while (true) {
                if (grid[x][y] == null) return true;
                if (y == y2)
                    break;
                y += iy;
                d += dx2;
                if (d > dy) {
                    x += ix;
                    d -= dy2;
                }
            }
        }
        return false;
    }

    private void sortTargetPointsByDistance(ArrayList<java.awt.Point> possibleWayPoints) {

    }

    private ArrayList<java.awt.Point> getPickUpPoints(int x0, int y0, int radius, int maxX, int maxY, boolean checkIfInsideTaboo) {
       ArrayList<java.awt.Point> points = new ArrayList<>(60);
            int f = 1 - radius;
            int ddF_x = 0;
            int ddF_y = -2 * radius;
            int x = 0;
            int y = radius;

            points.add(new java.awt.Point(x0, y0 + radius));
            points.add(new java.awt.Point(x0, y0 - radius));
            points.add(new java.awt.Point(x0 + radius, y0));
            points.add(new java.awt.Point(x0 - radius, y0));

            while(x < y)
            {
                if(f >= 0)
                {
                    y--;
                    ddF_y += 2;
                    f += ddF_y;
                }
                x++;
                ddF_x += 2;
                f += ddF_x + 1;

                points.add(new java.awt.Point(x0 + x, y0 + y));
                points.add(new java.awt.Point(x0 - x, y0 + y));
                points.add(new java.awt.Point(x0 + x, y0 - y));
                points.add(new java.awt.Point(x0 - x, y0 - y));
                points.add(new java.awt.Point(x0 + y, y0 + x));
                points.add(new java.awt.Point(x0 - y, y0 + x));
                points.add(new java.awt.Point(x0 + y, y0 - x));
                points.add(new java.awt.Point(x0 - y, y0 - x));
            }

        for (int i = points.size()-1; i >= 0; i--) {
            int xCheck = points.get(i).x;
            int yCheck = points.get(i).y;
            if ( xCheck<0 || xCheck>= maxY || yCheck<0 || yCheck >= maxX){ //stupid flip again

                points.remove(i);
                continue;
            }
            if (grid[yCheck][xCheck]==null) {
                points.remove(i);
                continue;
            }
            if(checkIfInsideTaboo&&DijkstraPathFinder.insideTabooArea(xCheck,yCheck,optionalTabooAreaCenter,optionalTabooAreaRadiusSquared,optionalTabooArea,stepsize)){
                points.remove(i);
                continue;
            }


        }
        return points;
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
            e.printStackTrace();
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
