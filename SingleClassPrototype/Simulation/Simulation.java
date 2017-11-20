package Simulation;

import Control.RobotControl;
import Model.ComputerVision;
import Model.Node;
import Model.TraversalHandler;
import SpecialSettingsEtc.Settings;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;

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

    private String pathToPicture;
    private ArrayList<Integer> robotCurrentDetails;
    private MatOfPoint contour;
    private ScheduledExecutorService scheduler;
    private TraversalHandler traversalHandler;
    private RobotControl robotControl;
    private Node n;
    private double direction;

    public Simulation(String picture) {
        this.pathToPicture = picture;
        Mat gray = ComputerVision.preprocess(picture);
        this.robotCurrentDetails = ComputerVision.retrieveRobot(gray, 0, 0);
        this.contour = ComputerVision.retrieveContour(gray, robotCurrentDetails);
        LinkedList<Node> shortestPath = ComputerVision.retrievePath(gray, new MatOfPoint2f(contour.toArray()), robotCurrentDetails, ComputerVision.STEP_SIZE);
        this.traversalHandler = new TraversalHandler(shortestPath, new Node(robotCurrentDetails.get(0), robotCurrentDetails.get(1)));
        this.robotControl = new RobotControl();

        if (robotCurrentDetails == null || contour == null || shortestPath == null) System.out.println("null pointer constructing simulation");

        connect();
        schedule();
    }

    private void connect() {
        robotControl.startConnection();
    }

    private void initAngle() {

    }

    private void schedule() {
        //scheduler = Executors.newScheduledThreadPool(1);
        scheduler =  Executors.newSingleThreadScheduledExecutor();

        //Problems with initial angle
        n = null;
        direction = 0;

        final Runnable robotDetector = new Runnable() {
            @Override
            public void run() {
                ArrayList<Integer> oldLocation = robotCurrentDetails;
                robotCurrentDetails = ComputerVision.retrieveRobot(ComputerVision.preprocess(Settings.getInputPath()), 0, 0);
                if (oldLocation.equals(robotCurrentDetails)) System.out.println("No change / can't detect");

                int x = robotCurrentDetails.get(0);
                int y = robotCurrentDetails.get(1);
                if (x - ComputerVision.PROXIMITY <= n.getX() && x + ComputerVision.PROXIMITY >= n.getX() && y - ComputerVision.PROXIMITY <= n.getY() && y + ComputerVision.PROXIMITY >= n.getY()) {
                    System.out.println("Close enough to move to next node");
                    traversalHandler.step();
                    int m = traversalHandler.getIndex();
                    if (m != 0) {
                        Node prevNode = traversalHandler.getNode(m-1);
                        Node newNode = traversalHandler.getNode(m);
                        direction = Math.atan2(newNode.getY() - prevNode.getY(), newNode.getX() - prevNode.getX());
                        //send robot command
                        //problem: determine correct angle..
                    }
                }
            }
        };

        final ScheduledFuture<?> robotDetectorHandle = scheduler.scheduleAtFixedRate(robotDetector, 1000, Simulation.TIME_STEP, TimeUnit.MILLISECONDS);
    }

    private void finish() {
        scheduler.shutdown();
        System.exit(0);
    }

    public static void main(String[] args) {
        Simulation sim = new Simulation(Settings.getInputPath());
    }

}
