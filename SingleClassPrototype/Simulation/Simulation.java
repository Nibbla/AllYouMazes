package Simulation;

import Control.RobotControl;
import Model.*;
import SpecialSettingsEtc.Settings;
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

    private Agent agent;
    private String pathToPicture;
    private MatOfPoint contour;
    private ScheduledExecutorService scheduler;
    private RobotControl robotControl;
    private Node n;
    private double direction;

    public Simulation(String picture) {
        this.pathToPicture = picture;
        Mat gray = ComputerVision.grayScale(ComputerVision.resize(picture));
        RoboPos currentDetails = ComputerVision.retrieveRobot(gray, 0, 0);
        this.contour = ComputerVision.retrieveContour(gray, currentDetails);
        LinkedList<Node> shortestPath = ComputerVision.retrievePath(gray, new MatOfPoint2f(contour.toArray()), currentDetails, ComputerVision.STEP_SIZE);
        TraversalHandler traversalHandler = new TraversalHandler(shortestPath, new Node((int) currentDetails.getX(), (int) currentDetails.getY()));
        this.agent = new Agent(0,currentDetails, traversalHandler);
        this.robotControl = new RobotControl();

        if (agent == null || contour == null || shortestPath == null) System.out.println("null pointer constructing simulation");

        connect();

        Camera camera = new Camera();
        camera.startCamera(60, 1, 1300, 2000, 75, false, false, Settings.getInputPath(), 0.075, 0.1, 0.8, 0.8);


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
                RoboPos oldLocation = new RoboPos(agent.getCurrentPosition().getX(), agent.getCurrentPosition().getY(), agent.getCurrentPosition().getRadius());
                agent.setCurrentPosition(ComputerVision.retrieveRobot(ComputerVision.grayScale(ComputerVision.resize(Settings.getInputPath())), 0, 0));
                if (oldLocation.equals(agent.getCurrentPosition())) System.out.println("No change / can't detect");

                int x = (int) agent.getCurrentPosition().getX();
                int y = (int) agent.getCurrentPosition().getY();
                if (x - ComputerVision.PROXIMITY <= n.getX() && x + ComputerVision.PROXIMITY >= n.getX() && y - ComputerVision.PROXIMITY <= n.getY() && y + ComputerVision.PROXIMITY >= n.getY()) {
                    System.out.println("Close enough to move to next node");
                    agent.getHandler().step();
                    int m = agent.getHandler().getIndex();
                    if (m != 0) {
                        Node prevNode = agent.getHandler().getNode(m-1);
                        Node newNode = agent.getHandler().getNode(m);

                        // changed direction calculation to be consistent with (hopefully correct) calculation in RoboPos
                        direction = Math.atan2(newNode.getX() - prevNode.getX(), -1 *(newNode.getY() - prevNode.getY()));
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
