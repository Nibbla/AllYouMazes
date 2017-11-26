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

    public final static int TIME_STEP = 1000;

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


    public Simulation(String picture) {
        this.pathToPicture = picture;

        Camera camera = new Camera();
        camera.startCamera(60, 1, 1300, 2000, 75, (Settings.getInputPath()), 0.075, 0.1, 0.8, 0.8);


        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        Mat gray = ComputerVision.grayScale(ComputerVision.resize(picture));
        KeyPoint[] kps = ComputerVision.robotv2(ComputerVision.resize(pathToPicture), 0, 0);
        
        if (kps[0] == null || kps[1] == null) {
            System.out.println("oh god no");
            detected = false;
        } else {
            detected = true;
        }
        
        int robotX = (int)kps[0].pt.x;
        int robotY = (int)kps[0].pt.y;
        int robotR = (int)kps[0].size/2;
        
        RoboPos rp = new RoboPos(robotX, robotY, robotR);
        rp.setDirection(rp.getAngleTo(new Node((int)kps[1].pt.x, (int)kps[1].pt.y)));
        
        this.contour = ComputerVision.retrieveContour(gray, rp);
        System.out.println("test0");
        LinkedList<Node> shortestPath = ComputerVision.retrievePath(gray, new MatOfPoint2f(contour.toArray()), rp, 8);
        TraversalHandler traversalHandler = new TraversalHandler(shortestPath, new Node((int) rp.getX(), (int) rp.getY()));
        this.agent = new Agent(0,rp, traversalHandler);
        System.out.println("test1");

        try{
            this.control = factoryControl.getInstance();
        } catch(Exception e){
            e.printStackTrace();
        }

        if (agent == null || contour == null || shortestPath == null) System.out.println("null pointer constructing simulation");

        System.out.println("test2");

        connect();

        System.out.println("test3");
        schedule();

        System.out.println("test4");
    }

    private void connect() {
        control.startConnection();
    }

    private void schedule() {
        //scheduler = Executors.newScheduledThreadPool(1);
        scheduler =  Executors.newSingleThreadScheduledExecutor();

        //Problems with initial angle
        n = agent.getHandler().getNode(agent.getHandler().getIndex());

        final Runnable robotDetector = new Runnable() {
            @Override
            public void run() {
                //boolean finished = agent.getHandler().getIndex()+1 > agent.getHandler().length();
                double rotationCoefficient = 0;
                double linearCoefficient = 0;

                KeyPoint[] kps = null;

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

                if (detected) {
                    int robotX = (int) kps[0].pt.x;
                    int robotY = (int) kps[0].pt.y;
                    int robotR = (int) kps[0].size / 2;

                    lastPosition.setPosition(agent.getCurrentPosition().getX(), agent.getCurrentPosition().getY());
                    lastPosition.setRadius(agent.getCurrentPosition().getRadius());
                    lastPosition.setDirection(agent.getCurrentPosition().getDirection());

                    agent.getCurrentPosition().setPosition(robotX, robotY);
                    agent.getCurrentPosition().setRadius(robotR);
                    agent.getCurrentPosition().setDirection(agent.getCurrentPosition().getAngleTo(new Node((int) kps[1].pt.x, (int) kps[1].pt.y)));
                    // agent.getCurrentPosition().setDirection(lastPosition.getAngleTo(new Node((int) agent.getCurrentPosition().getX(),(int) agent.getCurrentPosition().getY())));

                    double correctAngle = agent.getCurrentPosition().getAngleTo(agent.getHandler().getNode(agent.getHandler().getIndex() + 1));

                    double distance = (Math.toDegrees(correctAngle) - Math.toDegrees(agent.getCurrentPosition().getDirection())) % 360;

                    if (distance < -180) {
                        distance += 360;
                    } else if (distance > 179) {
                        distance -= 360;
                    }

                    if (Math.abs(distance) >= 10) {
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

                    if (!turning) {
                        int x = (int) agent.getCurrentPosition().getX();
                        int y = (int) agent.getCurrentPosition().getY();
                        if (x - ComputerVision.PROXIMITY <= n.getX() && x + ComputerVision.PROXIMITY >= n.getX() && y - ComputerVision.PROXIMITY <= n.getY() && y + ComputerVision.PROXIMITY >= n.getY()) {
                            agent.getHandler().step();
                            turning = false;
                            moving = false;
                            System.out.println("next step");
                        }
                    }

                    if (turning && !moving) {
                        System.out.println("turning");
                        if (!alreadyTurning && (rotationCoefficient == lastRotationCoefficient || lastRotationCoefficient == 0)) {
                            control.sendCommand(linearCoefficient, rotationCoefficient);
                            alreadyTurning = true;
                            alreadyMoving = false;
                            lastRotationCoefficient = rotationCoefficient;
                        }
                    } else if (moving && !turning) {
                        System.out.println("moving");
                        if (!alreadyMoving) {
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
