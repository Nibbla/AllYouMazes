import Control.RobotControl;
import Interfaces.*;
import Model.Model;
import SpecialSettingsEtc.Archivar;
import view.PixelObjectType;
import Model.Path;
import Model.SpecialGraph;
import view.View;
import javafx.util.Pair;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Random;
import java.util.Stack;
import Model.RoboPos;


/** This class defines the basic structure of our program
 * It is defined by the model view controller architecture.
 * view is responsible for processing the image stream and showing it on screen
 * model is responsible for converting the visual to the internal data model
 * and it creates pathways
 * RobotControl then converts the path order in orders fors the robot.
 */
public class Modul {
    private static Random rnd = new RandomCount();



    private static View factoryView = new View();
    private static Model factoryModel = new Model(rnd);
    private static Control.RobotControl factoryControl = new RobotControl();


    private Thread processingThread;
    public IView view;
    public IModel model;
    public IControl control;
    private boolean[] workmodes = new boolean[Workmode.values().length]; //which workmodes are turned on.
    private boolean running;


    public Modul(View view , Model model, IControl control) {
        this.view = view;
        this.model = model;
        this.control = control;
    }


    public static void main(String[] args){

        try {
            Modul modul = new Modul(factoryView.getInstance(),factoryModel.getInstance(),factoryControl);

            modul.setWorkmode(Workmode.CAMERAON,true);
            modul.setWorkmode(Workmode.KEYBOARDON, true);
            modul.setWorkmode(Workmode.SYSTEMOUT, true);
            modul.setWorkmode(Workmode.SYSTEMOUTARCHIVE, false);
            modul.setWorkmode(Workmode.SHOWSENSOR,true);
            //modul.setWorkmode(Workmode.SHOWKLASSIFIED, true);
            //modul.setWorkmode(Workmode.SHOWTESSELATED, true);
            modul.start(true);



        } catch (CloneNotSupportedException e) {
            e.printStackTrace();
        }



    }

    private void mainLoop() {
        running = true;
        int loop = -1;
        while (running){
            loop++;
            long loopStart = System.currentTimeMillis();
            //something is happening
            windowManagment();

            //https://www.youtube.com/watch?v=bjSpO2B6G4s

            //getImage

            BufferedImage bi = view.getCurrentShot();

            ObjectType[][] m2 = view.classify(bi);

            //Get robot pos

            RoboPos robotPos = view.getRobotCenter(m2, 50);
            System.out.println("Robot position is " + robotPos.getX() + ":" + robotPos.getY());

            SpecialGraph g = view.getGraph(m2,bi.getType(),robotPos);
            g.calculatePathway(robotPos,0,0);
           // g.setStart(m.getRobotPosition());
           // g.setGoal(4f,200f);
           // Path p = g.calculatePathway();
           // translateIntoCommands(p);
            
            long loopEnd = System.currentTimeMillis();
            double timeHappend = (loopEnd - loopStart)/1000.;
            Archivar.shout("Loop: "+ loop + " took " + timeHappend + " seconds to complete");
            break;
        }

    }



    private void translateIntoCommands(Path p) {

    }

    private void windowManagment() {
        show(Workmode.SHOWSENSOR,isWorkmode(Workmode.SHOWSENSOR));
    }


    private void start(boolean b) {
        stopProcessingThread();
        if (!b) return;


        processingThread = new Thread() {

            public void run(){
                mainLoop();
            }
        };
        processingThread.start();
    }



    private void stopProcessingThread() {
        running = false;
        processingThread = null;
    }

    private void show(Workmode tesselated, boolean b) {
        switch (tesselated) {
            case KEYBOARDON:
                break;
            case SYSTEMOUT:
                break;
            case CAMERAON:
                break;
            case SHOWKLASSIFIED:
                break;
            case SHOWTESSELATED:
                break;
            case SHOWSENSOR:
                break;
        }
    }

    public boolean isWorkmode(Workmode w){
        return workmodes[w.ordinal()];
    }

    private void setWorkmode(Workmode w, boolean b) {
        workmodes[w.ordinal()] = b;
        if (w.equals(Workmode.SYSTEMOUT)) Archivar.liveOutput(b);
        if (w.equals(Workmode.SYSTEMOUTARCHIVE)) Archivar.setStoreShouts(b);
    }









    /**
     * extendes random so the time it is executed is measured
     */
    private static class RandomCount extends Random {
        private static long rndCount = 0l;

        @Override
        public double nextDouble() {
            rndCount++;
            return super.nextDouble();
        }

        @Override
        public long nextLong() {
            rndCount++;
            return super.nextLong();
        }

        @Override
        public boolean nextBoolean() {
            rndCount++;
            return super.nextBoolean();
        }

        @Override
        public int nextInt() {
            rndCount++;
            return super.nextInt();
        }

        @Override
        public int nextInt(int bound) {
            rndCount++;
            return super.nextInt(bound);
        }
    }


}
