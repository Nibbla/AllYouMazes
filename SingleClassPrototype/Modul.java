import Control.Control;
import Interfaces.*;
import Model.Model;
import SpecialSettingsEtc.Archivar;
import View.PixelObjectType;
import Model.Path;
import Model.SpecialGraph;
import View.View;
import javafx.util.Pair;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Random;
import java.util.Stack;


/** This class defines the basic structure of our program
 * It is defined by the model view controller architecture.
 * view is responsible for processing the image stream and showing it on screen
 * model is responsible for converting the visual to the internal data model
 * and it creates pathways
 * Control then converts the path order in orders fors the robot.
 */
public class Modul {
    private static Random rnd = new RandomCount();



    private static View factoryView = new View();
    private static Model factoryModel = new Model(rnd);
    private static Control factoryControl = new Control();


    private Thread processingThread;
    public IView view;
    public IModel model;
    public IControl control;
    private boolean[] workmodes = new boolean[Workmode.values().length]; //which workmodes are turned on.
    private boolean running;


    public Modul(View view , Model model, Control control) {
        this.view = view;
        this.model = model;
        this.control = control;
    }


    public static void main(String[] args){

        try {
            Modul modul = new Modul(factoryView.getInstance(),factoryModel.getInstance(),factoryControl.getInstance());

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

            View.PixelObjectType[][] m2 = view.classify(bi);

            //Get robot pos
            Pair<Double, Double> robotPos = getRobotCenter(m2, 50);
            System.out.println("Robot position is " + robotPos.getKey() + ":" + robotPos.getValue());

            SpecialGraph g = view.getGraph();
           // g.setStart(m.getRobotPosition());
           // g.setGoal(4f,200f);
           // Path p = g.calculatePathway();
           // translateIntoCommands(p);
            
            long loopEnd = System.currentTimeMillis();
            double timeHappend = (loopEnd - loopStart)/1000.;
            Archivar.shout("Loop: "+ loop + " took " + timeHappend + " seconds to complete");
        }

    }

    private Pair<Double, Double> getRobotCenter(View.PixelObjectType[][] m2, int numberOfPixelsToSkip) {
        ArrayList<ArrayList<Pair<Integer, Integer>>> clusters = new ArrayList<>();

        //Checking every pixel takes way too much computation time, I tested with skipping 50, works fine.
        for (int i = 0; i < m2.length; i+=numberOfPixelsToSkip) {
            for (int j = 0; j < m2[i].length; j+=numberOfPixelsToSkip) {
                if (m2[i][j].getSelectedClass().name().equals("robot")) {

                    for (int k = 0; k < clusters.size(); k++) {
                        ArrayList<Pair<Integer, Integer>> curCluster = clusters.get(k);
                        if (curCluster.contains(new Pair(i, j))) {
                            continue;
                        }
                    }

                    Stack<Pair<Integer, Integer>> s = new Stack();
                    ArrayList<Pair<Integer, Integer>> cluster = new ArrayList<>();
                    s.push(new Pair(i, j));

                    //Flood fill, also making sure we don't get into loops
                    while (!s.isEmpty()) {
                        Pair<Integer, Integer> pair = s.pop();
                        if (m2[pair.getKey()][pair.getValue()].getSelectedClass().name().equals("robot") && !cluster.contains(pair)) {
                            //System.out.println(i + ":" + j + " " + s.size());
                            cluster.add(pair);

                            if (pair.getKey() - 1 > 0 && pair.getKey() + 1 < m2.length && pair.getValue() - 1 > 0 && pair.getValue() + 1 < m2[0].length) {
                                s.push(new Pair(pair.getKey(), pair.getValue() + 1));
                                s.push(new Pair(pair.getKey(), pair.getValue() - 1));
                                s.push(new Pair(pair.getKey() + 1, pair.getValue()));
                                s.push(new Pair(pair.getKey() - 1, pair.getValue()));
                            }

                            //We can probably ignore all the other cases as the robot should be around the wall then, also we'd only miss a few pixels
                        }

                    }

                    if (cluster.size() > 1) {
                        clusters.add(cluster);
                    }
                }
            }
        }

        //Extrapolating robot position from the largest cluster that we found (we assume the largest is the robot)
        ArrayList<Pair<Integer, Integer>> largestCluster = new ArrayList<>();
        int max = 0;
        for (int i = 0; i < clusters.size(); i++) {
            if (clusters.get(i).size() > max) {
                largestCluster = clusters.get(i);
                max = clusters.get(i).size();
            }
        }

        double xSum = 0;
        double ySum = 0;

        for (Pair<Integer, Integer> p: largestCluster) {
            xSum += p.getKey();
            ySum += p.getValue();
        }

        double x = xSum / largestCluster.size();
        double y = ySum / largestCluster.size();
        return new Pair(x, y);
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
