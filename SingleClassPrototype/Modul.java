import Control.Control;
import Interfaces.*;
import Model.Model;
import View.View;

import java.util.Random;


/** This class defines the basic structure of our program
 * It is defined by the model view controller architecture.
 * View is responsible for processing the image stream and showing it on screen
 * Model is responsible for converting the visual to the internal data model
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
            modul.start(true);
            modul.show(Showoptions.SENSOR,true);
            modul.show(Showoptions.KLASSIFIED, true);
            modul.show(Showoptions.TESSELATED, true);


        } catch (CloneNotSupportedException e) {
            e.printStackTrace();
        }



    }

    private void mainLoop() {
        running = true;
        while (running){

        }

    }


    private void start(boolean b) {
        stopProcessingThread();
        if (!b) return;

        processingThread = new Thread() {
            private boolean running;
            public void run(){
                mainLoop();
            }
        };
    }



    private void stopProcessingThread() {
        running = false;
        processingThread = null;
    }

    private void show(Showoptions tesselated, boolean b) {
        switch (tesselated) {
            case KLASSIFIED:
                break;
            case TESSELATED:
                break;
            case SENSOR:
                break;
        }

    }

    public boolean isWorkmode(Workmode w){
        return workmodes[w.ordinal()];
    }

    private void setWorkmode(Workmode w, boolean b) {
        workmodes[w.ordinal()] = true;
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
