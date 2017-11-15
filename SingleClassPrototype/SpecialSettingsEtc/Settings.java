package SpecialSettingsEtc;

import java.io.File;

/**
 * Created by Nibbla on 06.10.2017.
 */
public class Settings {
    private static String inputPath = System.getProperty("user.dir") + File.separator +"SingleClassPrototype" + File.separator + "Input" + File.separator + "latestScreen.jpg";
    private static String defaultInputPath = System.getProperty("user.dir") + "\\Input\\latestScreen.jpg";
    private static int stepX = 5; //should be odd number
    private static int stepY = 5; //should be odd number
    private static int scansize = 4;
    private static int medianFilterSize = 3;
    private static int maximumFilterSize = 3;
    public static double[] wallClassifier = {0.3,0.3,0.3,0.9,0.1,0.2,0.2,0.65,0.15};  //inplement store save (numblock and wsd. 0 means save)
    private static int graphCreationPixelSkip = 4;


    public static String getInputPath() {
        return inputPath;
    }

    public static void setInputPath(String inputPath) {
        Settings.inputPath = inputPath;
    }

    public static String getDefaultInputPath() {
        return defaultInputPath;
    }

    public static void setDefaultInputPath(String defaultInputPath) {
        Settings.defaultInputPath = defaultInputPath;
    }

    public static int getStepX() {
        return stepX;
    }

    public static void setStepX(int stepX) {
        Settings.stepX = stepX;
    }

    public static int getStepY() {
        return stepY;
    }

    public static int getScansize() {
        return scansize;
    }

    public static void setScansize(int scansize) {
        Settings.scansize = scansize;
    }

    public static int getMedianFilterSize(int i) {
        return medianFilterSize;
    }

    public static int getMaximumFilterSize() {
        return maximumFilterSize;
    }

    public static void setMaximumfilterSize(int maximumfilterSize) {
        Settings.maximumFilterSize = maximumfilterSize;
    }

    public static int getGraphCreationPixelSkip() {
        return graphCreationPixelSkip;
    }
}
