package SpecialSettingsEtc;

/**
 * Created by Nibbla on 06.10.2017.
 */
public class Settings {
    private static String inputPath = System.getProperty("user.dir") + "\\SingleClassPrototype\\Input\\latestScreen.jpg";
    private static String defaultInputPath = System.getProperty("user.dir") + "\\Input\\latestScreen.jpg";
    private static int stepX = 5; //should be odd number
    private static int stepY = 5; //should be odd number

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
}
