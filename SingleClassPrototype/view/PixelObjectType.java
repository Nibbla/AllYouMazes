package view;

import Interfaces.ObjectType;
import SpecialSettingsEtc.Classifier;
import SpecialSettingsEtc.Settings;

import java.awt.*;
import java.awt.image.BufferedImage;

public class PixelObjectType {

    private final int y;
    private final int x;
    private final double red;
    private final double green;
    private final double blue;
    public double wallility;
    public double floorility;
    public double robotility;
    public double goalility1;
    public double goalility2;

    double[] wc = Settings.wallClassifier;
    private ObjectType classifiedas;

    public PixelObjectType(int x, int y, BufferedImage rgb, Classifier cl) {
        Integer value = rgb.getRGB(x, y);
        this.x = x;
        this.y = y;
        Color c = new Color(value);
        red = c.getRed();
        green = c.getGreen();
        blue = c.getBlue();

        if (red/green>cl.wRedOgreenLowBound) wallility= wallility + cl.wRedOgreenLowBoundP;
        if (red/blue>cl.wRedOblueLowBound) wallility= wallility + cl.wRedOblueLowBoundP;
        if (red>cl.wRed1LowBound) wallility= wallility +cl.wRed1LowBoundP;
        if (red>cl.wRed2LowBound) wallility= wallility +cl.wRed2LowBoundP;
        if (green>cl.wGreenLowBound) wallility= wallility +cl.wGreenLowBoundP;
        if (blue>cl.wBlueLowBound) wallility= wallility +cl.wBlueLowBoundP;



        if (red/green<cl.fRedOgreenUpBound1&&red/green>cl.fRedOgreenLowBound1) floorility= floorility +cl.wRedOGreenP1;
        if (green/blue<cl.fGreenOblueUpBound&&green/blue>cl.fgreenOblueLowBound) floorility=floorility +cl.wGreenOBlueP;
        if (blue/green<cl.fBlueOGreenUpBound&&blue/green>cl.fBlueOGreenLowBound) floorility=floorility +cl.wBlueOGreen;
        if (red/green<cl.fRedOGreenUpBound2&&red/green>cl.fRedOGreenLowBound2) floorility=floorility +cl.wRedOGreenP2;
        if (green/blue<cl.fGreenOBlueUpBound&&green/blue>cl.fGreenOBlueLowBound) floorility= floorility +cl.fGreenOBlue;
        if (blue/green<cl.fBlueOGreenUpBound2&&blue/green>cl.fblueOGreenLowBound2) floorility= floorility +cl.fblueOgreenP2;


        //if (green/blue<1.1&&green/blue>0.9) floorility+=1;
        //if (blue/green<1.1&&blue/green>0.9) floorility+=1;


        if (green/red>cl.rGreenOredLowBound) robotility= robotility+cl.rGreenOredLowBoundP;
        if (green/blue>cl.rGreenOblueLowBound) robotility= robotility+ cl.rGreenOblueLowBoundP;
        //if (green/red>1.25) robotility+=1;
        //if (green/blue>1.25) robotility+=1;
        if (green>cl.rGreenLowBound&&green<cl.rGreenUpBound) robotility = robotility + cl.rGreenP;
        if (green<cl.rGreenUpBound2) robotility= robotility + cl.rGreenUpBoundP2;
        if (red>cl.rRedLowBound) robotility= robotility + cl.rRedLowBoundP;

        // wallility = wc[0] * red + wc[1] * green + wc[2] * blue;
        // floorility = wc[3] * red + wc[4] * green + wc[5] * blue;
        // robotility = wc[6] * red + wc[7] * green + wc[8] * blue;

    }

    public PixelObjectType(int x, int y, double red, double green, double blue, double wallility, double floorility, double robotility, ObjectType classifiedas) {
        this.x = x;
        this.y = y;
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.wallility = wallility;
        this.floorility = floorility;
        this.robotility = robotility;
        this.classifiedas = classifiedas;
    }

    public ObjectType getSelectedClass() {
        if (classifiedas != null)  return classifiedas;
        if (wallility > floorility && wallility > robotility) return ObjectType.wall;
        if (floorility > robotility) return ObjectType.floor;

        return ObjectType.robot;

    }

    public void classifiedAs(int i) {
        this.classifiedas = ObjectType.values()[i];
    }
    public void classifiedAs(ObjectType i) {
        this.classifiedas = i;
    }

    public PixelObjectType copy() {
        PixelObjectType p = new PixelObjectType(x,y,red,green,blue,wallility,floorility,robotility,classifiedas);
        return p;
    }
}