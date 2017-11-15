package view;

import Interfaces.ObjectType;
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

    double[] wc = Settings.wallClassifier;
    private ObjectType classifiedas;

    public PixelObjectType(int x, int y, BufferedImage rgb) {
        Integer value = rgb.getRGB(x, y);
        this.x = x;
        this.y = y;
        Color c = new Color(value);
        red = c.getRed();
        green = c.getGreen();
        blue = c.getBlue();

        if (red/green>1.1) wallility+=1;
        if (red/blue>1.3) wallility+=1;
        if (red>165) wallility+=1;
        if (red>200) wallility+=2;
        if (green>200) wallility+=2;
        if (blue>200) wallility+=2;


        if (red/green<1.2&&red/green>0.8) floorility+=1;
        if (green/blue<1.2&&green/blue>0.8) floorility+=1;
        if (blue/green<1.2&&blue/green>0.8) floorility+=1;
        if (red/green<1.1&&red/green>0.9) floorility+=1;
        if (green/blue<1.1&&green/blue>0.9) floorility+=1;
        if (blue/green<1.1&&blue/green>0.9) floorility+=1;


        //if (green/blue<1.1&&green/blue>0.9) floorility+=1;
        //if (blue/green<1.1&&blue/green>0.9) floorility+=1;


        if (green/red>1.15) robotility+=2;
        if (green/blue>1.15) robotility+=2;
        //if (green/red>1.25) robotility+=1;
        //if (green/blue>1.25) robotility+=1;
        if (green>100&&green<200) robotility+=2;
        if (green<70) robotility-=1;

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