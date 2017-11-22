package SpecialSettingsEtc;

import java.io.IOException;
import java.util.List;

/**
 * Created by Nibbla on 21.10.2017.
 */
public class Classifier implements Changeable<Classifier> {

    //wall rules
    public String wallrules = "wallrules";
    public double wRedOgreenLowBound;
    public double wRedOgreenLowBoundP = 1;
    public double wRedOblueLowBound = 1.3;
    public double wRedOblueLowBoundP = 1;
    public double wRed1LowBound = 165;
    public double wRed1LowBoundP = 1;
    public double wRed2LowBound = 200;
    public double wRed2LowBoundP = 2;
    public double wGreenLowBound = 200;
    public double wGreenLowBoundP = 2;
    public double wBlueLowBound = 200;
    public double wBlueLowBoundP = 2;
    //floor rules
    public String floorrules = "floorrules";
    public double fRedOgreenUpBound1=1.2;
    public double fRedOgreenLowBound1 = 0.8;
    public double wRedOGreenP1 = 1;
    public double fGreenOblueUpBound = 1.2;
    public double fgreenOblueLowBound = 0.8;
    public double wGreenOBlueP = 1;
    public double fBlueOGreenUpBound = 1.2;
    public double fBlueOGreenLowBound = 0.8;
    public double wBlueOGreen = 1;
    public double fRedOGreenUpBound2=1.1;
    public double fRedOGreenLowBound2 = 0.9;
    public double wRedOGreenP2 = 1;
    public double fGreenOBlueUpBound = 1.1;
    public double fGreenOBlueLowBound = 0.9;
    public double fGreenOBlue = 1;
    public double fBlueOGreenUpBound2=1.1;
    public double fblueOGreenLowBound2 = 0.9;
    public double fblueOgreenP2 = 1;


    //robot rules
    public String robotrules = "robotrules";
    public double rGreenOredLowBound=1.15;
    public double rGreenOredLowBoundP=2;
    public double rGreenOblueLowBound=1.15;
    public double rGreenOblueLowBoundP=2;
    public double rGreenLowBound =140;
    public double rGreenUpBound=200;
    public double rGreenP=2;
    public double rGreenUpBound2=270;
    public double rGreenUpBoundP2 = -1;


    //goal1 rules

    //goal2 rules


    public Classifier() {
    }





    @Override
    public void edit() {

        ClassChanger<Classifier> c;
        Class<?> d = null;

            d = this.getClass();//;Class.forName("sample.RuleSet");

        c = new ClassChanger<Classifier>(d,this);

    }

    @Override
    public void save() throws IOException {

    }

    @Override
    public void load() throws IOException {

    }


    public enum RuleSetChoises {PvEDefault}
}
