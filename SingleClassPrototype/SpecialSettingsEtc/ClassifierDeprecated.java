package SpecialSettingsEtc;

import java.io.IOException;

/**
 * Created by Nibbla on 21.10.2017.
 */
public class ClassifierDeprecated implements Changeable<ClassifierDeprecated> {

    //wall rules
    public static  String wallrules = "wallrules";
    public static  double wRedOgreenLowBound = 1.1;
    public static  double wRedOgreenLowBoundP = 1;
    public static  double wRedOblueLowBound = 1.3;
    public static  double wRedOblueLowBoundP = 1;
    public static  double wRed1LowBound = 165;
    public static  double wRed1LowBoundP = 1;
    public static  double wRed2LowBound = 200;
    public static  double wRed2LowBoundP = 2;
    public static  double wGreenLowBound = 200;
    public static  double wGreenLowBoundP = 2;
    public static  double wBlueLowBound = 200;
    public static  double wBlueLowBoundP = 2;
    //floor rules
    public static  String floorrules = "floorrules";
    public static  double fRedOgreenUpBound1=1.2;
    public static  double fRedOgreenLowBound1 = 0.8;
    public static  double wRedOGreenP1 = 1;
    public static  double fGreenOblueUpBound = 1.2;
    public static  double fgreenOblueLowBound = 0.8;
    public static  double wGreenOBlueP = 1;
    public static  double fBlueOGreenUpBound = 1.2;
    public static  double fBlueOGreenLowBound = 0.8;
    public static  double wBlueOGreen = 1;
    public static  double fRedOGreenUpBound2=1.1;
    public static  double fRedOGreenLowBound2 = 0.9;
    public static  double wRedOGreenP2 = 1;
    public static  double fGreenOBlueUpBound = 1.1;
    public static  double fGreenOBlueLowBound = 0.9;
    public static  double fGreenOBlue = 1;
    public static  double fBlueOGreenUpBound2=1.1;
    public static  double fblueOGreenLowBound2 = 0.9;
    public static  double fblueOgreenP2 = 1;


    //robot rules
    public static  String robotrules = "robotrules";
    public static  double rGreenOredLowBound=1.15;
    public static  double rGreenOredLowBoundP=2;
    public static  double rGreenOblueLowBound=1.15;
    public static  double rGreenOblueLowBoundP=2;
    public static  double rGreenLowBound =140;
    public static  double rGreenUpBound=200;
    public static  double rGreenP=2;
    public static  double rGreenUpBound2=270;
    public static  double rGreenUpBoundP2 = -1;


    //goal1 rules

    //goal2 rules


    public ClassifierDeprecated() {
        ClassChanger<ClassifierDeprecated> c;
        Class<?> d = null;
        d = this.getClass();//;Class.forName("sample.RuleSet");
        c = new ClassChanger<ClassifierDeprecated>(d,this);
        c.load("default.clsf");




    }





    @Override
    public void editFields() {

        ClassChanger<ClassifierDeprecated> c;
        Class<?> d = null;

            d = this.getClass();//;Class.forName("sample.RuleSet");

        c = new ClassChanger<ClassifierDeprecated>(d,this);

    }

    @Override
    public void save(String path) throws IOException {
        ClassChanger<ClassifierDeprecated> c;
        Class<?> d = null;

        d = this.getClass();//;Class.forName("sample.RuleSet");

        c = new ClassChanger<ClassifierDeprecated>(d,this);
        c.save("default.clsf");
    }

    @Override
    public void load(String path) throws IOException {
        ClassChanger<ClassifierDeprecated> c;
        Class<?> d = null;

        d = this.getClass();//;Class.forName("sample.RuleSet");

        c = new ClassChanger<ClassifierDeprecated>(d,this);
        c.load("default.clsf");
    }


    public enum RuleSetChoises {PvEDefault}
}
