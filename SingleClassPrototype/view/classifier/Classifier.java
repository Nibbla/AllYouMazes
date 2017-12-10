package view.classifier;

import Interfaces.ObjectType;
import SpecialSettingsEtc.ClassChanger;
import net.objecthunter.exp4j.operator.Operator;


import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Created by Nibbla on 21.10.2017.
 */
public class Classifier implements IClassifier {

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
    public static  double rRedLowBound = 200;
    public static double rRedLowBoundP = 5;
    private final List<Operator> operators;

    ClassChanger<Classifier> c;
    private ArrayList<String> rules = new ArrayList<>();
    //goal1 rules

    //goal2 rules


    public ArrayList<String> getRules() {
        return rules;
    }

    public void setRules(ArrayList<String> rules) {
        this.rules = rules;
    }

    public Classifier() {
        operators = createOperators();
        createRules();

        Class<?> d = null;

        d = this.getClass();//;Class.forName("sample.RuleSet");

        c = new ClassChanger<Classifier>(d,this);
        c.load("default.clsf");
        c.apply();
    }

    private void createRules() {

    }

    private List<Operator> createOperators() {
        ArrayList<Operator> op = new ArrayList<>();
        op.add(new bigger(2));
        op.add(new biggerOrEqual(3));
        op.add(new smallerOrEqual(4));
        op.add(new smaller(5));
        op.add(new equal(2));
        op.add(new and(6));
        op.add(new or(7));
        op.add(new leadsToo(1));
        return op;
    }





    @Override
    public void editFields() {

       c.setVisible(true);

    }

    @Override
    public void save(String path) throws IOException {
        ClassChanger<Classifier> c;
        Class<?> d = null;

        d = this.getClass();//;Class.forName("sample.RuleSet");

        c = new ClassChanger<Classifier>(d,this);
        c.save("default.clsf");
    }

    @Override
    public void load(String path) throws IOException {
        ClassChanger<Classifier> c;
        Class<?> d = null;

        d = this.getClass();//;Class.forName("sample.RuleSet");

        c = new ClassChanger<Classifier>(d,this);
        c.load("default.clsf");
    }

    public List<Operator> getOperators() {
        return operators;
    }

    @Override
    public HashMap<ObjectType, Double> classify(double red, double green, double blue) {
        return null;
    }

    public HashMap<ObjectType, Double> preclassify(double red, double green, double blue) {

        return null;
    }


    public enum whyRuleSetChoises {PvEDefault}
//https://lallafa.objecthunter.net/exp4j/#Custom_operators

    private class leadsToo extends Operator {
        public leadsToo(int precedence) {
            super("-->", 2, true, precedence);
        }

        @Override
        public double apply(double... doubles) {
            final double arg1 = (int) doubles[0];
            final double arg2 = (int) doubles[1];
            if (arg1>0) return arg2;
            else return 0;
        }
    }
    private class bigger extends Operator {
        public bigger(int precedence) {
            super(">", 2, true, precedence);
        }

        @Override
        public double apply(double... doubles) {
            final double arg1 = (int) doubles[0];
            final double arg2 = (int) doubles[1];
            if (arg1 > arg2) return 1;
            else return 0;
        }
    }

    private class biggerOrEqual extends Operator {
        public biggerOrEqual(int i) {
            super(">=",2,true,i);
        }

        @Override
        public double apply(double... doubles) {
            final double arg1 = (int) doubles[0];
            final double arg2 = (int) doubles[1];
            if (arg1 >= arg2) return 1;
            else return 0;
        }
    }

    private class smallerOrEqual extends Operator {
        public smallerOrEqual(int i) {
            super("<=",2,true,i);
        }

        @Override
        public double apply(double... doubles) {
            final double arg1 = (int) doubles[0];
            final double arg2 = (int) doubles[1];
            if (arg1 <= arg2) return 1;
            else return 0;
        }
    }

    private class smaller extends Operator {
        public smaller(int i) {
            super("<",2,true,i);
        }

        @Override
        public double apply(double... doubles) {
            final double arg1 = (int) doubles[0];
            final double arg2 = (int) doubles[1];
            if (arg1 < arg2) return 1;
            else return 0;
        }
    }

    private class equal extends Operator {
        public equal(int i) {
            super("==",2,true,i);
        }


        @Override
        public double apply(double... doubles) {
            final double arg1 = (int) doubles[0];
            final double arg2 = (int) doubles[1];
            if (arg1 == arg2) return 1;
            else return 0;

        }

    }

    private class and extends Operator {
        public and(int i) {
            super("&&",2,true,i);
        }

        @Override
        public double apply(double... doubles) {
            final double arg1 = (int) doubles[0];
            final double arg2 = (int) doubles[1];
            if ((arg1>0) && (arg2>0)) return 1;
            else return 0;
        }
    }

    private class or extends Operator {
        public or(int i) {
            super("||",2,true,i);
        }

        @Override
        public double apply(double... doubles) {
            final double arg1 = (int) doubles[0];
            final double arg2 = (int) doubles[1];
            if ((arg1>0) || (arg2>0)) return 1;
            else return 0;
        }
    }
}
