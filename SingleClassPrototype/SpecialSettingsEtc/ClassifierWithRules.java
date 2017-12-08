package SpecialSettingsEtc;

import Interfaces.ObjectType;
import net.objecthunter.exp4j.operator.Operator;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Created by Nibbla on 21.10.2017.
 */
public class ClassifierWithRules implements  IClassifier {

    //wall rules

    private final List<Operator> operators;

    ClassChanger<ClassifierWithRules> c;
    private  List<Rules> rules;
    private Frame frame;
    //goal1 rules

    //goal2 rules
    private static class Rules {
        private int i;
        public static  int j;
        private int string;

        public static void addOrder(Operator o) {
            if ()
        }

        public int getString() {
            return string;
        }
    }
    public static List<Rules> loadRules(String s){
        JFileChooser fileChooser = new JFileChooser();
        String currentDir = Settings.getDefaultInputPath();
        fileChooser.setCurrentDirectory(new File(currentDir));
        File file = null;
        if (s==null) {
            if (fileChooser.showOpenDialog(frame) == JFileChooser.APPROVE_OPTION) {
                file = fileChooser.getSelectedFile();
            }
        }else {
            file = new File(s);
        }
        if (file == null) saveRules("default.clr");

        try {
            BufferedReader bufferedReader = new BufferedReader(new FileReader(file));
            String line =  bufferedReader.readLine();

            while (line!=null){
                String[] values = line.split(";");
                dictionary2.get(values[0]).setText(values[1]);
                line =  bufferedReader.readLine();


            }
            bufferedReader.close();;
        } catch (FileNotFoundException e) {
            saveRules("default.clr");
            loadRules("default.clsf");
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (NullPointerException e){
            e.printStackTrace();
        }
        return new ArrayList<>();
    }

    private static void saveRules(String s) {

    }
    public List<Rules> getRules() {
        return rules;
    }

    public void setRules(List<Rules> rules) {
        this.rules = rules;
    }

    public ClassifierWithRules() {
        operators = createOperators();
        createRules();

        Class<?> d = null;
        d = this.getClass();//;Class.forName("sample.RuleSet");
        c = new ClassChanger<ClassifierWithRules>(d,this);
        c.load("default.clsf");
        c.apply();
        rules = loadRules("default.clr");

        createWindow();


    }

    private void createWindow(Rules r, ArrayList<Rules> rules) {
        this.frame = new Frame("Rule based genetic classifier");

        JPanel imagePanel = new JPanel();

        JPanel rulePanel = new JPanel(new GridLayout(rules.size(), 1));

        JPanel ee = new JPanel(new GridLayout(5, 1));

        List<Operator> ops = getOperators();
        JPanel ruleButtonPanel = new JPanel(new GridLayout(ops.size(), 1));
        for (Operator o : ops) {
            JButton b = new JButton(o.getSymbol());
            b.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    Rules.addOrder(o);
                }
            });
            ruleButtonPanel.add(b);
        }
        for (Rules rule : rules) {
            TextField tf = new TextField(rule.getString());
        }

        frame.setLayout(new BorderLayout());


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
        ClassChanger<ClassifierWithRules> c;
        Class<?> d = null;

        d = this.getClass();//;Class.forName("sample.RuleSet");

        c = new ClassChanger<ClassifierWithRules>(d,this);
        c.save("default.clsf");
    }

    @Override
    public void load(String path) throws IOException {
        ClassChanger<ClassifierWithRules> c;
        Class<?> d = null;

        d = this.getClass();//;Class.forName("sample.RuleSet");

        c = new ClassChanger<ClassifierWithRules>(d,this);
        c.load("default.clsf");
    }

    public List<Operator> getOperators() {
        return operators;
    }

    public HashMap<ObjectType, Double> classify(double red, double green, double blue) {
        return null;
    }

    public HashMap<ObjectType, Double> preclassify(double red, double green, double blue) {

        return null;
    }

    public void editFields(BufferedImage bi) {

    }


    public enum RuleSetChoises {PvEDefault}
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
