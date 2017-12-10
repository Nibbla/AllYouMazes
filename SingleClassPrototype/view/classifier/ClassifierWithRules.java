package view.classifier;

import Interfaces.ObjectType;
import SpecialSettingsEtc.Archivar;
import SpecialSettingsEtc.ClassChanger;
import SpecialSettingsEtc.Settings;
import net.objecthunter.exp4j.Expression;
import net.objecthunter.exp4j.ExpressionBuilder;
import net.objecthunter.exp4j.operator.Operator;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.WritableRaster;
import java.io.*;
import java.lang.reflect.Field;
import java.util.*;
import java.util.List;

/**
 * Created by Nibbla on 21.10.2017.
 */
public class ClassifierWithRules implements IClassifier {

    //wall rules

    private static List<Operator> operators;

    ClassChanger<ClassifierWithRules> c;

    private Frame frame;
    private JTextField ruleTextBox;
    private JTextArea textArea;

    private JPanel rulePanel;
    private ImagePanel imagePanel;
    private ArrayList<String> rules = new ArrayList<>(10);
    private boolean testRule = false;
    private boolean applyAllRules = false;
    int scansize = Settings.getScansize();

    public void updateImage(BufferedImage bi2) {
        imagePanel.setImage(bi2);

        imagePanel.repaint();

        frame.pack();
        frame.repaint();

    }
    //private Rule rules;
    //goal1 rules

    //goal2 rules
    private static class Rules {
        private int i;
        public static  int j;
        private int string;
        private  List<Rule> rules;



        public int getString() {
            return string;
        }



        public static ObjectType getObjectType(String ruleText, int rgb) {
            ArrayList<String> arrayList = new ArrayList<String>();
            arrayList.add(ruleText);
            return getObjectType(arrayList,rgb);
        }

        public static ObjectType getObjectType(ArrayList<String> rules, int rgb) {
            Color c = new Color(rgb);

            int[] scorebord = new int[ObjectType.values().length];
            float[] values = new float[3];
            Color.RGBtoHSB(c.getRed(),c.getGreen(),c.getBlue(),values);
            for (String rule : rules) {
                String[] splito = rule.split("-->");
                boolean doesAplly = doesRuleApply(values,c,splito[0]);
                String s;
                s = "\\*";
                String[] classPlust = splito[1].split(s);
                int val = 1;
                ObjectType ot;
                if (classPlust.length>1){
                    ot = ObjectType.valueOf(classPlust[1]);
                    val = Integer.decode(classPlust[0]);
                }else   ot = ObjectType.valueOf(classPlust[0]);

                if (doesAplly) {

                    scorebord[ot.ordinal()] += val;
                }
            }



            int s = scorebord.length;
            int maxInt = 0;
            int maxVal = scorebord[0];
            for (int i = 1; i < s; i++) {
                if (scorebord[i]>maxVal){
                    maxVal =scorebord[i];
                    maxInt = i;
                }
            }
            return ObjectType.values()[maxInt];
        }

        private static boolean doesRuleApply(float[] values, Color c, String s) {
            Expression ex = new ExpressionBuilder(s)
                    .variables("R", "G","B","Hue","Sat","Brg")
                    .operator(getOperators())
                    .build()
                    .setVariable("R", c.getRed())
                    .setVariable("G", c.getGreen())
                    .setVariable("B", c.getBlue())
                    .setVariable("Hue", values[0])
                    .setVariable("Sat", values[1])
                    .setVariable("Brg", values[2]);
            double result = ex.evaluate();
            //System.out.println(result);
            boolean isApply = (result == 1.0);
            return isApply;
        }

        private class Rule {

        }
    }

    private JTextField getRuleTextbox() {
        return ruleTextBox;
    }





    private static void saveRules(String s) {

    }
    public List<String> getRules() {
        return rules;
    }

    public void setRules(ArrayList<String> rules) {
        this.rules = rules;
    }
    private List<Operator> createOperators() {
        ArrayList<Operator> op = new ArrayList<>();
        op.add(new bigger(7));
        op.add(new biggerOrEqual(8));
        op.add(new smallerOrEqual(9));
        op.add(new smaller(10));
        op.add(new equal(6));
        op.add(new and(5));
        op.add(new or(4));
        op.add(new leadsToo(1));
        return op;
    }
    /**
     * Creates a window which shows us on the left side buttons to create rules and the rules and
     * on the right side, what happens if you apply them
     * @param b
     */
    public ClassifierWithRules(boolean b) {
        operators = createOperators();
        createRules();

        Class<?> d = null;
        d = this.getClass();//;Class.forName("sample.RuleSet");
       // c = new ClassChanger<ClassifierWithRules>(d,this);
      //  c.load("default.clsf");
      //  c.apply();
       // rules = loadRules("default.clr");

        createWindow();

        this.frame.setVisible(b);
        this.frame.repaint();

    }

    private void createWindow() {
        this.frame = new Frame("Rule based genetic classifier");
        this.frame.setPreferredSize(new Dimension(600,1024));
        this.ruleTextBox = new JTextField();
        this.textArea = new JTextArea("Hier könnte ihre Werbung stehen",10,30);
        textArea.setLineWrap(true);
        //textArea.setSize(200,600);
       // textArea.setPreferredSize(new Dimension(200,600));
       // textArea.setMinimumSize(new Dimension(200,600));
        imagePanel = new ImagePanel();



        JPanel leftPanel = new JPanel();
        BoxLayout gg = new BoxLayout(leftPanel,BoxLayout.Y_AXIS);
        leftPanel.setLayout(gg);
        JPanel rulePanel = createRulePanel();

        JPanel objectButtonPanel = createObjectButtonPanel();
        JPanel ruleOptionPanel = createRuleOptionPanel();
        JPanel saveLoadPanel = createSaveLoadPanel();
        leftPanel.add(ruleTextBox);

        leftPanel.add(rulePanel);

        addRuleButtonPanel(leftPanel);

        leftPanel.add(objectButtonPanel);

        leftPanel.add(ruleOptionPanel);

        leftPanel.add(textArea);

        leftPanel.add(saveLoadPanel);

        Box.Filler glue = (Box.Filler)Box.createVerticalGlue();
        glue.changeShape(glue.getMinimumSize(),
                new Dimension(0, Short.MAX_VALUE), // make glue greedy
                glue.getMaximumSize());
        leftPanel.add(glue);

        frame.setLayout(new BorderLayout());
        frame.add(leftPanel,BorderLayout.WEST);

        frame.add(imagePanel,BorderLayout.CENTER);
        frame.pack();


    }

    private JPanel createObjectButtonPanel() {
        JPanel jp = new JPanel();
        jp.setLayout(new BoxLayout(jp,BoxLayout.X_AXIS));
        for (ObjectType ot: ObjectType.values()){
            JButton otb = new JButton(ot.name());
            otb.addActionListener(e->{
                ruleTextBox.setText(ruleTextBox.getText()+ot.name());
            });
            jp.add(otb);
        }
        return jp;
    }

    private JPanel createSaveLoadPanel() {
        JPanel jp = new JPanel();
        jp.setLayout(new BoxLayout(jp,BoxLayout.X_AXIS));
        JButton save = new JButton("Save"); save.addActionListener(e -> {
            try {
                save(null);
            } catch (IOException e1) {
                e1.printStackTrace();
            }
        });

        JButton load = new JButton("Load"); load.addActionListener(e -> {
            try {
                load(null);
            } catch (IOException e1) {
                e1.printStackTrace();
            }
        });
        jp.add(save);
        jp.add(load);
        return jp;
    }

    private JPanel createRuleOptionPanel() {
        JPanel ruleOptionPanel = new JPanel();
        ruleOptionPanel.setLayout(new BoxLayout(ruleOptionPanel,BoxLayout.X_AXIS));


        JToggleButton b = new JToggleButton("TestRule");
        b.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                testRule = b.isSelected();
                imagePanel.setImage(null);
            }
        });
        ruleOptionPanel.add(b);

        JToggleButton b2 = new JToggleButton("Apply all Rules");
        b2.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                applyAllRules = b2.isSelected();
                imagePanel.setImage(null);
            }
        });
        ruleOptionPanel.add(b2);

        JButton b3 = new JButton("Add Rule");
        b3.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                String rule = ruleTextBox.getText();
                rules.add(rule);
                ruleTextBox.setText("");
                updateRulePanel();
            }
        });
        ruleOptionPanel.add(b3);

        return ruleOptionPanel;
    }

    private JPanel createRulePanel() {
        rulePanel = new JPanel();

        updateRulePanel();


        return rulePanel;
    }
    public static List<Operator> getOperators() {
        return operators;
    }
    private void updateRulePanel() {
        for (int i = rulePanel.getComponentCount()-1;i>=0;i--){
            rulePanel.remove(i);
        }
        if (rules == null) return;
        rulePanel.setLayout(new GridLayout(rules.size(),1));
      //  rulePanel.setPreferredSize();

        for (String rule : rules) {
            JPanel singleRule = new JPanel(); singleRule.setLayout(new BoxLayout(singleRule,BoxLayout.X_AXIS));
            TextField tf = new TextField(rule);
            JButton jb = new JButton("Remove");
          singleRule.add(jb);
            singleRule.add(tf);



            jb.addActionListener(e->{
                rules.remove(rule);
                updateRulePanel();
            });
            tf.setMaximumSize(new Dimension(300,80));
            rulePanel.add(singleRule);

        }
        frame.pack();
        frame.repaint();

    }

    private void addRuleButtonPanel(JPanel leftPanel) {
        List<Operator> ops = getOperators();
        JPanel ruleButtonPanel = new JPanel(); ruleButtonPanel.setLayout(new BoxLayout(ruleButtonPanel,BoxLayout.X_AXIS)); //by accident both are that size 8(+ColorChannel.values().length));

        for (ColorChannel cc : ColorChannel.values()){
            JButton b = new JButton(cc.getText());
            b.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    ruleTextBox.setText(ruleTextBox.getText()+cc.getText());
                }
            });
            ruleButtonPanel.add(b);
        }
        leftPanel.add(ruleButtonPanel);

         ruleButtonPanel = new JPanel(); ruleButtonPanel.setLayout(new BoxLayout(ruleButtonPanel,BoxLayout.X_AXIS)); //by accident both are that size 8(+ColorChannel.values().length));
        for (Operator o : ops) {
            JButton b = new JButton(o.getSymbol());
            b.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    ruleTextBox.setText(ruleTextBox.getText()+o.getSymbol());
                }
            });
            ruleButtonPanel.add(b);
        }
        leftPanel.add(ruleButtonPanel);
        ruleButtonPanel = new JPanel(new GridLayout(2, 5)); //by accident both are that size 8(+ColorChannel.values().length));
        for (int d = 1; d <=10; d+=1) {
            int val = d;
            JButton b = new JButton(val + "*");

            final int finalVal = val;
            b.addActionListener(new ActionListener() {
                final int finalVal2 = finalVal;
                @Override
                public void actionPerformed(ActionEvent e) {
                    ruleTextBox.setText(ruleTextBox.getText()+ finalVal2 + "*");
                }
            });
            ruleButtonPanel.add(b);
        }
        leftPanel.add(ruleButtonPanel);
        ruleButtonPanel = new JPanel(new GridLayout(4, 5)); //by accident both are that size 8(+ColorChannel.values().length));
        for (int d = -1; d <=255; d+=16) {
            int val = d;
            if (val == -1) val  = 0;
            JButton b = new JButton(val + "");

            final int finalVal = val;
            b.addActionListener(new ActionListener() {
                final int finalVal2 = finalVal;
                @Override
                public void actionPerformed(ActionEvent e) {
                    ruleTextBox.setText(ruleTextBox.getText()+ finalVal2);
                }
            });
            ruleButtonPanel.add(b);
        }
        leftPanel.add(ruleButtonPanel);

        ruleButtonPanel = new JPanel(new GridLayout(4, 5)); //by accident both are that size 8(+ColorChannel.values().length));
        for (double d = 0; d <=1.; d+=1./16.) {
            double val = d;

            JButton b = new JButton(val + "");

            final float finalVal = (float) val;
            b.addActionListener(new ActionListener() {
                final float finalVal2 = finalVal;
                @Override
                public void actionPerformed(ActionEvent e) {
                    ruleTextBox.setText(ruleTextBox.getText()+ finalVal2);
                }
            });
            ruleButtonPanel.add(b);
        }
        leftPanel.add(ruleButtonPanel);



    }

    private void createRules() {

    }







    @Override
    public void editFields() {

       c.setVisible(true);

    }

    @Override
    public void save(String s) throws IOException {
        JFileChooser fileChooser = new JFileChooser();
        String currentDir = Settings.getDefaultInputPath();
        fileChooser.setCurrentDirectory(new File(currentDir));
        File file = null;
        if (s==null) {
            if (fileChooser.showSaveDialog(frame) == JFileChooser.APPROVE_OPTION) {
                file = fileChooser.getSelectedFile();
            }
        }else {
            file = new File(s);
        }

        try {
            BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(file));
            for (String rule : rules){

                bufferedWriter.write(rule+"\n");
            }
            bufferedWriter.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        Archivar.shout("New Classifier Values Applied");

    }

    @Override
    public void load(String s) throws IOException {
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
        if (file == null) save("default" + Math.nextUp((float)Math.random()*100) + ".clr");

        try {
            BufferedReader bufferedReader = new BufferedReader(new FileReader(file));
            rules.clear();
            String line =  bufferedReader.readLine();

            while (line!=null){
                rules.add(line);
                line =  bufferedReader.readLine();
            }
            bufferedReader.close();;
            updateRulePanel();
        } catch (FileNotFoundException e) {
            save("default.clr");
            load("default.clr");
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (NullPointerException e){
            e.printStackTrace();
        }
    }



    public HashMap<ObjectType, Double> classify(double red, double green, double blue) {
        return null;
    }

    public HashMap<ObjectType, Double> preclassify(double red, double green, double blue) {

        return null;
    }





//https://lallafa.objecthunter.net/exp4j/#Custom_operators

    private class leadsToo extends Operator {
        public leadsToo(int precedence) {
            super("-->", 2, true, precedence);
        }

        @Override
        public double apply(double... doubles) {
            final double arg1 =  doubles[0];
            final double arg2 = doubles[1];
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
            final double arg1 =  doubles[0];
            final double arg2 =  doubles[1];
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
            final double arg1 = doubles[0];
            final double arg2 =  doubles[1];
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
            final double arg1 =  doubles[0];
            final double arg2 =  doubles[1];
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
            final double arg1 = doubles[0];
            final double arg2 =  doubles[1];
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
            final double arg1 =  doubles[0];
            final double arg2 =  doubles[1];
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
            final double arg1 = doubles[0];
            final double arg2 =  doubles[1];
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
            final double arg1 =  doubles[0];
            final double arg2 =  doubles[1];
            if ((arg1>0) || (arg2>0)) return 1;
            else return 0;
        }
    }




    private class ImagePanel extends JPanel{
        private JLabel imageP;
        private BufferedImage image;
        private BufferedImage imageCopy;
        private BufferedImage scaledImage;

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            if (scaledImage!=null) g.drawImage(scaledImage, 0, 0, null);
            g.dispose();
        }


        public ImagePanel() {

            this.setLayout(new BorderLayout());
           // this.imageP = new JLabel();

            //this.add(imageP);

            this.addMouseListener(createMouseListener());
            addResizeListener();
            repaint();
        }

        private void addResizeListener() {
            this.addComponentListener(new ComponentListener() {
                public void componentResized(ComponentEvent e) {
                    setImage(image);
                }

                @Override
                public void componentMoved(ComponentEvent e) {

                }

                @Override
                public void componentShown(ComponentEvent e) {

                }

                @Override
                public void componentHidden(ComponentEvent e) {

                }
            });
        }

        public void setImage(BufferedImage image) {
            if (image != null)this.image = image;
            if (this.image==null) return;
            this.imageCopy = deepCopy(this.image);
            int width = this.getWidth();
            if (width <= 2) width = 400;
                width = Math.min(400,width);
            int height = this.getHeight();
            if (height <= 2) height = 600;
            height = Math.min(600,height);

            scaledImage = new BufferedImage(width, height, this.image.getType());
           //
            width = 200;
            height = 400;
            Graphics2D graphics2D = scaledImage.createGraphics();

            graphics2D.drawImage(this.imageCopy, 0, 0, width, height, null);
            applyRules(scaledImage);

            graphics2D.dispose();



            this.setMinimumSize(new Dimension(800,500));
            this.setPreferredSize(new Dimension(800,500));
           // frame.pack();
            repaint();

        }

        public BufferedImage deepCopy(BufferedImage bi) {
            ColorModel cm = bi.getColorModel();
            boolean isAlphaPremultiplied = cm.isAlphaPremultiplied();
            WritableRaster raster = bi.copyData(bi.getRaster().createCompatibleWritableRaster());
            return new BufferedImage(cm, raster, isAlphaPremultiplied, null);
        }

        public ObjectType[][] applyRules(BufferedImage scaledImage) {
            Graphics2D graphics2D = scaledImage.createGraphics();
            ObjectType[][] otf = null;
            if (testRule) {
                int widht = scaledImage.getWidth();
                int height = scaledImage.getHeight();
               otf = new ObjectType[widht][height];

                for (int x = 0; x < widht; x++) {
                    for (int y = 0; y < height; y++) {
                        ObjectType ot = Rules.getObjectType(ruleTextBox.getText(),scaledImage.getRGB(x,y));

                        if (ot == ObjectType.placeholder) {
                            otf[x][y] = ObjectType.floor;
                        }
                        if (ot != ObjectType.floor) {
                            otf[x][y] =ot;
                            graphics2D.setColor(ot.getColorC());
                            graphics2D.fillRect(x, y, 1, 1);
                        }

                    }

                }
            }
            if (applyAllRules){
                int widht = scaledImage.getWidth();
                int height = scaledImage.getHeight();
                otf = new ObjectType[widht][height];

                for (int x = 0; x < widht; x++) {
                    for (int y = 0; y < height; y++) {
                        ObjectType ot = Rules.getObjectType(rules,scaledImage.getRGB(x,y));


                        if (ot == ObjectType.placeholder) {
                            otf[x][y] = ObjectType.floor;
                        }
                        if (ot != ObjectType.floor) {
                            otf[x][y] =ot;
                            graphics2D.setColor(ot.getColorC());
                            graphics2D.fillRect(x, y, 1, 1);
                        }

                    }

                }
            }
            return otf;
        }

        private MouseListener createMouseListener() {
            MouseListener ml = new MouseListener() {
                @Override
                public void mouseClicked(MouseEvent e) {
                    int x = e.getX();
                    int y = e.getY();
                    Color c = new Color(scaledImage.getRGB(x,y));

                    float[] values = new float[3];
                    Color.RGBtoHSB(c.getRed(),c.getGreen(),c.getBlue(),values);
                    textArea.setText("");

                    StringBuilder sb = new StringBuilder();
                   sb.append("R: " + c.getRed()+" ");
                    sb.append("G: " + c.getGreen()+" ");
                    sb.append("B: " + c.getBlue()+"\n");
                    float rg = (float)c.getRed()/(float)c.getGreen();
                    sb.append("R/G: " + rg +" ");

                    float rb = (float)c.getRed()/(float)c.getBlue();
                    sb.append("R/B: " + rb +" ");

                    float gb = (float)c.getGreen()/(float)c.getBlue();
                    sb.append("G/B: " + gb +"\n");

                    sb.append("Hue: " + values[0]+" ");
                    sb.append("Sat: " + values[1]+" ");
                    sb.append("Brg: " + values[2]+" ");
                    textArea.setText(sb.toString());
                }

                @Override
                public void mousePressed(MouseEvent e) {

                }

                @Override
                public void mouseReleased(MouseEvent e) {

                }

                @Override
                public void mouseEntered(MouseEvent e) {

                }

                @Override
                public void mouseExited(MouseEvent e) {

                }
            };
            return ml;
        }


    }


}
