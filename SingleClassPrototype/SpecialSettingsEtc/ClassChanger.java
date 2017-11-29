package SpecialSettingsEtc;


import javax.swing.*;
import java.awt.*;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.io.*;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Hashtable;

/**The function of this class is, to
 * have a easy accessable way to editFields
 * all public fields of a class.
 * The current tradeoff is,
 * that public variables are needed for this.
 *
 * Created by Nibbla on 21.10.2017.
 */
public class ClassChanger<W> extends JFrame{

    private final Field[] fields;
    private final Hashtable<String, Field> dictionary;
    private final Hashtable<String, TextField> dictionary2;
    private final Hashtable<String, JLabel> dictionary3Labels;
    private final W object;
    private int blocks;
    private final ArrayList<String> blocknames;


    public ClassChanger(Class<?> wClass, W object) {
        this.object = object;
        fields = wClass.getFields();
        dictionary = new Hashtable<>(fields.length);
        dictionary2 = new Hashtable<>(fields.length);
        dictionary3Labels = new Hashtable<>(fields.length);
        getContentPane().setLayout(new BorderLayout());
        JPanel mainPanel = new JPanel(); mainPanel.setLayout(new BoxLayout(mainPanel,BoxLayout.X_AXIS));


        JPanel namePanel = null;
        JPanel editPanel = null;
         blocks = 0;
        blocknames = new ArrayList<>();
        for (Field field : fields) {
            try {
                String value = field.get(object).toString();
                double d = Double.parseDouble(value);

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            } catch (NumberFormatException e) {
                blocks++;
                try {
                    blocknames.add(field.get(object).toString());
                } catch (IllegalAccessException e1) {
                    e1.printStackTrace();
                }
            }
        }








        for (Field field : fields){
            try {

                String name = field.getName();
                for (int i = 0; i < blocks; i++) {
                    if (blocknames.get(i).equals(name)){
                        namePanel = new JPanel(); namePanel.setLayout(new BoxLayout(namePanel,BoxLayout.Y_AXIS));
                        editPanel = new JPanel(); editPanel.setLayout(new BoxLayout(editPanel,BoxLayout.Y_AXIS));
                        mainPanel.add(namePanel);
                        mainPanel.add(editPanel);
                    }
                }


                JLabel nameL = new JLabel(name);
                namePanel.add(nameL);
                String value = field.get(object).toString();
                TextField t = new TextField(value);
                //t.setPreferredSize(new Dimension(t.getPreferredSize().width, (int) nameL.getPreferredSize().getHeight()));
                //t.setMaximumSize(new Dimension(20, (int) nameL.getMaximumSize().getHeight()));
                nameL.setPreferredSize(new Dimension(nameL.getPreferredSize().width, (int) t.getPreferredSize().getHeight()));
                nameL.setMaximumSize(new Dimension(nameL.getPreferredSize().width, (int) t.getMaximumSize().getHeight()));
                editPanel.add(t);
                dictionary.put(name,field);
                dictionary2.put(name,t);
                dictionary3Labels.put(name,nameL);

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }

        }
        this.addComponentListener(new ResizeListener(this));
        JPanel buttonPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
        Button loadB = new Button("Load");
        loadB.addActionListener(e->load(null));
        Button saveB = new Button("Save");
        saveB.addActionListener(e->save(null));
        Button applyB = new Button("Apply");
        applyB.addActionListener(e->apply());
        buttonPanel.add(loadB);buttonPanel.add(saveB);buttonPanel.add(applyB);
        this.add(buttonPanel,BorderLayout.SOUTH);
        this.add(mainPanel,BorderLayout.CENTER);
        //this.add(namePanel,BorderLayout.WEST);
        addController();

        pack();


    }

    public void apply() {
        Enumeration fields = dictionary.elements();
        WHILUST:while (fields.hasMoreElements()){
            Field f = (Field) fields.nextElement();
            String name = f.getName();
            String value = dictionary2.get(name).getText();
            for (int i = 0; i < blocks; i++) {
                if (blocknames.get(i).equals(name)){
                   continue WHILUST;
                }
            }

            try {
                if (f.getName().contains("wRedOgreenLowBound")){
                    System.out.println(Classifier.wRedOgreenLowBound);}
                f.set(object,Double.parseDouble(value));
                if (f.getName().contains("wRedOgreenLowBound")){
                System.out.println(Classifier.wRedOgreenLowBound);}
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        Archivar.shout("New Classifier Values Applied");
    }

    private void addController() {
       // ButtonListener xbl = new XButtonListener();
        //Controller c1 = Controller.C1;
        //c1.buttonX.addButtonPressedListener(xbl);
    }

    public void save(String s) {

        JFileChooser fileChooser = new JFileChooser();
        String currentDir = Settings.getDefaultInputPath();
        fileChooser.setCurrentDirectory(new File(currentDir));
        File file = null;
        if (s==null) {
            if (fileChooser.showSaveDialog(this) == JFileChooser.APPROVE_OPTION) {
                file = fileChooser.getSelectedFile();
            }
        }else {
            file = new File(s);
        }

            try {
                BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(file));
                Enumeration fields = dictionary.elements();
                while (fields.hasMoreElements()){
                    Field f = (Field) fields.nextElement();
                    String value = dictionary2.get(f.getName()).getText();
                     bufferedWriter.write(f.getName()+";"+value+"\n");
                }
                bufferedWriter.close();
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }

        Archivar.shout("New Classifier Values Applied");

    }

    public void load(String s) {
        JFileChooser fileChooser = new JFileChooser();
        String currentDir = Settings.getDefaultInputPath();
        fileChooser.setCurrentDirectory(new File(currentDir));
        File file = null;
        if (s==null) {
            if (fileChooser.showOpenDialog(this) == JFileChooser.APPROVE_OPTION) {
                file = fileChooser.getSelectedFile();
            }
        }else {
            file = new File(s);
        }
           if (file == null) save("default.clsf");

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
                save("default.clsf");
                load("default.clsf");
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            } catch (NullPointerException e){
                e.printStackTrace();
            }


    }



    private class ResizeListener implements ComponentListener {
        private final Frame frame;
        private boolean isResizing = false;

        public ResizeListener(Frame f) {
            this.frame = f;
        }

        @Override
        public void componentResized(ComponentEvent e) {
            if (!isResizing) return;
            if (isResizing) return;

            isResizing=true;
            Enumeration fields = dictionary.elements();
            while (fields.hasMoreElements()) {
                Field f = (Field) fields.nextElement();
                String name = f.getName();
                TextField textField = dictionary2.get(name);
                JLabel label = dictionary3Labels.get(name);
                textField.setPreferredSize(new Dimension(textField.getPreferredSize().width, (int) label.getHeight()));
               textField.setMaximumSize(new Dimension(textField.getPreferredSize().width, (int) label.getHeight()));
              // textField.setSize(new Dimension(textField.getPreferredSize().width, (int) label.getHeight()));
                //textField.setBounds(textField.getX(),label.getY(),textField.getWidth(),label.getHeight());
                Font font = textField.getFont();
                Font newFont = new Font(font.getName(),font.getStyle(),label.getHeight()-4);
                textField.setFont(newFont);
            }
            isResizing=false;
            pack();
            repaint();
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
    }
}
