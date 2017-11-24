package SpecialSettingsEtc;


import javax.swing.*;
import java.awt.*;
import java.io.*;
import java.lang.reflect.Field;
import java.util.Enumeration;
import java.util.Hashtable;

/**The function of this class is, to
 * have a easy accessable way to edit
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
    private final W object;


    public ClassChanger(Class<?> wClass, W object) {
        this.object = object;
        fields = wClass.getFields();
        dictionary = new Hashtable<>(fields.length);
        dictionary2 = new Hashtable<>(fields.length);
        getContentPane().setLayout(new BorderLayout());
        JPanel namePanel = new JPanel(); namePanel.setLayout(new BoxLayout(namePanel,BoxLayout.Y_AXIS));
        JPanel editPanel = new JPanel(); editPanel.setLayout(new BoxLayout(editPanel,BoxLayout.Y_AXIS));

        for (Field field : fields){
            try {

                String name = field.toString();
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

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }

        }

        JPanel buttonPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
        Button loadB = new Button("Load");
        loadB.addActionListener(e->load());
        Button saveB = new Button("Save");
        saveB.addActionListener(e->save());
        Button applyB = new Button("Apply");
        saveB.addActionListener(e->apply());
        buttonPanel.add(loadB);buttonPanel.add(saveB);buttonPanel.add(applyB);
        this.add(buttonPanel,BorderLayout.SOUTH);
        this.add(editPanel,BorderLayout.EAST);
        this.add(namePanel,BorderLayout.WEST);
        addController();

        pack();
        setVisible(true);
    }

    private void apply() {
        Enumeration fields = dictionary.elements();
        while (fields.hasMoreElements()){
            Field f = (Field) fields.nextElement();
            String value = dictionary2.get(f.getName()).getText();
            if (value == "wallrules" || value == "floorrules" || value == "robotrules"){
                continue;
            }
            try {

                f.set(object,Double.parseDouble(value));
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

    }

    private void addController() {
       // ButtonListener xbl = new XButtonListener();
        //Controller c1 = Controller.C1;
        //c1.buttonX.addButtonPressedListener(xbl);
    }

    private void save() {

        JFileChooser fileChooser = new JFileChooser();
        if (fileChooser.showSaveDialog(this) == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile();
            try {
                BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(file));
                Enumeration fields = dictionary.elements();
                while (fields.hasMoreElements()){
                    Field f = (Field) fields.nextElement();
                    String value = dictionary2.get(f.getName()).getText();
                     bufferedWriter.write(f.getName()+";"+value);
                }

            } catch (FileNotFoundException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }

        }

    }

    private void load() {
        JFileChooser fileChooser = new JFileChooser();
        if (fileChooser.showOpenDialog(this) == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile();
            try {
                BufferedReader bufferedReader = new BufferedReader(new FileReader(file));
                String[] values = bufferedReader.readLine().split(";");
                while (values!=null){
                   dictionary2.get(values[0]).setText(values[1]);
                   values = bufferedReader.readLine().split(";");


                }
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }


}
