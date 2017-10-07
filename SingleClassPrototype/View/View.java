package View;

import Interfaces.IView;
import Interfaces.ObjectType;
import SpecialSettingsEtc.Archivar;
import SpecialSettingsEtc.Settings;
import javafx.collections.transformation.SortedList;
import model.Model;
import model.SpecialGraph;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;

/**
 * Created by Nibbla on 27.09.2017.
 */
public class View implements IView {
    private static View factoryView = new View();
    private JFrame frame;

    public View() {

    }

    public View getInstance() throws CloneNotSupportedException {
        return (View) factoryView.clone();
    }

    @Override
    public BufferedImage getCurrentShot() {
        long tick1 = System.currentTimeMillis();


        String path = SpecialSettingsEtc.Settings.getInputPath();
        Archivar.shout("Loading image at time " + System.currentTimeMillis());
        Archivar.shout("at path " + path);
        Archivar.shout("Working Directory = " +
                System.getProperty("user.dir"));

        BufferedImage bi = null;
        try {

            bi = ImageIO.read(new File(path));

        } catch (IOException e) {
            Archivar.shout("Error whilst loading image from " + path);
            Archivar.shout("loading image from " + Settings.getDefaultInputPath() + " instead");
            try {
                bi = ImageIO.read(new File(Settings.getDefaultInputPath()));
            } catch (IOException e1) {
                Archivar.shout("Error whilst loading image from " + Settings.getDefaultInputPath());
                e1.printStackTrace();
            }
            e.printStackTrace();
        }

        long tick2 = System.currentTimeMillis();
        Archivar.shout("Loading image took " + (tick2 - tick1)/1000. + " seconds to complete");

        return bi;
    }

    @Override
    public Model getClassifiedModel() {
        return null;
    }

    @Override
    public SpecialGraph getGraph() {
        return null;
    }

    @Override
    public Model classify(BufferedImage bi) {
        long tick1 = System.currentTimeMillis();
        int width = bi.getWidth();
        int heigth = bi.getHeight();
        int stepX = Settings.getStepX();
        int stepY = Settings.getStepY();
        int scansize = Settings.getScansize();
        int medianfilterSize = Settings.getMedianFilterSize(3);

        long tick3 = System.currentTimeMillis();
        BufferedImage b2 = new BufferedImage((int)Math.ceil(bi.getWidth()/scansize),(int)Math.ceil(bi.getHeight()/scansize),bi.getType());

        //Step1 reduce resolution
        int xcount = 0;
        int ycount = 0;
        for (int x = scansize/2; x < width && xcount < b2.getWidth(); x+=scansize) {
            for (int y = scansize/2; y < heigth &&ycount <b2.getHeight(); y+=scansize) {
                //Archivar.shout(xcount + " "+ ycount + " " + x + "  :" + y + "max" + width + " " + heigth);
               // if (ycount == 21){
               //     System.out.println("ee");
                //    int i = bi.getRGB(x,y);
               // }
                b2.setRGB(xcount,ycount,bi.getRGB(x,y));
                ycount++;
            }
            xcount++;
            ycount=0;
        }
        long tick4 = System.currentTimeMillis();
        Archivar.shout("Downsizing image took " + (tick4 - tick3)/1000. + " seconds to complete");

        //Step2 simple redmedian filter

        int smallStep = (int) Math.floor(medianfilterSize / 2);
        int width2 = b2.getWidth();
        int heigth2 = b2.getHeight();
        long tick5 = System.currentTimeMillis();
        for (int x = smallStep+1; x < width; x+=medianfilterSize) {
            for (int y = smallStep+1; y < heigth; y+=medianfilterSize) {

                    int count = 0;

                int[] vals = new int[medianfilterSize*medianfilterSize];
                    for (int xsmall = x-smallStep-1; xsmall < x+smallStep && xsmall < width2; xsmall++) {
                        for (int ysmall = y-smallStep-1; ysmall < y+smallStep && ysmall < heigth2&& count<vals.length; ysmall++) {

                            vals[count] = b2.getRGB(xsmall,ysmall);


                            count++;
                        }
                    }


                Arrays.sort(vals);
                int redMedian = vals[medianfilterSize/2];
                for (int xsmall = x-smallStep-1; xsmall < x+smallStep && xsmall < width2; xsmall++) {
                    for (int ysmall = y-smallStep-1; ysmall < y+smallStep && ysmall < heigth2; ysmall++) {
                        b2.setRGB(xsmall,ysmall,redMedian);
                    }
                }



            }
        }
        long tick6 = System.currentTimeMillis();
        Archivar.shout("Medianfilter image took " + (tick6 - tick5)/1000. + " seconds to complete");

        //Step3 simple redmedian filter

        int maximumfilterSize = Settings.getMaximumFilterSize();
        smallStep = (int) Math.floor(maximumfilterSize / 2);
        width2 = b2.getWidth();
        heigth2 = b2.getHeight();
        long tick7 = System.currentTimeMillis();
        for (int x = smallStep+1; x < width; x+=maximumfilterSize) {
            for (int y = smallStep+1; y < heigth; y+=maximumfilterSize) {

                int count = 0;

                int[] vals = new int[maximumfilterSize*maximumfilterSize];
                for (int xsmall = x-smallStep-1; xsmall < x+smallStep && xsmall < width2; xsmall++) {
                    for (int ysmall = y-smallStep-1; ysmall < y+smallStep && ysmall < heigth2; ysmall++) {

                        vals[count] = b2.getRGB(xsmall,ysmall);


                        count++;
                    }
                }


                Arrays.sort(vals);
                int redMedian =vals[maximumfilterSize-1]; // vals[0];//
                for (int xsmall = x-smallStep-1; xsmall < x+smallStep && xsmall < width2; xsmall++) {
                    for (int ysmall = y-smallStep-1; ysmall < y+smallStep && ysmall < heigth2; ysmall++) {
                        b2.setRGB(xsmall,ysmall,redMedian);
                    }
                }



            }
        }
        long tick8 = System.currentTimeMillis();
        Archivar.shout("Maximumfilter image took " + (tick8 - tick7)/1000. + " seconds to complete");


        int smallStepX = (int) Math.floor(stepX/2.);
        int smallStepY = (int) Math.floor(stepY/2.);



        //blockwiseprocessing
        PixelObjectType[][] ot = new PixelObjectType[width2][heigth2];
        for (int x = 0; x < width2; x+=1) {
            for (int y = 0; y < heigth2; y+=1) {
                ot[x][y] = new PixelObjectType(x, y, b2);

            }

        }


        for (int x = 0; x < width2; x+=1) {
            for (int y = 0; y < heigth2; y+=1) {

                double[] counter = new double[ObjectType.values().length];
                for (int i = 0; i < counter.length; i++) {
                    counter[i] = 0;
                }

                for (int x2 = x-2; x2 <= x+2 ; x2+=1) {
                    for (int y2 = y-2; y2 <= y+2 ; y2 += 1) {
                        if (x2<0 || y2<0|| x2>=width2||y2>=heigth2) continue;
                        double d = Math.sqrt((x2-x)*(x2-x)+ (y2-y)*(y2-y));
                        double faktor = 1/16.*0.3;
                        if (d < 1.5) faktor = 1/8.*0.5;
                        if (d < 1) faktor = 1;

                        counter[ot[x2][y2].getSelectedClass().ordinal()] += faktor;
                    }
                }

                int maxAt = 0;

                for (int i = 0; i < counter.length; i++) {
                    //Archivar.shout(i + "  " + counter[i]);
                    maxAt = counter[i] > counter[maxAt] ? i : maxAt;
                }
                ot[x][y].classifiedAs(maxAt);
            }

        }
        BufferedImage b3 = new BufferedImage(b2.getWidth(),b2.getHeight(),b2.getType());
        for (int x = 0; x < b3.getWidth(); x++) {
            for (int y = 0; y < b3.getHeight(); y++) {
                b3.setRGB(x,y,ot[x][y].getSelectedClass().getColor());
            }

        }

        long tick2 = System.currentTimeMillis();
        Archivar.shout("Classifying image took " + (tick2 - tick1)/1000. + " seconds to complete");
        if (frame!=null) frame.setVisible(false);
        frame = new JFrame();
        frame.getContentPane().setLayout(new FlowLayout());
        frame.getContentPane().add(new JLabel(new ImageIcon(b3)));
       // frame.getContentPane().add(new JLabel(new ImageIcon(b2)));

        frame.pack();
        frame.setVisible(true);

        return null;
    }

    private void calcBlock(BufferedImage bi, int x, int y, int width, int heigth, int stepX, int stepY, int smallStepX, int smallStepY) {

        //first

        //get the average
        double val = bi.getRGB(x,y);
        double valueR = val / (16*16);
        val = val % (16*16);
        double valueG = val/(16*16);
        val = val % (16*16);
        double valueB = val;

        int count = 1;
        for (int xsmall = Math.max(x-smallStepX,0); xsmall < x+smallStepX && xsmall < width; xsmall++) {
            for (int ysmall = Math.max(y-smallStepY,0); ysmall < y+smallStepY && ysmall < heigth; ysmall++) {
                val = bi.getRGB(xsmall,ysmall);
                valueR += val / (16*16);
                val = val % (16*16);
                valueG += val/(16*16);
                val = val % (16*16);
                valueB += val;
                count++;
            }
        }
        valueR /= count;
        valueG /= count;
        valueB /= count;


        Archivar.shout("Value at " + x + "  " + y + " is R:" + valueR + " G:" + valueG + " B:" + valueB);
    }



    @Override
    public Model classify() {
        return classify(this.getCurrentShot());
    }

    public class Pixel{

    }

    public class Workflow{

    }


    private class PixelObjectType {

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


            wallility = wc[0] * red + wc[1] * green + wc[2] * blue;
            floorility = wc[3] * red + wc[4] * green + wc[5] * blue;
            robotility = wc[6] * red + wc[7] * green + wc[8] * blue;

        }

        public ObjectType getSelectedClass() {
            if (wallility > floorility && wallility > robotility) return ObjectType.wall;
            if (floorility > robotility) return ObjectType.floor;
            return ObjectType.robot;

        }

        public void classifiedAs(int i) {
            this.classifiedas = ObjectType.values()[i];
        }
    }
}