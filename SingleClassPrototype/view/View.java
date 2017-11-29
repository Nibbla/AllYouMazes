package view;

import Interfaces.IView;
import Interfaces.ObjectType;
import Model.DijkstraPathFinder;
import SpecialSettingsEtc.Archivar;
import SpecialSettingsEtc.Classifier;
import SpecialSettingsEtc.Settings;
import Model.ModelDeprecated;
import Model.RoboPos;
import javafx.util.Pair;


import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Stack;

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


        String path = Settings.getInputPath();
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
    public ModelDeprecated getClassifiedModel() {
        return null;
    }

    @Override
    public DijkstraPathFinder getGraph(ObjectType[][] g, int imageType, RoboPos roboPos, int graphSkip, boolean showPathway) {




        return new DijkstraPathFinder(g,imageType,roboPos,graphSkip,showPathway);
    }

    @Override
    public ObjectType[][] classify(BufferedImage bi, boolean showKlassification, Classifier classifier) {
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
                ot[x][y] = new PixelObjectType(x, y, b2,classifier);

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

        //errode
        ObjectType[][] ot4 = new ObjectType[width2][heigth2];
        ot4 = extractObjectType(ot);
      // ot = erode(ot,ObjectType.robot,ObjectType.floor,width2,heigth2);
       // ot = erode(ot,ObjectType.robot,ObjectType.floor,width2,heigth2);
       // ot = erode(ot,ObjectType.robot,ObjectType.floor,width2,heigth2);


        ot4 = erode(ot4,ObjectType.wall,ObjectType.floor,width2,heigth2);
        ot4 = erode(ot4,ObjectType.wall,ObjectType.floor,width2,heigth2);
        ot4 = erode(ot4,ObjectType.wall,ObjectType.floor,width2,heigth2);
        ot4 = erode(ot4,ObjectType.wall,ObjectType.floor,width2,heigth2);
        ot4 = dialate(ot4,ObjectType.wall,width2,heigth2);
        ot4 = dialate(ot4,ObjectType.wall,width2,heigth2);
        ot4 = dialate(ot4,ObjectType.wall,width2,heigth2);
        ot4 = dialate(ot4,ObjectType.wall,width2,heigth2);
        ot4 = dialate(ot4,ObjectType.wall,width2,heigth2);
        ot4 = dialate(ot4,ObjectType.wall,width2,heigth2);

         ot4 = erode(ot4,ObjectType.robot,ObjectType.floor,width2,heigth2);
         ot4 = erode(ot4,ObjectType.robot,ObjectType.floor,width2,heigth2);
         ot4 = erode(ot4,ObjectType.robot,ObjectType.floor,width2,heigth2);
        ot4 = dialate(ot4,ObjectType.robot,width2,heigth2);
        ot4 = dialate(ot4,ObjectType.robot,width2,heigth2);
        ot4 = dialate(ot4,ObjectType.robot,width2,heigth2);
        ot4 = dialate(ot4,ObjectType.robot,width2,heigth2);
        ot4 = dialate(ot4,ObjectType.robot,width2,heigth2);
        ot4 = erode(ot4,ObjectType.robot,ObjectType.floor,width2,heigth2);
        ot4 = erode(ot4,ObjectType.robot,ObjectType.floor,width2,heigth2);

        long tick2 = System.currentTimeMillis();
        Archivar.shout("Classifying image took " + (tick2 - tick1)/1000. + " seconds to complete");

        if (showKlassification)
        showClassification(b2, ot4);

        return ot4;
    }

    private void showClassification(BufferedImage b2, ObjectType[][] ot4) {
        BufferedImage b3 = new BufferedImage(b2.getWidth(),b2.getHeight(),b2.getType());
        for (int x = 0; x < b3.getWidth(); x++) {
            for (int y = 0; y < b3.getHeight(); y++) {
                if (ot4[x][y]!=ObjectType.floor)b3.setRGB(x,y,ot4[x][y].getColor());
                else {
                    b3.setRGB(x,y,b2.getRGB(x,y));
                }
            }

        }


        if (frame!=null) frame.setVisible(false);
        frame = new JFrame();
        frame.getContentPane().setLayout(new FlowLayout());
        frame.getContentPane().add(new JLabel(new ImageIcon(b3)));
        // frame.getContentPane().add(new JLabel(new ImageIcon(b2)));

        frame.pack();
        frame.setFocusableWindowState(false);
        frame.setVisible(true);
        frame.setFocusableWindowState(true);
    }


    private ObjectType[][] extractObjectType(PixelObjectType[][] ot) {
        ObjectType[][] ot3 = new ObjectType[ot.length][ot[0].length];

        for (int i = 0; i < ot.length; i++) {
            for (int j = 0; j < ot[0].length; j++) {
                ot3[i][j] = ot[i][j].getSelectedClass();
            }
        }


        return ot3;
    }

    private ObjectType[][] erode(ObjectType[][] ot2, ObjectType typeToErrode, ObjectType typeToReplace, int width2, int heigth2) {
        ObjectType[][] ot3 = new ObjectType[ot2.length][ot2[0].length];

        for (int x = 0; x < width2; x+=1) {
            for (int y = 0; y < heigth2; y += 1) {
                ot3[x][y] = ot2[x][y];
                if (ot2[x][y] != typeToErrode){
                    continue;
                }
                ArrayList<Integer[]> l = getNeighbours(x,y,width2,heigth2);
                for (Integer[] c : l){
                    if (typeToErrode != ot2[c[0]][c[1]]){
                        ot3[x][y] = typeToReplace;
                        break;
                    };
                }
            }
        }

        return  ot3;
    }

    public static ObjectType[][] dialate(ObjectType[][] ot2, ObjectType typeToFill, int width2, int heigth2) {
        ObjectType[][] ot3 = new ObjectType[ot2.length][ot2[0].length];

        for (int x = 0; x < width2; x+=1) {
            for (int y = 0; y < heigth2; y += 1) {
                ot3[x][y] = ot2[x][y];
                if (ot2[x][y] == typeToFill){
                    continue;
                }
                ArrayList<Integer[]> l = getNeighbours(x,y,width2,heigth2);
                for (Integer[] c : l){
                    if (typeToFill == ot2[c[0]][c[1]]){
                        ot3[x][y]=typeToFill;
                        break;
                    };
                }
            }
        }

        return  ot3;
    }

    private static ArrayList<Integer[]> getNeighbours(int x, int y, int width2, int heigth2) {
        ArrayList<Integer[]> neighbours = new ArrayList<>(8);
        if (x-1>=0&&y-1>=0) neighbours.add(new Integer[]{x-1,y-1});
        if (x-1>=0&&y>=0) neighbours.add(new Integer[]{x-1,y});
        if (x-1>=0&&y+1<heigth2) neighbours.add(new Integer[]{x-1,y+1});

        if (x+1<width2&&y-1>=0) neighbours.add(new Integer[]{x+1,y-1});
        if (x+1<width2&&y>=0) neighbours.add(new Integer[]{x+1,y});
        if (x+1<width2&&y+1<heigth2) neighbours.add(new Integer[]{x+1,y+1});

       // if (y-1>=0) neighbours.add(new Integer[]{x,y-1});
       // if (y+1<=heigth2) neighbours.add(new Integer[]{x,y+1});

        return neighbours;
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

    public ArrayList<RoboPos> getRobotCenter(ObjectType[][] m2, int numberOfPixelsToSkip) {
        ArrayList<ArrayList<Pair<Integer, Integer>>> clusters = new ArrayList<>();
        int cpimt = 0;
        //Checking every pixel takes way too much computation time, I tested with skipping 50, works fine.

        getClustersNew(m2, numberOfPixelsToSkip, clusters, cpimt);
       // getClusters(m2, numberOfPixelsToSkip, clusters, cpimt);




        //Extrapolating robot position from the largest cluster that we found (we assume the largest is the robot)
        ArrayList<Pair<Integer, Integer>> largestCluster = new ArrayList<>();
        ArrayList<RoboPos> allRobots = new ArrayList<>();


        while (clusters.size()!= 0) {
            int max = 0;
            int maxIndex = 0;
            for (int i = 0; i < clusters.size(); i++) {
                if (clusters.get(i).size() > max) {
                    largestCluster = clusters.get(i);
                    max = clusters.get(i).size();
                    maxIndex = i;
                }
            }

            double xSum = 0;
            double ySum = 0;

            for (Pair<Integer, Integer> p : largestCluster) {
                xSum += p.getKey();
                ySum += p.getValue();
            }

            double x = xSum / largestCluster.size();
            double y = ySum / largestCluster.size();

            allRobots.add(new RoboPos(x, y, Math.sqrt(largestCluster.size()) * 1.1284 * 0.5));
            clusters.remove(maxIndex);
        }
        return allRobots;
    }

    private void getClustersNew(ObjectType[][] m2, int numberOfPixelsToSkip, ArrayList<ArrayList<Pair<Integer, Integer>>> clusters, int cpimt) {
        ObjectType[][] m3 = new ObjectType[m2.length][m2[0].length];
        int   xLength = m2.length;
        int   yLength = m2[0].length;
        for (int i = 0; i < xLength; i++) {
            System.arraycopy(m2[i],0,m3[i],0,xLength);
        }

        for (int i = 0; i < xLength; i++) {
            for (int j = 0; j < yLength; j++) {
                if (m3[i][j] == ObjectType.robot){
                    ArrayList<Pair<Integer, Integer>> cluster = getClusterFrom(m3, i, j,xLength,yLength,numberOfPixelsToSkip);
                    clusters.add(cluster);
                }
            }
        }

    }

    private ArrayList<Pair<Integer, Integer>> getClusterFrom(ObjectType[][] m3, int i, int j,int width, int height,int stepsize) {
        ArrayList<Pair<Integer, Integer>> cluster = new ArrayList<>(100);
        if (i<0||i>=width||j<0||j>=height) return cluster;
        if (m3[i][j]!=ObjectType.robot) return cluster;


        m3[i][j]=ObjectType.floor;
        cluster.add(new Pair<>(i,j));

        cluster.addAll(getClusterFrom(m3, i+stepsize, j,width,height,stepsize));
        cluster.addAll(getClusterFrom(m3, i-stepsize, j,width,height,stepsize));
        cluster.addAll(getClusterFrom(m3, i, j+stepsize,width,height,stepsize));
        cluster.addAll(getClusterFrom(m3, i, j-stepsize,width,height,stepsize));

        return cluster;
    }

    private void getClusters(ObjectType[][] m2, int numberOfPixelsToSkip, ArrayList<ArrayList<Pair<Integer, Integer>>> clusters, int cpimt) {
        for (int i = 0; i < m2.length; i+=numberOfPixelsToSkip) {
            for (int j = 0; j < m2[i].length; j+=numberOfPixelsToSkip) {
                Archivar.shout(i + " " + j +" :RobotCenterCheck");
                if (m2[i][j]== ObjectType.robot) {

                    Archivar.shout("RobotPixelCheck: " + cpimt++);
                    for (int k = 0; k < clusters.size(); k++) {
                        ArrayList<Pair<Integer, Integer>> curCluster = clusters.get(k);
                        if (curCluster.contains(new Pair(i, j))) {
                            continue;
                        }
                    }

                    Stack<Pair<Integer, Integer>> s = new Stack();
                    ArrayList<Pair<Integer, Integer>> cluster = new ArrayList<>();
                    s.push(new Pair(i, j));

                    //Flood fill, also making sure we don't get into loops
                    while (!s.isEmpty()) {
                        Pair<Integer, Integer> pair = s.pop();
                        if (m2[pair.getKey()][pair.getValue()]== ObjectType.robot && !cluster.contains(pair)) {
                            //System.out.println(i + ":" + j + " " + s.size());
                            cluster.add(pair);

                            if (pair.getKey() - 1 > 0 && pair.getKey() + 1 < m2.length && pair.getValue() - 1 > 0 && pair.getValue() + 1 < m2[0].length) {
                                s.push(new Pair(pair.getKey(), pair.getValue() + 1));
                                s.push(new Pair(pair.getKey(), pair.getValue() - 1));
                                s.push(new Pair(pair.getKey() + 1, pair.getValue()));
                                s.push(new Pair(pair.getKey() - 1, pair.getValue()));
                            }

                            //We can probably ignore all the other cases as the robot should be around the wall then, also we'd only miss a few pixels
                        }

                    }

                    if (cluster.size() > 1) {
                        clusters.add(cluster);
                    }
                }
            }
        }
    }


    public class Pixel{

    }

    public class Workflow{

    }



}