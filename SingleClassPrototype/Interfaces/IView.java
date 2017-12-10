package Interfaces;


import Model.*;
import view.classifier.Classifier;


import java.awt.image.BufferedImage;
import java.util.ArrayList;

/**
 * Created by Nibbla on 26.09.2017.
 */
public interface IView extends Cloneable {
    BufferedImage getCurrentShot();

    //RoboPos getRobotCenter(ObjectType[][] m2, int numberOfPixelsToSkip);

    ModelDeprecated getClassifiedModel();

    public DijkstraPathFinder getGraph(ObjectType[][] g, int imageType, RoboPos roboPos, int graphSkip, boolean workmode);



    ObjectType[][] classify(BufferedImage bi, boolean showClassification, Classifier classifier);

    ArrayList<RoboPos> getRobotCenter(ObjectType[][] m2, int i);
}
