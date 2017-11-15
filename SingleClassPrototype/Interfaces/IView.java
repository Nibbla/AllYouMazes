package Interfaces;


import Model.*;


import java.awt.image.BufferedImage;

/**
 * Created by Nibbla on 26.09.2017.
 */
public interface IView extends Cloneable {
    BufferedImage getCurrentShot();

    //RoboPos getRobotCenter(ObjectType[][] m2, int numberOfPixelsToSkip);

    Model getClassifiedModel();

    public SpecialGraph getGraph(ObjectType[][] g, int imageType, RoboPos roboPos, int graphSkip, boolean workmode);



    ObjectType[][] classify(BufferedImage bi, boolean showClassification);

    RoboPos getRobotCenter(ObjectType[][] m2, int i);
}
