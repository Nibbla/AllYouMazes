package Interfaces;


import Model.Model;
import Model.SpecialGraph;
import Model.RoboPos;
import javafx.util.Pair;
import view.View;
import view.PixelObjectType;

import java.awt.image.BufferedImage;

/**
 * Created by Nibbla on 26.09.2017.
 */
public interface IView extends Cloneable {
    BufferedImage getCurrentShot();

    //RoboPos getRobotCenter(ObjectType[][] m2, int numberOfPixelsToSkip);

    Model getClassifiedModel();

    public SpecialGraph getGraph(ObjectType[][] g, int imageType, RoboPos roboPos);

    ObjectType[][] classify(BufferedImage bi);

    ObjectType[][] classify();

    RoboPos getRobotCenter(ObjectType[][] m2, int i);
}
