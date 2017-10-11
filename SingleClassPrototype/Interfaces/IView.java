package Interfaces;


import Model.Model;
import Model.SpecialGraph;
import javafx.util.Pair;
import view.View;
import view.PixelObjectType;

import java.awt.image.BufferedImage;

/**
 * Created by Nibbla on 26.09.2017.
 */
public interface IView extends Cloneable {
    BufferedImage getCurrentShot();

    Pair<Double, Double> getRobotCenter(PixelObjectType[][] m2, int numberOfPixelsToSkip);

    Model getClassifiedModel();

    SpecialGraph getGraph(PixelObjectType[][] g);

    PixelObjectType[][] classify(BufferedImage bi);

    PixelObjectType[][] classify();
}
