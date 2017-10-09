package Interfaces;


import view.PixelObjectType;
import model.Model;
import model.SpecialGraph;

import java.awt.image.BufferedImage;

/**
 * Created by Nibbla on 26.09.2017.
 */
public interface IView extends Cloneable {
    BufferedImage getCurrentShot();



    Model getClassifiedModel();

    SpecialGraph getGraph();

    PixelObjectType[][] classify(BufferedImage bi);

    PixelObjectType[][] classify();
}
