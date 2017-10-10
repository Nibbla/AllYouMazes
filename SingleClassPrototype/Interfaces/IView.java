package Interfaces;


import Model.Model;
import Model.SpecialGraph;
import View.View;

import java.awt.image.BufferedImage;

/**
 * Created by Nibbla on 26.09.2017.
 */
public interface IView extends Cloneable {
    BufferedImage getCurrentShot();



    Model getClassifiedModel();

    SpecialGraph getGraph();

    View.PixelObjectType[][] classify(BufferedImage bi);

    View.PixelObjectType[][] classify();
}
