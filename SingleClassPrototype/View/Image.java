package view;

import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.IndexColorModel;
import java.awt.image.WritableRaster;
import java.util.Hashtable;

/**
 * Created by Nibbla on 06.10.2017.
 */
public class Image extends BufferedImage {
    public Image(int width, int height, int imageType) {
        super(width, height, imageType);
    }

    public Image(int width, int height, int imageType, IndexColorModel cm) {
        super(width, height, imageType, cm);
    }

    public Image(ColorModel cm, WritableRaster raster, boolean isRasterPremultiplied, Hashtable<?, ?> properties) {
        super(cm, raster, isRasterPremultiplied, properties);
    }
}
