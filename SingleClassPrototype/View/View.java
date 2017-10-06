package View;

import Interfaces.IView;
import Interfaces.ObjectType;
import SpecialSettingsEtc.Archivar;
import SpecialSettingsEtc.Settings;
import model.Model;
import model.SpecialGraph;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.List;

/**
 * Created by Nibbla on 27.09.2017.
 */
public class View implements IView {
    private static View factoryView = new View();

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

        int smallStepX = (int) Math.floor(stepX/2.);
        int smallStepY = (int) Math.floor(stepY/2.);




        for (int x = smallStepX+1; x < width; x+=stepX) {
            for (int y = smallStepY+1; y < heigth; y+=stepY) {

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

        }

        long tick2 = System.currentTimeMillis();
        Archivar.shout("Classifying image took " + (tick2 - tick1)/1000. + " seconds to complete");
        return null;
    }

    @Override
    public Model classify() {
        return classify(this.getCurrentShot());
    }

    public class Pixel{

    }

    public class Workflow{

    }


}