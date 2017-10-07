package Interfaces;


import java.awt.*;

/**
 * Created by Nibbla on 06.10.2017.
 */
public enum ObjectType {
    floor(Color.WHITE),wall(Color.MAGENTA), robot(Color.GREEN);

    private final Color color;

    ObjectType(Color green) {
        this.color = green;
    }

    public int getColor() {
        return color.getRGB();
    }
}
