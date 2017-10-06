package model;

/**
 * Created by Nibbla on 26.09.2017.
 */
public class KartesianCoordinates {
    float x;
    float y;
    float z;

    public KartesianCoordinates clone() {
        KartesianCoordinates kc = new KartesianCoordinates();
        kc.x = x;
        kc.y = y;
        kc.z = z;
        return kc;
    }
}