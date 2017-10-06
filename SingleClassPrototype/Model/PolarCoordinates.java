package model;

import SpecialSettingsEtc.Tangential;

/**
 * Created by Nibbla on 26.09.2017.
 */
public class PolarCoordinates {
    byte phi;
    byte theta;
    float r;

    public KartesianCoordinates toKartesianCoordinates() {
        KartesianCoordinates k = new KartesianCoordinates();
        k.x = Tangential.sin(theta) * Tangential.cos(phi);
        k.y = r *Tangential.sin(phi)*Tangential.sin(theta);
        k.z = r *  Tangential.cos(phi);
        return k;
    }
}