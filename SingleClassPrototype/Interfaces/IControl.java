package Interfaces;

import model.Path;
import SpecialSettingsEtc.Tangential;

/**
 * Created by Nibbla on 26.09.2017.
 */
public interface IControl extends Cloneable {
    boolean setPath(Path pathway);
    boolean move(Path pathway);
    boolean move(double[] moveDouble);
    boolean move(Tangential.Direction Direction);
}
