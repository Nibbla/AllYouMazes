package Interfaces;

import Model.Path;
import SpecialSettingsEtc.Tangential;

/**
 * Created by Nibbla on 26.09.2017.
 */
public interface IControl {
    boolean setPath(Path pathway);
    boolean move(Path pathway);
    boolean move(double[] moveDouble);
    boolean move(Tangential.Direction Direction);

    void startConnection();
    void resetConnection();
    void closeConnection();
}
