package Interfaces;

import Model.Path;
import Model.RoboPos;
import SpecialSettingsEtc.Tangential;

/**
 * Created by Nibbla on 26.09.2017.
 */
public interface IControl extends Cloneable {
    boolean setPath(Path pathway);
    boolean move(Path pathway);
    boolean move(double[] moveDouble);
    boolean move(Tangential.Direction Direction);
    boolean sendCommand(double width, double height, RoboPos currentPosition, double currentRotation, Path pathway);

    void startConnection();
    void resetConnection();
    void closeConnection();

    void testCommands();
}
