package Interfaces;

import Model.Node;
import Model.Path;
import Model.RoboPos;
import SpecialSettingsEtc.Tangential;

import java.util.ArrayList;

/**
 * Created by Nibbla on 26.09.2017.
 */
public interface IControl extends Cloneable {
    void move(Tangential.Direction Direction);
    void sendCommand(double width, double height, RoboPos currentPosition, double currentRotation, ArrayList<Node> pathway);

    void startConnection();
    void resetConnection();
    void closeConnection();

    void testCommands();
}
