package Interfaces;

import Model.Node;
import Model.RoboPos;
import SpecialSettingsEtc.Tangential;

import java.util.LinkedList;

/**
 * Created by Nibbla on 26.09.2017.
 */
public interface IControl extends Cloneable {
    void move(Tangential.Direction Direction);
    void sendCommand(double width, double height, RoboPos currentPosition, double currentRotation, LinkedList<Node> pathway);

    void startConnection();
    void resetConnection();
    void closeConnection();

    void testCommands();
}
