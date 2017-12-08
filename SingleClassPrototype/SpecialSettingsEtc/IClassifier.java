package SpecialSettingsEtc;

import Interfaces.ObjectType;

import java.io.IOException;
import java.util.HashMap;

/**
 * Created by Nibbla on 03.12.2017.
 */
public interface IClassifier extends Changeable<IClassifier> {
    @Override
    void editFields();

    @Override
    void save(String path) throws IOException;

    @Override
    void load(String path) throws IOException;

    HashMap<ObjectType, Double> classify(double red, double green, double blue);
}
