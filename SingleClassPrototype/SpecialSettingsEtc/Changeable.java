package SpecialSettingsEtc;


import java.io.IOException;

/**
 * Created by Nibbla on 21.10.2017.
 */
public interface Changeable<E> {
    public void editFields();
    public void save(String path) throws IOException;
    public void load(String path)throws IOException;
}
