package SpecialSettingsEtc;


import java.io.IOException;

/**
 * Created by Nibbla on 21.10.2017.
 */
public interface Changeable<E> {
    public void edit();
    public void save() throws IOException;
    public void load()throws IOException;
}
