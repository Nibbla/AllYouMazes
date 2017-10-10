package Control;

import Interfaces.IControl;
import Model.Path;
import SpecialSettingsEtc.Tangential;

/**
 * Created by Nibbla on 27.09.2017.
 */
public class Control implements IControl {
    private static Control factoryControl = new Control();
    public Control getInstance() throws CloneNotSupportedException {
        return (Control) factoryControl.clone();
    }




    @Override
    public boolean setPath(Path pathway) {
        return false;
    }

    @Override
    public boolean move(Path pathway) {
        return false;
    }

    @Override
    public boolean move(double[] moveDouble) {
        return false;
    }

    @Override
    public boolean move(Tangential.Direction Direction) {
        return false;
    }
}
