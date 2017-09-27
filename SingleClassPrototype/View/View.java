package View;

import Interfaces.IView;

import java.util.Random;

/**
 * Created by Nibbla on 27.09.2017.
 */
public class View implements IView {
    private View factoryView = new View();

    public View() {

    }

    public View getInstance() throws CloneNotSupportedException {
        return (View) factoryView.clone();
    }

    public class Pixel{

    }

    public class Workflow{

    }


}