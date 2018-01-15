package Simulation;

import org.python.core.PyInstance;
import org.python.util.PythonInterpreter;

/*
Download jython, add JAR to project
https://github.com/gctronic/epuck_driver/blob/master/src/epuck/ePuck.py (python file)
https://github.com/gctronic/epuck_driver/blob/master/scripts/epuck_driver.py
http://search.maven.org/remotecontent?filepath=org/python/jython-installer/2.7.0/jython-installer-2.7.0.jar
 */

public class EPuckWrapper {

    public static void main(String[] args) {
        PythonInterpreter interpreter = null;
        PythonInterpreter.initialize(System.getProperties(), System.getProperties(), new String[0]);
        interpreter = new PythonInterpreter();

        //python file
        interpreter.execfile("path/to/ePuck.py");
        /*classname + opts
        ePuck(MAC ADDRESS)*/
        PyInstance instance = (PyInstance) interpreter.eval("ePuck(MAC)");
        /*method name
        set_motors_speed
        maybe create our own method
        check epuck_driver.py for math (handler_velocity)
        to test just use max speed (1000)*/
        instance.invoke("");
    }

}
