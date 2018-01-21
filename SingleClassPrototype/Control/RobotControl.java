package Control;

import Interfaces.IControl;
import Simulation.Simulation;
import SpecialSettingsEtc.Tangential;

import java.util.Map;
import java.io.PrintWriter;

/**
 * This class is used to Control the epuck robot
 */
public class RobotControl implements IControl {

    // Implementation of factory pattern
    private static RobotControl factoryControl = new RobotControl();
    // Use this to dynamically change to another ROS-Version, i.e. Kinetic.
    private final String ROSversion = "kinetic";
    // Replace this with whatever username is valid for the current System.
    private final String username = "pi";
    // The amount of seconds that will be waited after starting to connect to the epuck via bluetooth.
    private final int startUpSeconds = 10;
    // The basic structure of the startup command. The respective port and launchfile are specified here.
    private final String[] startCommand = {"python","/home/" + username + "/catkin_ws/src/epuck_driver/src/epuck/ePuck.py"};
    // The basic structure of a movement command. For more information check the ROS documentation.
    private final double[] movementCommand = {0,0};
    // Values used for moving either forward or angular
    private final double FORWARDSPEED = 520; // with a max of 3.5
    private final double ANGULARSPEED = 200; // with a max of 1.5
    // These are used to spawn the processes that Control the epuck.
    private ProcessBuilder processGenerator;
    private Process initRosProcess;
    private Process motorSpeedProcess;
    private boolean isRunning;
    private boolean isSending; 
    private PrintWriter out;
    private int linearSpeedCount = 0;
    private int linearSpeedCountTreshhold = 10;
    private int overWriteSteps;
    private int overWriteStepsThreshold = 10;


    /**
     * Initialize an Object of type RobotControl.
     * <p>
     * This will automatically set up the ProcessBuilder connected to it.
     */
    public RobotControl() {
        this.initProcessBuilder();
    }

    /**
     * Implementation of factory pattern
     *
     * @return instance of RobotControl to be reused
     * @throws CloneNotSupportedException
     */
    public RobotControl getInstance()  {
        try {
            return (RobotControl) factoryControl.clone();
        } catch (CloneNotSupportedException e) {
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Start the connection with the epuck.
     * <p>
     * NOTE: After the process for connecting with the epuck is started it will currently force the Thread to sleep for 'startUpSeconds' seconds in order to assure that a successful connection with the epuck can be established.
     */
    @Override
    public void startConnection() {
        this.initConnection();
    }

    /**
     * Restart the connection with the epuck. This can be used if the connection gets lost or is broken.
     */
    @Override
    public void resetConnection() {
        this.initRosProcess.destroy();
        this.initConnection();
    }

    /**
     * Closes the current connection with the epuck. Can be used when ending the program.
     */
    @Override
    public void closeConnection() {
        if (isRunning) {
            this.initRosProcess.destroy();
            isRunning = false;
        }
    }

    /**
     * Takes as input a Tangential direction enum and makes the epuck move slowly into that direction
     *
     * @param Direction
     * @return
     */
    @Override
    public void move(Tangential.Direction Direction) {
        setMotorSpeed(Direction.linearSpeed * FORWARDSPEED, Direction.angularSpeed * ANGULARSPEED);
        issueMotorSpeed();
    }

    /**
     * Method used to send a command to the robot. Currently only makes use of defined *SPEED in order to move with a stable speed
     *
     * @param linearSpeed  linear speed coefficient (between 0 and 1)
     * @param angularSpeed angular speed coefficient (between 0 and 1)
     */
    @Override
    public void sendCommand(double linearSpeed, double angularSpeed) {
        System.out.println("Command sended. Linear: " + linearSpeed + " Angular: " + angularSpeed);

        if (byPassCommand(linearSpeed,angularSpeed)){return;}

	if (linearSpeed == 1){
        	setMotorSpeed(linearSpeed * FORWARDSPEED, linearSpeed * FORWARDSPEED);
	} else if (angularSpeed == -1){
		setMotorSpeed(-1 * angularSpeed * ANGULARSPEED, angularSpeed * ANGULARSPEED);
	} else if (angularSpeed == 1){
		setMotorSpeed(-1 * angularSpeed * ANGULARSPEED, angularSpeed * ANGULARSPEED);
	} else if (linearSpeed == 0 && angularSpeed == 0){
		setMotorSpeed(0,0);
	}
        issueMotorSpeed();
    }

    private boolean byPassCommand(double linearSpeed, double angularSpeed) {
        if (linearSpeed == 0){
            linearSpeedCount++;
        }else {linearSpeedCount = 0;}
        if (overWriteSteps > overWriteStepsThreshold){
            overWriteSteps = 0;
            linearSpeedCount = 0;
            return false;
        }
        if (linearSpeed > linearSpeedCountTreshhold|| overWriteSteps++ < overWriteStepsThreshold){
            setMotorSpeed(240,240);
            issueMotorSpeed();
            return true;
        }
        return false;
    }

    /**
     * This creates the environment variables for the ProcessBuilder which are needed.
     * <p>
     * Probably not all of them are needed. If there is enough time it can be tested which of the below can be removed without breaking functionality.
     */
    private void initProcessBuilder() {
        processGenerator = new ProcessBuilder();

        //processGenerator.redirectOutput(ProcessBuilder.Redirect.INHERIT);

        Map<String, String> env = processGenerator.environment();

        env.put("LD_LIBRARY_PATH", "/home/" + username + "/catkin_ws/devel/lib:/opt/ros/" + ROSversion + "/lib");
        env.put("CPATH", "/opt/ros/" + ROSversion + "/include");
        env.put("PATH", env.get("PATH") + ":/opt/ros/" + ROSversion + "/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/home/" + username + "/bin:/usr/lib/jvm/jdk1.8.0/bin:/opt/microchip/xc16/v1.32/bin:/opt/microchip/xc16/v1.32/bin");
        env.put("ROSLISP_PACKAGE_DIRECTORIES", "/home/" + username + "/catkin_ws/devel/share/common-lisp");
        env.put("PYTHONPATH", "/home/" + username + "/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/" + ROSversion + "/lib/python2.7/dist-packages");
        env.put("ROS_DISTRO", ROSversion);
        env.put("ROS_ROOT", "/opt/ros/" + ROSversion + "/share/ros");
        env.put("ROS_PACKAGE_PATH", "/home/" + username + "/catkin_ws/src:/opt/ros/" + ROSversion + "/share:/opt/ros/" + ROSversion + "/stacks");
        env.put("ROS_MASTER_URI", "http://localhost:11311");
        env.put("PKG_CONFIG_PATH", "/home/" + username + "/catkin_ws/devel/lib/pkgconfig:/opt/ros/" + ROSversion + "/lib/pkgconfig");
        env.put("CMAKE_PREFIX_PATH", "/home/" + username + "/catkin_ws/devel:/opt/ros/" + ROSversion);
        env.put("ROS_ETC_DIR", "/opt/ros/" + ROSversion + "/etc/ros");
        env.put("PWD", "/home/" + username);
    }

    /**
     * Starts the roslaunch process with the correct parameters which will set-up a connection with the epuck.
     */
    private void initConnection() {
        try {
            processGenerator.command(startCommand);

            //initRosProcess = processGenerator.start();
            System.out.println("Starting ROS");
            //Thread.sleep(startUpSeconds * 1000);
            System.out.println("Connection established");
            isRunning = true;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Starts the rostopic process with the correct parameters for the previously set speed. This will 'broadcast' the desired speed once and upon receiving it will be executed by the epuck.
     */
    private void issueMotorSpeed() {
        try {

            if (Simulation.DEBUG_CONTROLLER) {
                System.out.println("Trying to send");
            }

            if (isSending) {
                
            }
		try{
    out = new PrintWriter("/home/pi/Desktop/test.txt");
    out.println( ((int)movementCommand[0]) + " " + ((int)movementCommand[1]));

		}catch(Exception e){
		
}finally{
    out.close();
}
            isSending = true;

            if (Simulation.DEBUG_CONTROLLER) {
                System.out.println("Did send");
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * This is used to prepare the rostopic process with a linear and angular speed.
     * <p>
     *
     * @param left  linear speed. a positive value is for forward, a negative value is for backward.
     * @param right linear speed. a positive value is for forward, a negative value is for backward.
     */
    private void setMotorSpeed(double left, double right) {
        movementCommand[0] = (int) left;
	movementCommand[1] = (int) right;
    }

    /**
     * Using this command the epuck will drive in a curve.
     *
     * @param linear  a linear speed. a positive value is for forward, a negative value is for backward.
     * @param angular an anguar speed. a positive value is for counterclockwise, a negative value for clockwise.
     */
    private void moveCurve(double linear, double angular) {
        setMotorSpeed(linear, angular);
        issueMotorSpeed();
    }

    /**
     * Using this command the epuck will drive straight.
     *
     * @param speed a linear speed. a positive value is for forward, a negative value is for backward.
     */
    private void moveStraight(double speed) {
        setMotorSpeed(speed, 0);
        issueMotorSpeed();
    }

    /**
     * Using this command the epuck will rotate on the spot.
     *
     * @param speed an anguar speed. a positive value is for counterclockwise, a negative value for clockwise.
     */
    private void rotate(double speed) {
        setMotorSpeed(0, speed);
        issueMotorSpeed();
    }

    /**
     * Using this command the epuck will stop.
     */
    private void stop() {
        setMotorSpeed(0, 0);
        issueMotorSpeed();
    }
}
