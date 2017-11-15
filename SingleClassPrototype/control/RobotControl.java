package Control;

import Interfaces.IControl;
import Model.Path;
import SpecialSettingsEtc.Tangential;

import java.util.Map;

/**
 * This class is used to Control the epuck robot
 */
public class RobotControl implements IControl {

    // Use this to dynamically change to another ROS-Version, i.e. Kinetic. This has not been tested yet.
    private final String ROSversion = "indigo";

    // Replace this with whatever username is valid for the current System.
    private final String username = "eric";

    // The amount of seconds that will be waited after starting to connect to the epuck via bluetooth.
    private final int startUpSeconds = 20;

    // The basic structure of the startup command. The respective port and launchfile are specified here.
    private final String[] startCommand = {"bash", "-c", "/opt/ros/" + ROSversion + "/bin/roslaunch -p 11311 -v --screen epuck_driver multi_epuck.launch"};

    // The basic structure of a movement command. For more information check the ROS documentation.
    private final String[] movementCommand = {"bash", "-c", "/opt/ros/" + ROSversion + "/bin/rostopic pub --once /epuck_robot_0/mobile_base/cmd_vel geometry_msgs/Twist \'{linear:  {x: 0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: 0.0}}\'"};

    // These are used to spawn the processes that Control the epuck.
    private ProcessBuilder processGenerator;
    private Process initRosProcess;
    private Process motorSpeedProcess;

    /**
     * Initialize an Object of type RobotControl.
     * <p>
     * This will automatically set up the ProcessBuilder connected to it.
     */
    public RobotControl() {
        this.initProcessBuilder();
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

        this.initRosProcess.destroy();
    }

    /**
     * TODO: make use of paths.
     *
     * @param pathway
     * @return
     */
    @Override
    public boolean setPath(Path pathway) {

        return false;
    }

    /**
     * TODO: figure out a way to translate a path into moves while still using a closed-loop approach.
     *
     * @param pathway
     * @return
     */
    @Override
    public boolean move(Path pathway) {

        return false;
    }

    /**
     * TODO: what was this designed for? moving to a coordinate or directly setting the speed? In case of the later respective methods are already implemented.
     *
     * @param moveDouble
     * @return
     */
    @Override
    public boolean move(double[] moveDouble) {

        return false;
    }

    /**
     * TODO: translate a direction to a linear and angular speed so it can use the implemented methods.
     *
     * @param Direction
     * @return
     */
    @Override
    public boolean move(Tangential.Direction Direction) {

        return false;
    }

    /**
     * Used to test the connection and the commands.
     * <p>
     * This will make the robot:
     * 1) move straight with speed 1 (see below) for 3 seconds.
     * 2) stop for three seconds.
     * 3) rotate counterclockwise with speed 1 (see below) for 3 seconds
     * 4) stop.
     *
     * @throws InterruptedException because there is a Thread.sleep() between the commands.
     */
    public void testCommands() throws InterruptedException {
        this.moveStraight(1);
        Thread.sleep(3000);
        this.stop();
        Thread.sleep(3000);
        this.rotate(1);
        Thread.sleep(3000);
        this.stop();
    }

    /**
     * This creates the environment variables for the ProcessBuilder which are needed.
     * <p>
     * Probably not all of them are needed. If there is enough time it can be tested which of the below can be removed without breaking functionality.
     */
    private void initProcessBuilder() {
        processGenerator = new ProcessBuilder();

        processGenerator.redirectOutput(ProcessBuilder.Redirect.INHERIT);

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

            initRosProcess = processGenerator.start();

            Thread.sleep(startUpSeconds * 1000);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Starts the rostopic process with the correct parameters for the previously set speed. This will 'broadcast' the desired speed once and upon receiving it will be executed by the epuck.
     */
    private void issueMotorSpeed() {
        try {
            processGenerator.command(movementCommand);
            motorSpeedProcess = processGenerator.start();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * This is used to prepare the rostopic process with a linear and angular speed.
     * <p>
     * TODO: Figure out what 1 means in terms of m/s or cm/s. According to the manual a linear speed of 1 refers to 1m/s when using a turtlebot. Since we are not using a turtlebot these values need to be estimated by performing small tests like issuing a certain speed X for Y seconds and measuring the distance that to robot covered. Same for the angular speed in terms of degree/s.
     *
     * @param linear  a linear speed. a positive value is for forward, a negative value is for backward.
     * @param angular an anguar speed. a positive value is for counterclockwise, a negative value for clockwise.
     */
    private void setMotorSpeed(double linear, double angular) {
        movementCommand[2] = "/opt/ros/" + ROSversion + "/bin/rostopic pub --once /epuck_robot_0/mobile_base/cmd_vel geometry_msgs/Twist \'{linear:  {x: " + linear + ", y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: " + angular + "}}\'";
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
