package Control;

import Interfaces.IControl;
import Model.Path;
import SpecialSettingsEtc.Tangential;

import java.util.Map;

/**
 * Created by Nibbla on 27.09.2017.
 */
public class Control implements IControl {

    private final String ROSversion = "indigo";
    private final String username = "eric";
    private final int startUpSeconds = 20;

    private final String[] startCommand = {"bash", "-c", "/opt/ros/" + ROSversion + "/bin/roslaunch -p 11311 -v --screen epuck_driver multi_epuck.launch"};
    private final String[] movementCommand = {"bash", "-c", "/opt/ros/" + ROSversion + "/bin/rostopic pub --once /epuck_robot_0/mobile_base/cmd_vel geometry_msgs/Twist \'{linear:  {x: 0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: 0.0}}\'"};

    private ProcessBuilder processGenerator;
    private Process initRosProcess;
    private Process motorSpeedProcess;


    public Control() {
        this.initProcessBuilder();
    }


    @Override
    public void startConnection() {
        this.initConnection();
    }

    @Override
    public void resetConnection() {
        this.initRosProcess.destroy();
        this.initConnection();
    }

    @Override
    public void closeConnection() {

        this.initRosProcess.destroy();
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

    public void testCommands() throws InterruptedException {
        this.moveStraight(1);
        Thread.sleep(3000);
        this.stop();
        Thread.sleep(3000);
        this.rotate(1);
        Thread.sleep(3000);
        this.stop();
    }

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

    private void initConnection() {
        try {
            processGenerator.command(startCommand);

            initRosProcess = processGenerator.start();

            Thread.sleep(startUpSeconds * 1000);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void issueMotorSpeed() {
        try {
            processGenerator.command(movementCommand);
            motorSpeedProcess = processGenerator.start();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void setMotorSpeed(double linear, double angular) {
        movementCommand[2] = "/opt/ros/indigo/bin/rostopic pub --once /epuck_robot_0/mobile_base/cmd_vel geometry_msgs/Twist \'{linear:  {x: " + linear + ", y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: " + angular + "}}\'";
    }

    private void moveCurve(double linear, double angular) {
        setMotorSpeed(linear, angular);
        issueMotorSpeed();
    }

    private void moveStraight(double speed) {
        setMotorSpeed(speed, 0);
        issueMotorSpeed();
    }

    private void rotate(double speed) {
        setMotorSpeed(0, speed);
        issueMotorSpeed();
    }

    private void stop() {
        setMotorSpeed(0, 0);
        issueMotorSpeed();
    }
}
