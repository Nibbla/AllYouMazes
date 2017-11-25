package Control;

import Interfaces.IControl;
import Model.Node;
import Model.RoboPos;
import SpecialSettingsEtc.Tangential;

import java.util.LinkedList;
import java.util.Map;

/**
 * This class is used to Control the epuck robot
 */
public class RobotControl implements IControl {

    // Use this to dynamically change to another ROS-Version, i.e. Kinetic.
    private final String ROSversion = "kinetic";

    // Replace this with whatever username is valid for the current System.
    private final String username = "pi";

    // The amount of seconds that will be waited after starting to connect to the epuck via bluetooth.
    private final int startUpSeconds = 120;

    // The basic structure of the startup command. The respective port and launchfile are specified here.
    private final String[] startCommand = {"bash", "-c", "/opt/ros/" + ROSversion + "/bin/roslaunch -p 11311 -v --screen epuck_driver multi_epuck.launch"};

    // The basic structure of a movement command. For more information check the ROS documentation.
    private final String[] movementCommand = {"bash", "-c", "/opt/ros/" + ROSversion + "/bin/rostopic pub --once /epuck_robot_0/mobile_base/cmd_vel geometry_msgs/Twist \'{linear:  {x: 0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0,z: 0.0}}\'"};

    // These are used to spawn the processes that Control the epuck.
    private ProcessBuilder processGenerator;
    private Process initRosProcess;
    private Process motorSpeedProcess;
    private boolean isRunning;

    // Implementation of factory pattern
    private static RobotControl factoryControl = new RobotControl();

    // Values used for moving slowly either forward or angular
    private final double SLOWFORWARDSPEED = 0.5;
    private final double SLOWANGULARSPEED = 0.3;
    private final double POSITIONERROR = 3;
    private final double ROTATIONERROR = 3;


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
     * @return instance of RobotControl to be reused
     * @throws CloneNotSupportedException
     */
    public RobotControl getInstance() throws CloneNotSupportedException {
        return (RobotControl) factoryControl.clone();
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
        if (isRunning){
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
        setMotorSpeed(Direction.linearSpeed * SLOWFORWARDSPEED, Direction.angularSpeed * SLOWANGULARSPEED);
        issueMotorSpeed();
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
     * @throws *InterruptedException because there is a Thread.sleep() between the commands.
     */
    @Override
    public void testCommands(){
        /*try {
            this.moveStraight(1);
            Thread.sleep(3000);
            this.stop();
            Thread.sleep(3000);
            this.rotate(1);
            Thread.sleep(3000);
            this.stop();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/

        System.out.println("test");
    }

    /**
     * Takes as input raw data from the Modul and processes it into a command for the epuck
     * @param width represents the width of the observable area in px, currently unnused
     * @param height represents the height of the observable area in px, currently unused
     * @param currentPosition current position of the robot in the above specified grid
     * @param currentRotation current rotation in radians, with 0 facing 'north'
     * @param pathway next coordinates along the way to the goal
     */
    @Override
    public void sendCommand(double width, double height, RoboPos currentPosition, double currentRotation, LinkedList<Node> pathway){
        /**
         * Initialization of some of the variables used in this method. They can probably be transferred as private fields for efficiency
         */
        double positionDistance = 0;
        Node nextGoal = new Node(0,0);
        boolean goalReached = false;

        double rotationCoefficient = 0;
        double linearCoefficient = 0;

        /**
         * get next node along the path. in case it's close enough (POSITIONERROR) take the next possible node.
         */
        do{
            if(pathway.isEmpty()){
                goalReached = true;
            } else {
                nextGoal = pathway.removeFirst();
                positionDistance = Math.sqrt((Math.pow((currentPosition.getX() - nextGoal.getX()),2) + Math.pow((currentPosition.getY() - nextGoal.getY()),2)));
            }
        }while(positionDistance < POSITIONERROR && !goalReached);

        /**
         * In case there are still nodes in the path determine if a rotation is necessary to face towards it, otherwise move straight.
         */
        if(!goalReached){
            double desiredRotation = currentPosition.getAngleTo(nextGoal);
            double distance = (Math.toDegrees(desiredRotation) - Math.toDegrees(currentRotation)) % 360;

            if (distance < -180) {
                distance += 360;
            } else if (distance > 179) {
                distance -= 360;
            }

            if(Math.abs(distance) > ROTATIONERROR){
                if (distance > 0){
                    rotationCoefficient = -1;
                    linearCoefficient = 1;
                } else {
                    rotationCoefficient = 1;
                    linearCoefficient = 1;
                }
            } else {
                linearCoefficient = 1;
            }
        }


        /**
         * set and issue the new speed depending on the above findings, i.e. issue rotation or issue forward-movement
         */
        setMotorSpeed(linearCoefficient * SLOWFORWARDSPEED, rotationCoefficient * SLOWANGULARSPEED);
        issueMotorSpeed();
    }

    @Override
    public void sendCommand(double linearSpeed, double angularSpeed){
        setMotorSpeed(linearSpeed*SLOWFORWARDSPEED, angularSpeed * SLOWFORWARDSPEED);
        issueMotorSpeed();
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
            System.out.println("Starting ROS");
            Thread.sleep(startUpSeconds * 1000);
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
            processGenerator.command(movementCommand);
            System.out.println("Trying to send");
            motorSpeedProcess = processGenerator.start();
            System.out.println("Sending command: " + movementCommand);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * This is used to prepare the rostopic process with a linear and angular speed.
     * <p>
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
