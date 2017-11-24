package view;

import Interfaces.ICamera;

/**
 * Class for externalizing the Camera process. If a high enough duration is specified upon startup, this only should be called once.
 */
public class Camera implements ICamera {

    // Implementation of factory pattern
    private static Camera cameraFactory = new Camera();

    // Variables needed to spawn/control toe raspistill-process
    private ProcessBuilder processGenerator;
    private Process cameraProcess;
    private boolean isRunning = false;

    // Default placeholder command
    private final int defaultDuration = Integer.MAX_VALUE;
    private final int defaultPictursPerSecond = 1;
    private final String[] cameraCommand = {"raspistill","-w", "1300", "-h", "2000","-q", "50","-t", Integer.toString(defaultDuration),"-tl", Integer.toString(defaultPictursPerSecond * 1000) ,"-n", "-roi", "0.075,0.1,0.8,0.8" ,"-o", ""};

    /**
     * Constructor for creating an empty Camera-handler
     */
    public Camera(){
        processGenerator = new ProcessBuilder();
    }

    /**
     * Method for configuring and starting the camera process.
     *
     * TODO: maybe already include an imagefilter upon capturing using the "-ifx" option. possible options: blur, negative, ...
     * TODO: test if using raspistill's time-lapse feature resolves the issue of unnecessary long camera startup each picture
     *
     * @param durationInMinutes due to the nature of the raspistill time-lapse a maximal duration has to be specified
     * @param picturesPerSecond how many pictures per second
     * @param width target width of the picture
     * @param height target height of the picture
     * @param quality JPEG quality of the picture. 1 - 100
     * @param path path to the input picture
     */
    @Override
    public void startCamera(int durationInMinutes, double picturesPerSecond, int width, int height, int quality, String path, double roiX, double roiY, double roiW, double roiH) {
        this.stopCamera();

        cameraCommand[2] = Integer.toString(width);
        cameraCommand[4] = Integer.toString(height);

        cameraCommand[6] = Integer.toString(quality);
        cameraCommand[8] = Integer.toString(durationInMinutes * 60 * 1000);
        cameraCommand[10] = Integer.toString((int)(1000 / picturesPerSecond));
        cameraCommand[13] = Double.toString(roiX)+","+Double.toString(roiY)+","+Double.toString(roiW)+","+Double.toString(roiH);
        cameraCommand[15] = path;

        try {
            processGenerator.command(cameraCommand);
            cameraProcess = processGenerator.start();
            isRunning = true;
        } catch (Exception e) {
            System.out.println("Error starting camera process");
            e.printStackTrace();
        }
    }

    /**
     * Method for stopping the Camera process (i.e. on shutdown of the program).
     */
    @Override
    public void stopCamera() {
        if (isRunning){
            cameraProcess.destroy();
            isRunning = false;
        }

    }

    /**
     * Implementation of the factory pattern
     * @return an instance of the camera
     * @throws CloneNotSupportedException
     */
    public Camera getInstance() throws CloneNotSupportedException {
        return (Camera) cameraFactory.clone();
    }
}
