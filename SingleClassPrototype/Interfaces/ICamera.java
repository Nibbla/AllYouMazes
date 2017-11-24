package Interfaces;

/**
 * Interface for the Camera implementation. Since this is already very specific I'm not sure if its needed anyways.
 */
public interface ICamera extends Cloneable {
    void startCamera(int durationInMinutes, double picturesPerSecond, int width, int height, int quality, boolean verticalFlip, boolean horizontalFlip, String path, double roiX, double roiY, double roiW, double roiH);

    void stopCamera();
}
