package Model;

/**
 * Created by Jordy on 19-11-2017.
 */

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

//Everything in the Util package is copied from
//https://github.com/badlogic/opencv-fun/tree/master/src/pool/utils

/*
Installation instruction for (Windows) OpenCV (we are using 3.1.0)!

Download OpenCV 3.1.0 for Windows here and extract from the .exe to a folder
http://sourceforge.net/projects/opencvlibrary/pictures/opencv-win/3.1.0/opencv-3.1.0.exe/download

For eclipse, simply follow these steps:
http://opencv-java-tutorials.readthedocs.io/en/latest/01-installing-opencv-for-java.html#set-up-opencv-for-java-in-eclipse

For IntelliJ, File -> Project Structure -> Modules
Click the green plus icon on the right and add the opencv-310.jar found in \opencv\build\java
Double click on the jar you just added (in IntelliJ), click on the green plus icon in the screen that pops up and add
\opencv\build\java\x64 (for 64-bit) or \opencv\build\java\x86 for 32-bit systems.
 */

//Java OpenCV docs are here!
//https://docs.opencv.org/java/3.1.0/

public class ComputerVision {

    //To do
    //Method argument for robot location: last location
    //So we only check in a certain distance around it -> reduces computing time

    //Perhaps do scaling and grayscaling in separate method

    public final static double SCALE_FACTOR = 0.5;
    public final static double ROBOT_RADIUS = 50*SCALE_FACTOR;

    static{System.loadLibrary(Core.NATIVE_LIBRARY_NAME);}

    private static Mat preprocess(String picture) {
        Mat img = Imgcodecs.imread(picture);

        int w = img.width();
        int h = img.height();

        Mat simg = new Mat();
        Imgproc.resize(img, simg, new Size(SCALE_FACTOR*w, SCALE_FACTOR*h));

        Mat gray = new Mat();
        simg.copyTo(gray);
        Imgproc.cvtColor(gray, gray, Imgproc.COLOR_BGR2GRAY);
        return gray;
    }

    /*
    Returns x, y and r(adius) of the robot
     */
    public static ArrayList<Integer> retrieveRobot(Mat gray) {
        Mat circles = null;

        //dp can give some problems, might need to cycle through (did this for now)
        //0.33 - 1.7
        //0.5 - 1.2
        //1 - 1.4

        int dp = 1;
        int c = 0;
        double step = 0.1;
        int s = -1;

        while (s != 1 && c < 10) {
            circles = new Mat();
            Imgproc.HoughCircles(gray, circles, Imgproc.CV_HOUGH_GRADIENT, dp + (c*step), 100);
            c++;
            s = circles.cols();

            if (s == 1) {
                int r = (int)circles.get(0, 0)[2] + 1;
                if (!((r-4) <= ROBOT_RADIUS && (r+4) >= ROBOT_RADIUS)) {
                    s = -1;
                }
            }
        }

        if (circles.cols() != 1 || s == -1) {
            System.out.println("No / Multiple circles detected!");
            return null;
        }

        ArrayList<Integer> result = new ArrayList<Integer>();
        result.add((int)circles.get(0, 0)[0] + 1);
        result.add((int)circles.get(0, 0)[1] + 1);
        result.add((int)circles.get(0, 0)[2] + 1);

//        ImgWindow wnd = ImgWindow.newWindow(gray);
//
//        while(!wnd.closed) {
//            wnd.setImage(gray);
//            Graphics2D g = wnd.begin();
//            g.setColor(Color.MAGENTA);
//            g.setStroke(new BasicStroke(3));
//            for(int i = 0; i < circles.cols(); i++) {
//                double[] circle = circles.get(0, i);
//                g.drawOval((int)circle[0] - (int)circle[2], (int)circle[1] - (int)circle[2], (int)circle[2] * 2, (int)circle[2] * 2);
//            }
//            wnd.end();
//        }

        return result;
    }

    /*
    Returns points that together make up the contour polygon (aka maze polygon)
    Returned in [x1, y1, x2, y2, x3, y3, ... , xn, yn] format
     */
    public static ArrayList<Integer> retrieveContour(Mat gray) {
        Mat equ = new Mat();
        gray.copyTo(equ);
        CLAHE clahe = Imgproc.createCLAHE(4.0*SCALE_FACTOR, new Size(16.0*SCALE_FACTOR, 16.0*SCALE_FACTOR));
        clahe.apply(gray, equ);

        int blurSize = (int)(15*SCALE_FACTOR);
        if (blurSize % 2 != 1) {
            blurSize += 1;
        }

        Mat blur = new Mat();
        equ.copyTo(blur);
        Imgproc.medianBlur(equ, blur, blurSize);

        Mat thresh = new Mat();
        blur.copyTo(thresh);
        Imgproc.threshold(blur, thresh, 0, 255, Imgproc.THRESH_OTSU);
        Mat dilateElement = new Mat();
        org.opencv.core.Point p = new org.opencv.core.Point(-1, -1);
        Imgproc.dilate(thresh, thresh, dilateElement, p, (int)Math.ceil(ROBOT_RADIUS));

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        //Imgproc.drawContours(gray, contours, -1, new Scalar(0, 255, 0), 2);

        double size = -1;
        MatOfPoint contour = null;

        for (MatOfPoint mop: contours) {
            if (mop.size().height > size) {
                size = mop.size().height;
                contour = mop;
            }
        }

        if (size == -1 || size < 100) {
            return null;
        }

        ArrayList<Integer> result = new ArrayList<>();

        for (int i = 0; i < size; i++) {
            String[] tmp = contour.row(i).dump().substring(1, contour.row(i).dump().length()-1).split(", ");
            result.add(Integer.parseInt(tmp[0]));
            result.add(Integer.parseInt(tmp[1]));
        }

        //ImgWindow.newWindow(gray);

        return result;
    }

    public static void main(String[] args) {
        //Current only works (and is optimized) for latestScreen.jpg

        long start = System.currentTimeMillis();
        Mat gray = preprocess("path\\to\\picture");
        ArrayList<Integer> robotLoc = retrieveRobot(gray);
        ArrayList<Integer> contour = retrieveContour(gray);
        long end = System.currentTimeMillis();
        System.out.println("CV took " + (end-start) + "ms");

        System.out.println("Robot location details: " + robotLoc);
        System.out.println("Contour details: " + contour);
    }

}

