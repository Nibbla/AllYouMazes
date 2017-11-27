package Model;

/**
 * Created by Jordy on 19-11-2017.
 */

import Util.ImgWindow;
import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.CLAHE;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.BackgroundSubtractorKNN;
import org.opencv.video.BackgroundSubtractorMOG2;
import org.opencv.video.Video;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import java.awt.*;
import java.io.File;
import java.util.*;
import java.util.List;

//Everything in the Util package is copied from
//https://github.com/badlogic/opencv-fun/tree/master/src/pool/utils

/*
Installation instruction for (Windows) OpenCV (we are using 3.1.0)!

Download OpenCV 3.1.0 for Windows here and extract from the .exe to a folder
https://sourceforge.net/projects/opencvlibrary/files/opencv-win/3.1.0/opencv-3.1.0.exe/download

For eclipse, simply follow these steps:
http://opencv-java-tutorials.readthedocs.io/en/latest/01-installing-opencv-for-java.html#set-up-opencv-for-java-in-eclipse

For IntelliJ, File -> Project Structure -> Modules
Click the green plus icon on the right and add the opencv-310.jar found in \opencv\build\java
Double click on the jar you just added (in IntelliJ), click on the green plus icon in the screen that pops up and add
\opencv\build\java\x64 (for 64-bit) or \opencv\build\java\x86 for 32-bit systems.
 */

//Java OpenCV docs are here!
//https://docs.opencv.org/java/3.1.0/

//Don't use retrieveRobot anymore.
//In fact most of them are outdated

public class ComputerVision {

    //Perhaps do scaling and grayscaling in separate method
    //TODO detect robot by color (put something on it)
    //TODO delete all other contours before repeating findContours
    //TODO what if maze is not connected?
    //TODO maybe cut out part if image isnt entirely on the black paper
    //TODO blob detection/background substraction

    public final static boolean DEBUG = true;
    public final static double SCALE_FACTOR = 0.5;
    public final static int STEP_SIZE = 4;
    public final static int PROXIMITY = (int) (2 * SCALE_FACTOR);

    public final static int RADIUS_EST = (int) (100 * SCALE_FACTOR);
    public final static int RADIUS_ESTOFFSET = (int) (20 * SCALE_FACTOR);

    public static FeatureDetector angleDetector;
    public static FeatureDetector robotDetector;

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public static Mat readImg(String picture) {
        return Imgcodecs.imread(picture);
    }

    public static Mat resize(Mat orig) {
        Mat simg = new Mat();
        int w = orig.width();
        int h = orig.height();

        Imgproc.resize(orig, simg, new Size(SCALE_FACTOR * w, SCALE_FACTOR * h));
        orig.release();

        return simg;
    }

    public static void initDetectors() {
        angleDetector = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
        angleDetector.read(System.getProperty("user.dir") + File.separator + "SingleClassPrototype" + File.separator + "Model" + File.separator + "xml" + File.separator + "blobrobotangle.xml");
        robotDetector = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
        robotDetector.read(System.getProperty("user.dir") + File.separator + "SingleClassPrototype" + File.separator + "Model" + File.separator + "xml" + File.separator + "blobrobot.xml");
    }

    public static Mat grayScale(Mat img) {
        Mat gray = new Mat();
        Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);

        return gray;
    }

    /*
    Returns x, y and r(adius) of the robot based on previous location
    need to translate to correct x,y,r
    */

    @Deprecated
    public static RoboPos retrieveRobot(Mat gray, RoboPos previousLoc) {
        //TODO Consider corner cases!

        int maxX = gray.cols();
        int maxY = gray.rows();
        int addX = 0;
        int addY = 0;

        int previousX = (int) previousLoc.getX();
        int previousY = (int) previousLoc.getY();
        int previousR = (int) previousLoc.getRadius();

        int searchSpace = previousR * 2;
        Rect rect = null;
        if ((previousX - searchSpace) < 0 && (previousY - searchSpace) < 0) { //top left
            rect = new Rect(new Point(0, 0), new Point(searchSpace, searchSpace));
            addX = 0;
            addY = 0;
        } else if ((previousX + searchSpace) > maxX && (previousY - searchSpace) < 0) { //top right
            rect = new Rect(new Point(maxX - searchSpace * 2, 0), new Point(maxX, searchSpace * 2));
            addX = maxX - searchSpace * 2;
            addY = 0;
        } else if ((previousX - searchSpace) < 0 && (previousY + searchSpace) > maxY) { //bottom left
            rect = new Rect(new Point(0, maxY - searchSpace * 2), new Point(searchSpace * 2, maxY));
            addX = 0;
            addY = maxY - searchSpace * 2;
        } else if ((previousX + searchSpace) > maxX && (previousY + searchSpace) > maxY) { //bottom right
            rect = new Rect(new Point(maxX - searchSpace * 2, maxY - searchSpace * 2), new Point(maxX, maxY));
            addX = maxX - searchSpace * 2;
            addY = maxY - searchSpace * 2;
        } else if ((previousX - searchSpace) < 0) { //left
            rect = new Rect(new Point(0, previousY - searchSpace), new Point(searchSpace * 2, previousY + searchSpace));
            addX = 0;
            addY = previousY - searchSpace;
        } else if ((previousY - searchSpace) < 0) { //top
            rect = new Rect(new Point(previousX - searchSpace, 0), new Point(previousX + searchSpace, searchSpace * 2));
            addX = previousX - searchSpace;
            addY = maxY - searchSpace * 2;
        } else if ((previousX + searchSpace) > maxX) { //right
            rect = new Rect(new Point(maxX - searchSpace * 2, previousY - searchSpace), new Point(maxX, previousY + searchSpace));
            addX = previousX - searchSpace * 2;
            addY = maxY - searchSpace;
        } else if ((previousY + searchSpace) > maxY) { //bottom
            addX = previousX - searchSpace;
            addY = maxY - searchSpace * 2;
        } else {
            rect = new Rect(new Point(previousX - searchSpace, previousY - searchSpace), new Point(previousX + searchSpace, previousY + searchSpace));
            addX = previousX - searchSpace;
            addY = previousY - searchSpace;
        }

        Mat cropped = gray.submat(rect);
        ImgWindow.newWindow(cropped);
        return retrieveRobot(cropped, addX, addY);
    }

    /*
    Returns x, y and r(adius) of the robot
     */

    @Deprecated
    public static RoboPos retrieveRobot(Mat gray, int addX, int addY) {
        Mat blur = new Mat();
        gray.copyTo(blur);

        int w = (int) (21 * SCALE_FACTOR);
        int h = (int) (21 * SCALE_FACTOR);

        if (w % 2 != 1) {
            w++;
        }

        if (h % 2 != 1) {
            h++;
        }

        Imgproc.GaussianBlur(blur, blur, new Size(w, h), 2, 2);

        ImgWindow.newWindow(blur);

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
            Imgproc.HoughCircles(blur, circles, Imgproc.CV_HOUGH_GRADIENT, dp + (c * step), 100);
            c++;
            s = circles.cols();

            if (s == 1) {
                int r = (int) circles.get(0, 0)[2] + 1;

                //TODO FIX when we have correct setup
//                if (!((r - 4) <= ROBOT_RADIUS && (r + 4) >= ROBOT_RADIUS)) {
//                    s = -1;
//                }
            }
        }

        if (circles.cols() != 1 || s == -1) {
            System.out.println("No / Multiple circles detected!");
            return null;
        }

        RoboPos result = new RoboPos(0, 0, 0);

        if (addX == 0 && addY == 0) {
            result.setPosition((int) circles.get(0, 0)[0] + 1, (int) circles.get(0, 0)[1] + 1);
            result.setRadius((int) circles.get(0, 0)[2] + 1);
        } else {
            result.setPosition((int) circles.get(0, 0)[0] + addX + 2, (int) circles.get(0, 0)[1] + addY + 2);
            result.setRadius((int) circles.get(0, 0)[2] + 1);
        }

        if (ComputerVision.DEBUG) {
            ImgWindow wnd = ImgWindow.newWindow(gray);

            while (!wnd.closed) {
                wnd.setImage(gray);
                Graphics2D g = wnd.begin();
                g.setColor(Color.WHITE);
                g.setStroke(new BasicStroke(3));
                for (int i = 0; i < circles.cols(); i++) {
                    double[] circle = circles.get(0, i);
                    g.drawOval((int) circle[0] - (int) circle[2], (int) circle[1] - (int) circle[2], (int) circle[2] * 2, (int) circle[2] * 2);
                }
                wnd.end();
            }
        }

        return result;
    }

    /*
    Returns points that together make up the contour polygon (aka maze polygon)
    Returned in [x1, y1, x2, y2, x3, y3, ... , xn, yn] format
     */
    public static MatOfPoint retrieveContour(Mat gray, RoboPos robotPos) {
        Mat equ = new Mat();
        gray.copyTo(equ);
        CLAHE clahe = Imgproc.createCLAHE(4.0 * SCALE_FACTOR, new Size(16.0 * SCALE_FACTOR, 16.0 * SCALE_FACTOR));
        clahe.apply(gray, equ);

        int blurSize = (int) (15 * SCALE_FACTOR);
        if (blurSize % 2 != 1) {
            blurSize += 1;
        }

        Mat blur = new Mat();
        equ.copyTo(blur);
        Imgproc.medianBlur(equ, blur, blurSize);

        Mat thresh = new Mat();
        blur.copyTo(thresh);
        Imgproc.threshold(blur, thresh, 0, 255, Imgproc.THRESH_OTSU);

        int offset = (int) (10 * SCALE_FACTOR);

        //Remove robot from picture (paint black)
        if (robotPos != null)
            Imgproc.rectangle(thresh, new Point(robotPos.getX() - robotPos.getRadius() - offset, robotPos.getY() - robotPos.getRadius() - offset), new Point(robotPos.getX() + robotPos.getRadius() + offset, robotPos.getY() + robotPos.getRadius() + offset), new Scalar(0), -1);

        /*findContours
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Vind grootste contour
        Verwijder alle andere

        Hierboven wordt de robot verwijderd! Maar de contour van de robot verwijderen zou beter zijn.
        */

        Mat dilateElement = new Mat();
        org.opencv.core.Point p = new org.opencv.core.Point(-1, -1);
        //Imgproc.dilate(thresh, thresh, dilateElement, p, (int) robotPos.getRadius());

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        if (ComputerVision.DEBUG) {
            Imgproc.drawContours(gray, contours, -1, new Scalar(0, 255, 0), 2);
        }

        double size = -1;
        MatOfPoint contour = null;

        for (MatOfPoint mop : contours) {
            if (mop.size().height > size) {
                size = mop.size().height;
                contour = mop;
            }
        }

        if (size == -1 || size < 100) {
            return null;
        }

        if (ComputerVision.DEBUG) {
            ImgWindow.newWindow(gray);
        }

        return contour;
    }

    private ArrayList<Integer> contourToList(MatOfPoint contour) {
        ArrayList<Integer> result = new ArrayList<>();

        for (int i = 0; i < contour.size().height; i++) {
            String[] tmp = contour.row(i).dump().substring(1, contour.row(i).dump().length() - 1).split(", ");
            result.add(Integer.parseInt(tmp[0]));
            result.add(Integer.parseInt(tmp[1]));
        }

        return result;
    }


    private static Mat featureProcessing(Mat m) {
        Mat equ = new Mat();
        CLAHE clahe = Imgproc.createCLAHE(4.0 * SCALE_FACTOR, new Size(16.0 * SCALE_FACTOR, 16.0 * SCALE_FACTOR));
        clahe.apply(m, equ);

        int w = (int) (9 * SCALE_FACTOR);
        int h = (int) (9 * SCALE_FACTOR);

        if (w % 2 != 1) {
            w++;
        }

        if (h % 2 != 1) {
            h++;
        }

        Imgproc.GaussianBlur(equ, equ, new Size(w, h), 2, 2);
        return equ;
    }

    private static KeyPoint[] processAngle(Mat m) {
        MatOfKeyPoint angleKeyPoints = new MatOfKeyPoint();
        angleDetector.detect(m, angleKeyPoints);
        KeyPoint[] angleKeyPointArray = angleKeyPoints.toArray();

        if (ComputerVision.DEBUG) {
            Features2d.drawKeypoints(m, angleKeyPoints, m, new Scalar(255), 2);
            ImgWindow.newWindow(m);
        }

        return angleKeyPointArray;
    }

    private static KeyPoint[] processRobot(Mat m) {
        MatOfKeyPoint robotKeyPoints = new MatOfKeyPoint();
        robotDetector.detect(m, robotKeyPoints);
        KeyPoint[] robotKeyPointArray = robotKeyPoints.toArray();

        if (ComputerVision.DEBUG) {
            Features2d.drawKeypoints(m, robotKeyPoints, m, new Scalar(255), 2);
            ImgWindow.newWindow(m);
        }

        return robotKeyPointArray;
    }

    private static List<Mat> getRGBChannels(Mat img) {
        List<Mat> channels = new ArrayList<Mat>();
        Core.split(img, channels);
        return channels;
    }

    public static KeyPoint[] robotv2(Mat img, int x, int y, int r) {
        int searchSpace = (int)(r * 2);

        int maxX = img.cols();
        int maxY = img.rows();
        int ax = 0;
        int ay = 0;

        Rect rect = null;

        if ((x - searchSpace) < 0 && (y - searchSpace) < 0) { //top left
            rect = new Rect(new Point(0, 0), new Point(searchSpace, searchSpace));
            ax = 0;
            ay = 0;
        } else if ((x + searchSpace) > maxX && (y - searchSpace) < 0) { //top right
            rect = new Rect(new Point(maxX - searchSpace * 2, 0), new Point(maxX, searchSpace * 2));
            ax = maxX - searchSpace * 2;
            ay = 0;
        } else if ((x - searchSpace) < 0 && (y + searchSpace) > maxY) { //bottom left
            rect = new Rect(new Point(0, maxY - searchSpace * 2), new Point(searchSpace * 2, maxY));
            ax = 0;
            ay = maxY - searchSpace * 2;
        } else if ((x + searchSpace) > maxX && (y + searchSpace) > maxY) { //bottom right
            rect = new Rect(new Point(maxX - searchSpace * 2, maxY - searchSpace * 2), new Point(maxX, maxY));
            ax = maxX - searchSpace * 2;
            ay = maxY - searchSpace * 2;
        } else if ((x - searchSpace) < 0) { //left
            rect = new Rect(new Point(0, y - searchSpace), new Point(searchSpace * 2, y + searchSpace));
            ax = 0;
            ay = y - searchSpace;
        } else if ((y - searchSpace) < 0) { //top
            rect = new Rect(new Point(x - searchSpace, 0), new Point(x + searchSpace, searchSpace * 2));
            ax = x - searchSpace;
            ay = maxY - searchSpace * 2;
        } else if ((x + searchSpace) > maxX) { //right
            rect = new Rect(new Point(maxX - searchSpace * 2, y - searchSpace), new Point(maxX, y + searchSpace));
            ax = x - searchSpace * 2;
            ay = maxY - searchSpace;
        } else if ((y + searchSpace) > maxY) { //bottom
            ax = x - searchSpace;
            ay = maxY - searchSpace * 2;
        } else {
            rect = new Rect(new Point(x - searchSpace, y - searchSpace), new Point(x + searchSpace, y + searchSpace));
            ax = x - searchSpace;
            ay = y - searchSpace;
        }

        Mat cropped = img.submat(rect);
        return robotv2(null, cropped, ax, ay);
    }

    public static KeyPoint[] robotv2(Mat orig, Mat img, int ax, int ay) {
        List<Mat> channels = getRGBChannels(orig);
        Mat r = channels.get(2);
        Mat g = channels.get(1);

        Mat pimg = featureProcessing(img);

        KeyPoint[] robotKeyPoint = processRobot(pimg);
        KeyPoint[] angleKeyPoint = processAngle(pimg);

        KeyPoint realRobotKeyPoint = null;
        KeyPoint realAngleKeyPoint = null;

        boolean robotSkip, angleSkip;

        if (robotKeyPoint.length < 1) {
            if (ComputerVision.DEBUG) {
                System.out.println("No robot detected");
            }
        }

        if (angleKeyPoint.length < 1) {
            if (ComputerVision.DEBUG) {
                System.out.println("No angle detected");
            }
        }

        if (robotKeyPoint.length == 1) {
            if (ComputerVision.DEBUG) {
                System.out.println("Standard detection- robot");
            }

            if (getBestKeyPoint(robotKeyPoint, true, orig) == null) {
                if (ComputerVision.DEBUG) {
                    System.out.println("Found one robot, but seems to be flawed (not passing color test)");
                }
            } else {
                realRobotKeyPoint = robotKeyPoint[0];
            }
        }

        if (angleKeyPoint.length == 1) {
            if (ComputerVision.DEBUG) {
                System.out.println("Standard detection- angle");
            }
            if (getBestKeyPoint(angleKeyPoint, false, orig) == null) {
                if (ComputerVision.DEBUG) {
                    System.out.println("Found one angle, but seems to be flawed (not passing color test)");
                }
            } else {
                realAngleKeyPoint = angleKeyPoint[0];
            }
        }

        if (robotKeyPoint.length > 1) {
            realRobotKeyPoint = getBestKeyPoint(robotKeyPoint, true, orig);
        }

        if (angleKeyPoint.length > 1) {
            realAngleKeyPoint = getBestKeyPoint(angleKeyPoint, false, orig);
        }

        if (ax != 0 && realRobotKeyPoint != null) {
            realRobotKeyPoint.pt.x += ax;
        }

        if (ax != 0 && realAngleKeyPoint != null) {
            realAngleKeyPoint.pt.x += ax;
        }

        if (ay != 0 && realRobotKeyPoint != null) {
            realRobotKeyPoint.pt.y += ay;
        }

        if (ay != 0 && realAngleKeyPoint != null) {
            realAngleKeyPoint.pt.y += ay;
        }

        return new KeyPoint[]{realRobotKeyPoint, realAngleKeyPoint};
    }

    private static KeyPoint getBestKeyPoint(KeyPoint[] keypoints, boolean robot, Mat img) {
        int rThreshold = 0;
        int gThreshold = 0;
        int bThreshold = 0;
        Mat cImg = null;

        if (robot) {
            rThreshold = 130; // 200
            gThreshold = 80; // 120
            bThreshold = 80; // 120
        } else {
            rThreshold = 90; // 180
            gThreshold = 90; // 180
            bThreshold = 90; // 230
        }

        KeyPoint keyPoint = null;
        int num = 0;

        for (KeyPoint kp : keypoints) {
            int robotX = (int) kp.pt.x;
            int robotY = (int) kp.pt.y;

            double[] bgr = img.get(robotY, robotX);

//            if (robot) {
//                System.out.println("robot");
//            } else {
//                System.out.println("angle");
//            }
//
//            System.out.println("b:" + bgr[0] + "/g:" + bgr[1] + "/r:" + bgr[2]);

            if (robot) {
                if (bgr[0] <= bThreshold && bgr[1] <= gThreshold && bgr[2] >= rThreshold) {
                    keyPoint = kp;
                    num++;
                }
            } else {
                if (bgr[0] <= bThreshold && bgr[1] >= gThreshold && bgr[2] <= rThreshold) {
                    keyPoint = kp;
                    num++;
                }
            }
        }

        if (num == 0) {
            if (ComputerVision.DEBUG) {
                System.out.println("No suitable keypoint found out of multiple.");
            }
        }

        if (num == 1) {
            if (ComputerVision.DEBUG) {
                System.out.println("Best KeyPoint found");
            }
        }

        if (num > 2) {
            if (ComputerVision.DEBUG) {
                System.out.println("Multiple suitable keypoints found.");
            }
            keyPoint = null;
        }

        return keyPoint;
    }

    //Note: kp.size is diameter, /2 gives ROUGHLY the radius
    //TODO incorporate checking on prev position

    //only for testing
    @Deprecated
    public static void bgsTest() {
        Mat gray = new Mat();
        Mat diff = new Mat();
        Mat prev = new Mat();
        Mat frame = new Mat();
        Mat fgMaskMOG2 = new Mat();
        Mat fgMaskKNN = new Mat();
        BackgroundSubtractorMOG2 mog2 = Video.createBackgroundSubtractorMOG2();
        BackgroundSubtractorKNN knn = Video.createBackgroundSubtractorKNN();
        ImgWindow mog2MaskWindow = ImgWindow.newWindow();
        mog2MaskWindow.setTitle("MOG2");
        //ImgWindow knnMaskWindow = ImgWindow.newWindow();
        //knnMaskWindow.setTitle("KNN");
        ImgWindow camWindow = ImgWindow.newWindow();
        camWindow.setTitle("CAM");
        mog2.setDetectShadows(false);
        VideoCapture capture = new VideoCapture(0);
        int i = 1;

        if (!capture.isOpened()) {
            System.out.println("error");
        } else {
            while(true) {
                if (capture.read(frame)) {
                    mog2.apply(frame, fgMaskMOG2, .001);
                    //knn.apply(frame, fgMaskKNN, .1);

                    Imgproc.rectangle(frame, new Point(10, 2), new Point(100, 20), new Scalar(255, 255, 255), -1);
                    Imgproc.putText(frame, String.valueOf(i), new Point(15, 15), 1, 1, new Scalar(0, 0, 0), 1);
                    i++;

                    camWindow.setImage(frame);
                    mog2MaskWindow.setImage(fgMaskMOG2);
                    //knnMaskWindow.setImage(fgMaskKNN);
                } else {
                    System.out.println("couldnt read frame!");
                    break;
                }
            }
        }

        capture.release();
    }

    public static Mat bgs(Mat m1, Mat m2) {
        Mat diff = new Mat();
        Core.absdiff(m1, m2, diff);
        Imgproc.cvtColor(diff, diff, Imgproc.COLOR_BGR2HSV);
        Scalar low = new Scalar(170, 50, 50);
        Scalar high = new Scalar(180, 200, 200);
        Core.inRange(diff, low, high, diff);
        Imgproc.dilate(diff, diff, new Mat(), new Point(-1, -1), 5);
        //diff = grayScale(diff);
        //Mat kernel = Mat.ones(5, 5, CvType.CV_8UC1);
        //Imgproc.morphologyEx(diff, diff, Imgproc.MORPH_OPEN, kernel);
        //Imgproc.GaussianBlur(diff, diff, new Size(5,5), 2, 2);
        //Imgproc.threshold(diff, diff, 0, 255, Imgproc.THRESH_OTSU);
        //kernel.release();
        return diff;
    }

    public static void bgsLoop() {
        //TODO:
        Mat bg = new Mat();
        Mat frame = new Mat();

        ImgWindow camWindow = ImgWindow.newWindow();
        camWindow.setTitle("CAM");

        ImgWindow bgsWindow = ImgWindow.newWindow();
        bgsWindow.setTitle("BGS");

        VideoCapture capture = new VideoCapture(0);

        int i = 1;

        List<MatOfPoint> contours;
        Mat hierarchy, crBg;
        Mat crFrame = null;
        int rad, space;
        int ax = 0;
        int ay = 0;
        Rect roi;
        Mat cropped, diff;
        double area;
        double[] h;
        float[] radius;
        float[] radiusD = null;
        Point center;
        Point centerD = null;
        boolean found = false;

        capture.set(Videoio.CAP_PROP_FRAME_WIDTH, 400);
        capture.set(Videoio.CAP_PROP_FRAME_HEIGHT, 600);

        if (!capture.isOpened()) {
            System.out.println("error");
        } else {
            while(true) {
                if (capture.read(frame)) {
                    long start = System.currentTimeMillis();
                    //Imgproc.rectangle(frame, new Point(10, 2), new Point(200, 20), new Scalar(255, 255, 255), -1);
                    //Imgproc.putText(frame, "FRAME: " + String.valueOf(i), new Point(15, 15), 1, 1, new Scalar(0, 0, 0), 1);
                    i++;
                    space = 0;

                    if (i < 90) {
                        camWindow.setImage(frame);
                    } else if (i == 120) {
                        frame.copyTo(bg);
                    } else if (i > 120 && i <= 240) {
                        diff = bgs(frame, bg);
                        bgsWindow.setImage(diff);
                        camWindow.setImage(frame);
                    } else if (i > 240) {
                        if (found) {
                            space = (int)(radiusD[0]*1.5);
                            System.out.println("x:" + (int)(centerD.x + ax - space)  + "|y:" + (int)(centerD.y + ay - space) + "|r:" + space);
                            System.out.println("ax:" + ax + ", ay:" + ay + ", space:" + space);
                            roi = new Rect(new Point((int)(centerD.x + ax - space), (int)(centerD.y + ay - space)), new Point((int)centerD.x + ax + space, (int)centerD.y + ay + space));
                            crFrame = frame.submat(roi);
                            crBg = bg.submat(roi);
                            diff = bgs(crFrame, crBg);
                        } else {
                            diff = bgs(frame, bg);
                        }

                        bgsWindow.setImage(diff);

                        contours = new ArrayList<MatOfPoint>();
                        hierarchy = new Mat();
                        Imgproc.findContours(diff, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
                        diff.release();

                        if (!contours.isEmpty()) {
                            radius = new float[1];
                            center = new Point();
                            Imgproc.minEnclosingCircle(new MatOfPoint2f(contours.get(0).toArray()), center, radius);
                            //Imgproc.circle(frame, center, (int)radius[0], new Scalar(255, 0, 0), 1);
                            rad = (int)(radius[0] * 1.5);

                            roi = new Rect(new Point((int)center.x - rad, (int)center.y - rad), new Point((int)center.x + rad, (int)center.y + rad));

                            ax = (int)center.x - rad;
                            ay = (int)center.y - rad;

                            cropped = frame.submat(roi);
                            Imgcodecs.imwrite("crop.jpg", cropped);

                            //Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_BGR2HSV);
                            //Scalar low = new Scalar(170, 100, 100);
                            //Scalar high = new Scalar(180, 200, 200);
                            //Core.inRange(cropped, low, high, diff);

                            //cropped = grayScale(cropped);

                            Mat cc = new Mat();
                            Imgproc.cvtColor(cropped, cc, Imgproc.COLOR_BGR2HSV);

                            Mat mask = new Mat();
                            Mat tmp_mask1 = new Mat();
                            Mat tmp_mask2 = new Mat();
                            Core.inRange(cc, new Scalar(0, 50, 50), new Scalar(10, 200, 200), tmp_mask1);
                            Core.inRange(cc, new Scalar(170, 50, 50), new Scalar(180, 200, 200), tmp_mask2);
                            Core.add(tmp_mask1, tmp_mask2, mask);
                            Mat kernel = Mat.ones(5, 5, CvType.CV_8UC1);
                            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);

                            contours = new ArrayList<MatOfPoint>();
                            hierarchy = new Mat();

                            //Imgproc.threshold(cropped, cropped, 0, 255, Imgproc.THRESH_OTSU);
                            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);
                            //Imgproc.drawContours(frame, contours, -1, new Scalar(255, 0, 0), 1);

                            area = 0;
                            found = false;

                            if (!contours.isEmpty() && !hierarchy.empty()) {
                                for (int j = 0; j < contours.size(); j++) {
                                    h = hierarchy.get(0, j);
                                    if (h[3] != -1) {
                                        area = Imgproc.contourArea(contours.get(j));
                                        if (area > 10) {
                                            //System.out.println("Angle (possibly) found");
                                            Rect r = Imgproc.boundingRect(contours.get(j));
                                            Imgproc.rectangle(frame, new Point(r.x + ax, r.y + ay), new Point(r.x+r.width+ax, r.y+r.height+ay), new Scalar(255));
                                        }
                                    } else if (h[2] != -1) {
                                        area = Imgproc.contourArea(contours.get(j));
                                        if (area > 100) {
                                            double kidArea = Imgproc.contourArea(contours.get((int)h[2]));
                                            if (kidArea > 10) {
                                                //System.out.println("Robot (possibly) found");
                                                found = false;
                                                radiusD = new float[1];
                                                centerD = new Point();
                                                //Imgproc.minEnclosingCircle(new MatOfPoint2f(contours.get(j).toArray()), centerD, radiusD);
                                                //Imgproc.circle(frame, new Point(centerD.x+ax, centerD.y+ay), (int)radiusD[0], new Scalar(255));
                                                RotatedRect r = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(j).toArray()));
                                                r.center.x += ax;
                                                r.center.y += ay;
                                                Imgproc.ellipse(frame, r, new Scalar(255), 1);
                                                //System.out.println("x:" + (centerD.x+ax) + ", y:" + (centerD.y+ay));
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        if (found) {
                            camWindow.setImage(crFrame);
                        } else {
                            camWindow.setImage(frame);
                        }
                    }

                    long end = System.currentTimeMillis();
                    System.out.println("Update took " + (end-start) + " ms");

                } else {
                    System.out.println("couldnt read frame!");
                    break;
                }
            }
        }

        capture.release();
    }

    public static void contourv2(Mat img) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(img, hsv, Imgproc.COLOR_BGR2HSV);

        Imgproc.GaussianBlur(hsv, hsv, new Size(5, 5), 2, 2);
        Mat mask = new Mat();
        Scalar low = new Scalar(14, 30, 130);
        Scalar high = new Scalar(23, 90, 255);
        Core.inRange(hsv, low, high, mask);
        Mat kernel = Mat.ones(17, 17, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        //Need to make sure that robot is removed.
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint contour = contours.get(i);
            if (Imgproc.contourArea(contour) < 500) {
                Imgproc.fillConvexPoly(mask, contour, new Scalar(0));
            }
        }

        //iterations = radius
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 10);

        contours = new ArrayList<MatOfPoint>();
        hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(img, contours, -1, new Scalar(0, 0, 255), 2);

        ImgWindow.newWindow(img);
    }


    public static void main(String[] args) {
        //bgsLoop();
        Mat pic = resize(readImg("C:\\Users\\Jyr\\IdeaProjects\\Mazes4\\SingleClassPrototype\\Input\\latestScreen.jpg"));
        contourv2(pic);
    }
}

//Robot radius
//Possible to improve on houghcircles? Preset variables?
//Don't expect robot detection at EVERY image!
//Centralize shortest path (redundancy)
//TODO: cycle retrieve robot method to all possible methods we have
//TODO: multiple color channels?
