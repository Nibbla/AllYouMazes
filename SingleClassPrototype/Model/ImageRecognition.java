package Model;

import Util.ImgWindow;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import java.util.ArrayList;
import java.util.List;

public class ImageRecognition{
    private Mat bg, frame, diff, hierarchy, cc, mask, tmp_mask1, tmp_mask2, kernel, hsv, r;

    private VideoCapture capture;

    private int numberOfFrames;

    private List<MatOfPoint> contours;
    private MatOfPoint currentContours, mazeContour, backgroundContour;


    private double area;
    private double[] h;
    private double ax, ay;

    private float prevRadius;
    private float[] radius;

    private Point prev;
    private Point center;
    private Point angle;

    private boolean found;
    private boolean croppingAreaKnown;

    private Rect croppedArea;
    private Rect roi;

    ImgWindow camWindow = null;
    ImgWindow bgsWindow = null;


    public ImageRecognition(){
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        initVars();
    }

    private void initVars(){
        bg = new Mat();
        frame = new Mat();
        diff = new Mat();
        hierarchy = new Mat();
        cc = new Mat();
        mask = new Mat();
        tmp_mask1 = new Mat();
        tmp_mask2 = new Mat();
        kernel = new Mat();
        hsv = new Mat();
        r = new Mat();

        numberOfFrames = 0;

        contours = new ArrayList<MatOfPoint>();
        mazeContour = null;
        backgroundContour = null;

        area = 0;
        h = new double[]{};

        ax = 0;
        ay = 0;

        prevRadius = 0;
        radius = null;

        prev = null;
        center = null;

        found = false;
        croppingAreaKnown = false;

        roi = new Rect();
        croppedArea = new Rect();

        if (ComputerVision.DEBUG) {
            camWindow = ImgWindow.newWindow();
            camWindow.setTitle("CAM");
            bgsWindow = ImgWindow.newWindow();
            bgsWindow.setTitle("BGS");
        }
    }

    public void readNextFrame(){


        long start = System.currentTimeMillis();

        capture.read(frame);

        long end = System.currentTimeMillis();
        System.out.println("Update took " + (end-start) + " ms");


        if (croppingAreaKnown){
            frame = frame.submat(croppedArea);
        } else {
            determineCroppedArea();
            frame = frame.submat(croppedArea);
        }

        if (ComputerVision.DEBUG) {
            camWindow.setImage(frame);
        }



        //Imgcodecs.imwrite("ReallyNicePicture.jpg", frame);
    }

    private void determineCroppedArea(){
        try {
            croppedArea = backgroundRect(frame);
            croppingAreaKnown = true;
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error determining cropping area.");
        }
    }

    private void determineRobotSearchArea(){
        readNextFrame();

        if (center != null && radius != null) {
            prev = center.clone();
            prevRadius = radius[0];
            Rect rect = rectSearch(frame, (int)center.x, (int)center.y, (int)(radius[0]*1.5));
            r = frame.submat(rect);
            diff = bgs(r);
            r.release();
        } else {
            diff = bgs(frame);
        }

        if (ComputerVision.DEBUG) {
            bgsWindow.setImage(diff);
        }
    }

    private void determineAngleSearchArea(){
        readNextFrame();

        if (center != null && radius != null) {
            prev = center.clone();
            prevRadius = radius[0];
            Rect rect = rectSearch(frame, (int)center.x, (int)center.y, (int)(radius[0]*3));
            r = frame.submat(rect);
            diff = bgsAngle(r);
            r.release();
        } else {
            diff = bgsAngle(frame);
        }

        if (ComputerVision.DEBUG) {
            bgsWindow.setImage(diff);
        }
    }

    private Rect backgroundRect(Mat fullimage){
        backgroundContour = null;
        Imgproc.cvtColor(fullimage, hsv, Imgproc.COLOR_BGR2HSV);

        Imgproc.GaussianBlur(hsv, hsv, new Size(5, 5), 2, 2);

        Scalar low = new Scalar(0, 0, 0);
        Scalar high = new Scalar(180, 255, 30);
        Core.inRange(hsv, low, high, mask);
        kernel = Mat.ones(17, 17, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int biggestContourIndex = 0;


        for(int index = 0; index < contours.size(); index++){
            if (contours.get(index).size().area() >= contours.get(biggestContourIndex).size().area()){
                biggestContourIndex = index;
            }
        }

        hsv.release();
        mask.release();
        hierarchy.release();
        kernel.release();

        return Imgproc.boundingRect(contours.get(biggestContourIndex));
    }

    private Rect rectSearch(Mat img, int x, int y, int searchSpace) {
        Rect rect = null;
        int maxX = img.cols();
        int maxY = img.rows();

        if ((x - searchSpace) < 0 && (y - searchSpace) < 0) { //top left
            rect = new Rect(new Point(0, 0), new Point(searchSpace, searchSpace));
        } else if ((x + searchSpace) > maxX && (y - searchSpace) < 0) { //top right
            rect = new Rect(new Point(maxX - searchSpace * 2, 0), new Point(maxX, searchSpace * 2));
            ax = maxX - searchSpace * 2;
        } else if ((x - searchSpace) < 0 && (y + searchSpace) > maxY) { //bottom left
            rect = new Rect(new Point(0, maxY - searchSpace * 2), new Point(searchSpace * 2, maxY));
            ay = maxY - searchSpace * 2;
        } else if ((x + searchSpace) > maxX && (y + searchSpace) > maxY) { //bottom right
            rect = new Rect(new Point(maxX - searchSpace * 2, maxY - searchSpace * 2), new Point(maxX, maxY));
            ax = maxX - searchSpace * 2;
            ay = maxY - searchSpace * 2;
        } else if ((x - searchSpace) < 0) { //left
            rect = new Rect(new Point(0, y - searchSpace), new Point(searchSpace * 2, y + searchSpace));
            ay = y - searchSpace;
        } else if ((y - searchSpace) < 0) { //top
            rect = new Rect(new Point(x - searchSpace, 0), new Point(x + searchSpace, searchSpace * 2));
            ax = x - searchSpace;
        } else if ((x + searchSpace) > maxX) { //right
            rect = new Rect(new Point(maxX - searchSpace * 2, y - searchSpace), new Point(maxX, y + searchSpace));
            ax = maxX - searchSpace * 2;
            ay = y - searchSpace;
        } else if ((y + searchSpace) > maxY) { //bottom
            rect = new Rect(new Point(x - searchSpace, maxY - searchSpace * 2), new Point(x + searchSpace, maxY));
            ax = x - searchSpace;
            ay = maxY - searchSpace * 2;
        } else {
            rect = new Rect(new Point(x - searchSpace, y - searchSpace), new Point(x + searchSpace, y + searchSpace));
            ax = x - searchSpace;
            ay = y - searchSpace;
        }

        return rect;
    }

    private Mat bgs(Mat m1) {
        //Format I encountered is 360 (degrees), 100 (percent), 100 (percent)
        //HSV in OpenCV is 180, 255, 255 format.
        mask.release();
        m1.copyTo(cc);
        Imgproc.cvtColor(cc, cc, Imgproc.COLOR_BGR2HSV);
        Core.inRange(cc, new Scalar(0, 160, 160), new Scalar(15, 220, 210), tmp_mask1);
        Core.inRange(cc, new Scalar(170, 160, 160), new Scalar(180, 220, 210), tmp_mask2);
        Core.add(tmp_mask1, tmp_mask2, mask);
        kernel = Mat.ones(10, 10, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        cc.release();
        tmp_mask1.release();
        tmp_mask2.release();
        kernel.release();
        return mask;
    }

    private Mat bgsAngle(Mat m1) {
        //Format I encountered is 360 (degrees), 100 (percent), 100 (percent)
        //HSV in OpenCV is 180, 255, 255 format.
        mask.release();
        m1.copyTo(cc);
        Imgproc.cvtColor(cc, cc, Imgproc.COLOR_BGR2HSV);
        Core.inRange(cc, new Scalar(30, 60, 50), new Scalar(70, 180, 150), mask);
        kernel = Mat.ones(3, 3, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        cc.release();
        kernel.release();

        return mask;
    }

    private MatOfPoint contours(Mat img) {
        Imgproc.cvtColor(img, hsv, Imgproc.COLOR_BGR2HSV);

        Imgproc.GaussianBlur(hsv, hsv, new Size(15, 15), 2, 2);
        Core.inRange(hsv, new Scalar(10, 50, 115), new Scalar(40, 165, 185), mask);
        //Core.inRange(hsv, new Scalar(165, 30, 50), new Scalar(180, 190, 230), tmp_mask2);
        //Core.add(tmp_mask1, tmp_mask2, mask);
        kernel = Mat.ones(14, 14, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        //Need to make sure that robot is removed.
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        //Imgproc.drawContours(img, contours, -1, new Scalar(255, 0, 0), 3);

        while (contours.size() != 1) {
            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint contour = contours.get(i);
                if (Imgproc.contourArea(contour) < 25000) {
                    Imgproc.fillConvexPoly(mask, contour, new Scalar(0));
                }
            }
            contours.clear();
            hierarchy.release();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        }

        //iterations = radius
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 25);
        contours.clear();
        hierarchy.release();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//        hierarchy.release();
//        contours.clear();
//        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


//        if (contours.size() == 1) {
//            mazeContour = contours.get(0);
//        } else {
//            if (contours.isEmpty()) System.out.println("No contours");
//            else System.out.println("Found multiple contours");
//        }

        if (ComputerVision.CONTOUR_TEST) {
            Imgproc.drawContours(img, contours, -1, new Scalar(0, 0, 255), 2);
            ImgWindow w = ImgWindow.newWindow();
            w.setTitle("contour v2");
            w.setImage(img);
        }

        hsv.release();
        mask.release();
        tmp_mask1.release();
        tmp_mask2.release();
        kernel.release();
        hierarchy.release();

        return contours.get(0);
    }

    public void initCamera(int width, int height, int startupTimeMS){
        try {
            capture = new VideoCapture(0);
            capture.set(Videoio.CAP_PROP_FRAME_WIDTH, width);
            capture.set(Videoio.CAP_PROP_FRAME_HEIGHT, height);
            //width
            //capture.set(3, width);
            //height
            //capture.set(4, height);
            //fps
            //capture.set(5, 60);
            //brightness
            //capture.set(10, 0.5);
            //contrast
            //capture.set(11, 0.65);
            //saturation
            //capture.set(12, 0.9);
            //convert to RGB
            //capture.set(16, 1);
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error initializing camera.");
        }

        try {
            Thread.sleep(startupTimeMS);
        } catch (InterruptedException e) {
            e.printStackTrace();
            System.out.println("Error waiting for camera initialization.");
        }
    }

    public void determineMazeContours(){
        readNextFrame();
        frame.copyTo(bg);
        currentContours = contours(bg);
        bg.release();
    }

    public void findAnglePosition(){
        if (found){
            determineAngleSearchArea();
            boolean foundPoint = false;

            contours.clear();

            Imgproc.findContours(diff, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            int biggestContour = 0;
            Point furthestPoint = new Point();
            double furthestDistance = 0;

            if (!contours.isEmpty() && !hierarchy.empty()) {
                for (int j = 0; j < contours.size(); j++) {
                    if (contours.get(j).size().area() >= contours.get(biggestContour).size().area()){
                        foundPoint = true;
                        biggestContour = j;
                    }
                }

                for (Point contourPoint: contours.get(biggestContour).toArray()) {
                    double distance = Math.sqrt(Math.pow(((center.x - ax) - contourPoint.x),2) + Math.pow(((center.y - ay) - contourPoint.y),2));

                    if(distance >= furthestDistance){
                        furthestDistance = distance;
                        furthestPoint = contourPoint;
                    }
                }

            }

            if (!foundPoint) {
                angle = center;
                ax = 0;
                ay = 0;
            } else {
                angle = new Point(furthestPoint.x + ax, furthestPoint.y + ay);
            }

            hierarchy.release();
            //frame.release();
            diff.release();
        }

    }

    public void findRobotPosition(){
        determineRobotSearchArea();

        found = false;
        area = 0;

        contours.clear();

        Imgproc.findContours(diff, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        if (!contours.isEmpty() && !hierarchy.empty()) {
            for (int j = 0; j < contours.size(); j++) {

                h = hierarchy.get(0, j);
                //if it has a parent
                if (h[3] != -1) {
                    area = Imgproc.contourArea(contours.get(j));
                    if (area > 20) {
                        //System.out.println("Angle (possibly) found");
                        RotatedRect r = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(j).toArray()));
                        r.center.x += ax;
                        r.center.y += ay;

                        if (ComputerVision.DEBUG) {
                            Imgproc.ellipse(frame, r, new Scalar(0, 255, 0), 1);
                        }


                    }
                    //if it has a child
                } else if (h[2] != -1) {
                    area = Imgproc.contourArea(contours.get(j));
                    if (area > 200) {
                        double kidArea = Imgproc.contourArea(contours.get((int)h[2]));
                        if (kidArea > 20) {

                            boolean invalid = false;
                            int maxDiffPos = 20;
                            int maxDiffRad = 15;
                            if (prev != null) {
                                if (Math.abs(prev.x - center.x) > maxDiffPos || Math.abs(prev.y - center.y) > maxDiffPos || Math.abs(prevRadius - radius[0]) > maxDiffRad) {
                                    if (ComputerVision.DEBUG) {
                                        //System.out.println("Point INVALIDATED!");
                                        invalid = true;
                                    }
                                }
                            }

                            if (!invalid) {
                                //System.out.println("Robot (possibly) found");
                                found = true;
                                radius = new float[1];
                                Imgproc.minEnclosingCircle(new MatOfPoint2f(contours.get(j).toArray()), null, radius);
                                RotatedRect r = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(j).toArray()));
                                r.center.x += ax;
                                r.center.y += ay;

                                center = new Point(r.center.x, r.center.y);
                                if (ComputerVision.DEBUG) {
                                    Imgproc.ellipse(frame, r, new Scalar(0, 255, 0), 1);
                                }


                            }
                        }
                    }
                }
            }


            hierarchy.release();
            //frame.release();
            diff.release();

        }

        if (!found) {
            prev = null;
            prevRadius = 0;
            radius = null;
            center = null;
            ax = 0;
            ay = 0;
        }

    }

    public void stopImagerecognition(){
        capture.release();
    }

    public Point getCenter() {
        return center;
    }

    public Point getPrev() {
        return prev;
    }

    public MatOfPoint getCurrentContours() {
        readNextFrame();
        currentContours = contours(frame);
        return currentContours;
    }

    public float getRadius() {
        return radius[0];
    }

    public Mat getFrame() {
        readNextFrame();
        return frame;
    }

    public Point getAngle() {
        return angle;
    }
}