package Model;

import Simulation.Simulation;
import Util.ImgWindow;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;
import org.opencv.utils.Converters;

import java.util.ArrayList;
import java.util.List;

public class ImageRecognition {
    ImgWindow camWindow = null;
    ImgWindow bgsWindow = null;
    private Mat bg, frame, diff, hierarchy, cc, mask, tmp_mask1, tmp_mask2, kernel, hsv, r;
    private VideoCapture capture;
    private int numberOfFrames;
    private List<MatOfPoint> contours;
    private MatOfPoint currentContours;
    private double area;
    private double[] h;
    private double ax, ay;
    private float prevRadius;
    private float[] radius;
    private Point prev;
    private Point center;
    private Point angle;
	private Point cornerTL, cornerTR, cornerBL, cornerBR;
    private boolean found;
    private boolean croppingAreaKnown;
    private Rect croppedArea;


    public ImageRecognition() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        initVars();
    }

    private void initVars() {
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

        croppedArea = new Rect();

        if (Simulation.DEBUG_CV_ROBOT_ANGLE_DETECTION) {
            camWindow = ImgWindow.newWindow();
            camWindow.setTitle("CAM");
            bgsWindow = ImgWindow.newWindow();
            bgsWindow.setTitle("BGS");
        }
    }

    private void readNextFrame() {
        capture.read(frame);

		
        if (croppingAreaKnown) {
            frame = frame.submat(croppedArea);
        } else {
            determineCroppedArea();
            frame = frame.submat(croppedArea);
        }
			
		perspective();
		System.out.println("test");
        if (Simulation.DEBUG_CV_ROBOT_ANGLE_DETECTION) {
            camWindow.setImage(frame);
        }

    }

	private void perspective(){
		cornerTL = new Point(cornerTL.x - croppedArea.tl().x, cornerTL.y - croppedArea.tl().y);
		cornerTR = new Point(cornerTR.x - croppedArea.tl().x, cornerTR.y - croppedArea.tl().y);
		cornerBL = new Point(cornerBL.x - croppedArea.tl().x, cornerBL.y - croppedArea.tl().y);
		cornerBR = new Point(cornerBR.x - croppedArea.tl().x, cornerBR.y - croppedArea.tl().y);


		MatOfPoint2f corners = new MatOfPoint2f(cornerTL, cornerTR, cornerBL, cornerBR);

		double maxwidth = cornerBR.x - cornerBL.x;
		double maxheight = cornerBR.y - cornerTL.y;

		MatOfPoint2f target = new MatOfPoint2f(new Point(0,0),new Point(maxwidth-1,0),new Point(0,maxheight-1),new Point(maxwidth-1,maxheight-1));
		

		Mat trans = Imgproc.getPerspectiveTransform(Converters.vector_Point2f_to_Mat(corners.toList()), Converters.vector_Point2f_to_Mat(target.toList()));
		

		Mat tmpFrame = frame.clone();
		//frame.release();
		
		Imgproc.warpPerspective(frame, frame, trans, new Size(maxwidth, maxheight));

		//trans.release();
		//tmpFrame.release();
	}

    private void determineCroppedArea() {
        try {
            croppedArea = backgroundRect(frame);
            croppingAreaKnown = true;
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error determining cropping area.");
        }
    }

    private void determineRobotSearchArea() {

        if (center != null && radius != null) {
            prev = center.clone();
            prevRadius = radius[0];
            Rect rect = rectSearch(frame, (int) center.x, (int) center.y, (int) (radius[0] * 1.5));
            r = frame.submat(rect);
            diff = bgs(r);
            r.release();
        } else {
            diff = bgs(frame);
        }

        if (Simulation.DEBUG_CV_ROBOT_ANGLE_DETECTION) {
            bgsWindow.setImage(diff);
        }
    }

    private void determineAngleSearchArea() {

        if (center != null && radius != null) {
            prev = center.clone();
            prevRadius = radius[0];
            Rect rect = rectSearch(frame, (int) center.x, (int) center.y, (int) (radius[0] * 3));
            r = frame.submat(rect);
            diff = bgsAngle(r);
            r.release();
        } else {
            diff = bgsAngle(frame);
        }

        if (Simulation.DEBUG_CV_ROBOT_ANGLE_DETECTION) {
            bgsWindow.setImage(diff);
        }
    }

    public Rect backgroundRect(Mat fullimage) {
        Imgproc.cvtColor(fullimage, hsv, Imgproc.COLOR_BGR2HSV);

        Imgproc.GaussianBlur(hsv, hsv, new Size(9, 9), 2, 2);

        //Core.inRange(hsv, new Scalar(70, 10, 20), new Scalar(200, 150, 55), tmp_mask1);
        //Core.inRange(hsv, new Scalar(0, 10, 20), new Scalar(40, 150, 55), tmp_mask2);

        Core.inRange(hsv, new Scalar(0, 10, 0), new Scalar(125, 255, 20), tmp_mask1);
        Core.inRange(hsv, new Scalar(0, 10, 0), new Scalar(125, 255, 20), tmp_mask2);

        Core.add(tmp_mask1, tmp_mask2, mask);

        kernel = Mat.ones(7, 7, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        int biggestContourIndex = 0;


        for (int index = 0; index < contours.size(); index++) {
            if (contours.get(index).size().area() >= contours.get(biggestContourIndex).size().area()) {
                biggestContourIndex = index;
            }
        }

        hsv.release();
        mask.release();
        hierarchy.release();
        kernel.release();

		Point[] matrix = contours.get(biggestContourIndex).toArray();

        Point topLeft = matrix[0];
        Point topRight = matrix[0];
        Point bottomLeft = matrix[0];
        Point bottomRight = matrix[0];

        Point topLeftOrig = new Point(0,0);
        Point topRightOrig = new Point(400,0);
        Point bottomLeftOrig = new Point(0,600);
        Point bottomRightOrig = new Point(400,600);

        for (Point p:matrix) {
            if (Math.sqrt(Math.pow((topLeftOrig.x - p.x),2) + Math.pow((topLeftOrig.y - p.y),2)) < Math.sqrt(Math.pow((topLeftOrig.x - topLeft.x),2) + Math.pow((topLeftOrig.y - topLeft.y),2))){
                topLeft = p;
            }
            if (Math.sqrt(Math.pow((topRightOrig.x - p.x),2) + Math.pow((topRightOrig.y - p.y),2)) < Math.sqrt(Math.pow((topRightOrig.x - topRight.x),2) + Math.pow((topRightOrig.y - topRight.y),2))){
                topRight = p;
            }
            if (Math.sqrt(Math.pow((bottomLeftOrig.x - p.x),2) + Math.pow((bottomLeftOrig.y - p.y),2)) < Math.sqrt(Math.pow((bottomLeftOrig.x - bottomLeft.x),2) + Math.pow((bottomLeftOrig.y - bottomLeft.y),2))){
                bottomLeft = p;
            }
            if (Math.sqrt(Math.pow((bottomRightOrig.x - p.x),2) + Math.pow((bottomRightOrig.y - p.y),2)) < Math.sqrt(Math.pow((bottomRightOrig.x - bottomRight.x),2) + Math.pow((bottomRightOrig.y - bottomRight.y),2))){
                bottomRight = p;
            }
        }

        System.out.println(topLeft + " " + topRight + " " +bottomLeft + " " + bottomRight);

		cornerTL = topLeft;
		cornerTR = topRight;
		cornerBL = bottomLeft;
		cornerBR = bottomRight;

		

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

        Core.inRange(cc, new Scalar(0, 90, 100), new Scalar(10, 200, 210), tmp_mask1);
        Core.inRange(cc, new Scalar(170, 90, 100), new Scalar(180, 200, 210), tmp_mask2);

		//Core.inRange(cc, new Scalar(0, 90, 130), new Scalar(10, 200, 255), tmp_mask1);
        //Core.inRange(cc, new Scalar(170, 90, 130), new Scalar(180, 200, 255), tmp_mask2);

        Core.add(tmp_mask1, tmp_mask2, mask);
        kernel = Mat.ones(3, 3, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);

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

        //Core.inRange(cc, new Scalar(120, 35, 120), new Scalar(170, 85, 180), mask);
		Core.inRange(cc, new Scalar(100, 25, 50), new Scalar(170, 105, 105), mask);

        kernel = Mat.ones(7, 7, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        cc.release();
        kernel.release();

        return mask;
    }

    private MatOfPoint contours(Mat img) {
        Imgproc.cvtColor(img, hsv, Imgproc.COLOR_BGR2HSV);

        Imgproc.GaussianBlur(hsv, hsv, new Size(15, 15), 2, 2);
        Core.inRange(hsv, new Scalar(10, 20, 160), new Scalar(30, 165, 255), mask);
        //Core.inRange(hsv, new Scalar(165, 30, 50), new Scalar(180, 190, 230), tmp_mask2);
        //Core.add(tmp_mask1, tmp_mask2, mask);
        kernel = Mat.ones(14, 14, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        //Need to make sure that robot is removed.
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        while (contours.size() != 1) {
            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint contour = contours.get(i);
                if (Imgproc.contourArea(contour) < 10000) {
                    Imgproc.fillConvexPoly(mask, contour, new Scalar(0));
                }
            }
            contours.clear();
            hierarchy.release();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        }

        //iterations = radius
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 21);
        contours.clear();
        hierarchy.release();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        if (Simulation.DEBUG_CV_CONTOURS) {
            Imgproc.drawContours(img, contours, -1, new Scalar(0, 0, 255), 2);
            ImgWindow w = ImgWindow.newWindow();
            w.setTitle("contours");
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

    public void initCamera(int width, int height, int startupTimeMS, int skip) {
        try {
            capture = new VideoCapture(0);
            capture.set(Videoio.CAP_PROP_FRAME_WIDTH, width);
            capture.set(Videoio.CAP_PROP_FRAME_HEIGHT, height);
            //width
            //capture.set(3, width);
            //height
            //capture.set(4, height);
            //fps
            //capture.set(5, 30);
            //brightness
            //capture.set(10, 0.5);
            //contrast
            //capture.set(11, 0.65);
            //saturation
            //capture.set(12, 0.7);
            //convert to RGB
            //capture.set(16, 1);
            int i = 0;
            while (i < skip) {
                capture.read(frame);
                i++;
            }

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

    public void determineMazeContours() {
        frame.copyTo(bg);
        currentContours = contours(bg);
        bg.release();
    }

    public void findAnglePosition() {
        if (found) {
            determineAngleSearchArea();
            boolean foundPoint = false;

            contours.clear();

            Imgproc.findContours(diff, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            int biggestContour = 0;
            Point furthestPoint = new Point();
            double furthestDistance = 0;

            if (!contours.isEmpty() && !hierarchy.empty()) {
                for (int j = 0; j < contours.size(); j++) {
                    if (contours.get(j).size().area() >= contours.get(biggestContour).size().area()) {
                        foundPoint = true;
                        biggestContour = j;
                    }
                }

                for (Point contourPoint : contours.get(biggestContour).toArray()) {
                    double distance = Math.sqrt(Math.pow(((center.x - ax) - contourPoint.x), 2) + Math.pow(((center.y - ay) - contourPoint.y), 2));

                    if (distance >= furthestDistance) {
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
            diff.release();
        }

    }

    public void findRobotPosition() {
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

                        if (Simulation.DEBUG_CV_ROBOT_ANGLE_DETECTION) {
                            Imgproc.ellipse(frame, r, new Scalar(0, 255, 0), 1);
                        }


                    }
                    //if it has a child
                } else if (h[2] != -1) {
                    area = Imgproc.contourArea(contours.get(j));
                    if (area > 200) {
                        double kidArea = Imgproc.contourArea(contours.get((int) h[2]));
                        if (kidArea > 20) {

                            boolean invalid = false;
                            int maxDiffPos = 20;
                            int maxDiffRad = 15;
                            if (prev != null) {
                                if (Math.abs(prev.x - center.x) > maxDiffPos || Math.abs(prev.y - center.y) > maxDiffPos || Math.abs(prevRadius - radius[0]) > maxDiffRad) {
                                    if (Simulation.DEBUG_CV_ROBOT_ANGLE_DETECTION) {
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
                                if (Simulation.DEBUG_CV_ROBOT_ANGLE_DETECTION) {
                                    Imgproc.ellipse(frame, r, new Scalar(0, 255, 0), 1);
                                }


                            }
                        }
                    }
                }
            }


            hierarchy.release();
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

    public void stopImagerecognition() {
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

    public void releaseFrame() {
        frame.release();
    }

    public Point getAngle() {
        return angle;
    }

}
