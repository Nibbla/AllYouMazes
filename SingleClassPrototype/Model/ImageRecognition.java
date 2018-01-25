package Model;

import Simulation.Simulation;
import Util.ImgWindow;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;
import org.opencv.utils.Converters;
import org.opencv.imgcodecs.Imgcodecs;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class ImageRecognition {
    ImgWindow camWindow = null;
    //ImgWindow bgsWindow = null;
    private ImgWindow bgsWindowRobot;
    private ImgWindow bgsWindowAngle;
    private ImgWindow bgsWindowObject;

    private Mat bg, frame, diff, hierarchy, cc, mask, tmp_mask1, tmp_mask2, kernel, hsv, r, perspectiveTransform, transformation_x, transformation_y;
    private VideoCapture capture;
    private int numberOfFrames;
    private List<MatOfPoint> contours;
    private MatOfPoint currentContours;
    private MatOfPoint backgroundContour;
    private double area;
    private double[] h;
    private double ax, ay;
    private double perspectiveWidth, perspectiveHeight;
    private float prevRadius;
    private float[] radius;
    private Point prev;
    private Point center;
    private Point angle;
    private Point object;
	private Point cornerTL, cornerTR, cornerBL, cornerBR;
    private boolean found;
    private boolean croppingAreaKnown;
    private Rect croppedArea;
    private Scalar angleScalar1 = new Scalar(85, 55, 130);
    private Scalar angleScalar2 = new Scalar(115, 115, 210);
    private Scalar contourScalar1 =  new Scalar(15, 10, 150);
    private Scalar contourScalar2 = new Scalar(40, 105, 230);
    private Scalar backgroundScalar1 =new Scalar(0, 0, 10);
    private Scalar backgroundScalar2 = new Scalar(180, 255, 60);

    private Scalar robotBgs1_1 = new Scalar(135, 30, 160);
    private Scalar robotBgs1_2 = new Scalar(165, 80, 235);
    //private Scalar robotBgs2_1 = new Scalar(170, 90, 100);
    //private Scalar robotBgs2_2 = new Scalar(180, 200, 210);

    private Scalar objectBgs1_1 = new Scalar(65, 60, 75);
    private Scalar objectBgs1_2 =new Scalar(75, 115, 170);

    private Mat[] byPassImages = null;
    private boolean byPassCamera = false;
    private long byPassCount = 0;
    private MatOfPoint currentContoursWithoutDialation;


    public ImageRecognition(boolean debug) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        initVars(debug);
    }

    private void initVars(boolean debug) {
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
        object = null;

        found = false;
        croppingAreaKnown = false;

        croppedArea = new Rect();

        if (Simulation.DEBUG_CV_ROBOT_ANGLE_DETECTION || debug) {
            camWindow = ImgWindow.newWindow();
            camWindow.setTitle("CAM");
            bgsWindowRobot = ImgWindow.newWindow();
            bgsWindowRobot.setTitle("BGSRobot");
            bgsWindowAngle = ImgWindow.newWindow();
            bgsWindowAngle.setTitle("BGSAngle");
            bgsWindowObject = ImgWindow.newWindow();
            bgsWindowObject.setTitle("BGSObject");

        }

    }

    private void readNextFrame() {
        if (byPassCamera){
            byPassCount= (byPassCount+1)%byPassImages.length;
            frame = byPassImages[(int) byPassCount].clone();
        }else {
            capture.read(frame);
        }
		
		Imgcodecs.imwrite("bigInitialImage.jpg", frame);

        if (croppingAreaKnown) {
            frame = frame.submat(croppedArea);
            applyPerspective();
        } else {
            determineCroppedArea();
            frame = frame.submat(croppedArea);
            long start = System.currentTimeMillis(); 	

            determinePerspective();

			long end = System.currentTimeMillis();
            System.out.println("Transform1: " + (end - start) + " ms");
			start = System.currentTimeMillis(); 	
            
            applyPerspective();

			end = System.currentTimeMillis();
            System.out.println("Transform: " + (end - start) + " ms");
        }
			
        if (Simulation.DEBUG_CV_ROBOT_ANGLE_DETECTION) {
            camWindow.setImage(frame);
        }

    }

    private void determinePerspective(){
        Point[] matrix = backgroundContour.toArray();

        Point topLeft = matrix[0];
        Point topRight = matrix[0];
        Point bottomLeft = matrix[0];
        Point bottomRight = matrix[0];

        Point topLeftOrig = new Point(0,0);
        Point topRightOrig = new Point(600,0);
        Point bottomLeftOrig = new Point(0,400);
        Point bottomRightOrig = new Point(600,400);

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

        cornerTL = topLeft;
        cornerTR = topRight;
        cornerBL = bottomLeft;
        cornerBR = bottomRight;

        cornerTL = new Point(cornerTL.x - croppedArea.tl().x, cornerTL.y - croppedArea.tl().y);
        cornerTR = new Point(cornerTR.x - croppedArea.tl().x, cornerTR.y - croppedArea.tl().y);
        cornerBL = new Point(cornerBL.x - croppedArea.tl().x, cornerBL.y - croppedArea.tl().y);
        cornerBR = new Point(cornerBR.x - croppedArea.tl().x, cornerBR.y - croppedArea.tl().y);

        MatOfPoint2f corners = new MatOfPoint2f(cornerTL, cornerTR, cornerBL, cornerBR);

        perspectiveWidth = cornerBR.x - cornerBL.x;
        perspectiveHeight = cornerBR.y - cornerTL.y;

        MatOfPoint2f target = new MatOfPoint2f(new Point(0,0),new Point(perspectiveWidth-1,0),new Point(0,perspectiveHeight-1),new Point(perspectiveWidth-1,perspectiveHeight-1));

        perspectiveTransform = Imgproc.getPerspectiveTransform(Converters.vector_Point2f_to_Mat(corners.toList()), Converters.vector_Point2f_to_Mat(target.toList()));


        // Since the camera won't be moving, let's pregenerate the remap LUT
        Mat inverseTransMatrix = perspectiveTransform.inv();

        // Generate the warp matrix
        Mat map_x = new Mat();
        Mat map_y = new Mat();
        Mat srcTM;
        srcTM = inverseTransMatrix.clone(); // If WARP_INVERSE, set srcTM to transformationMatrix

        map_x.create(frame.size(), CvType.CV_32FC1);
        map_y.create(frame.size(), CvType.CV_32FC1);

	
        double M11, M12, M13, M21, M22, M23, M31, M32, M33;
        M11 = srcTM.get(0,0)[0];
        M12 = srcTM.get(0,1)[0];
        M13 = srcTM.get(0,2)[0];
        M21 = srcTM.get(1,0)[0];
        M22 = srcTM.get(1,1)[0];
        M23 = srcTM.get(1,2)[0];
        M31 = srcTM.get(2,0)[0];
        M32 = srcTM.get(2,1)[0];
        M33 = srcTM.get(2,2)[0];

        for (int y = 0; y < frame.rows(); y++) {
            double fy = (double)y;
            for (int x = 0; x < frame.cols(); x++) {
                double fx = (double)x;
                double w = ((M31 * fx) + (M32 * fy) + M33);
                w = w != 0.0f ? 1.f / w : 0.0f;
                float new_x = (float) (((M11 * fx) + (M12 * fy) + M13) * w);
                float new_y = (float) (((M21 * fx) + (M22 * fy) + M23) * w);
                map_x.put(y,x,new_x);
                map_y.put(y,x,new_y);
            }
        }

// This creates a fixed-point representation of the mapping resulting in ~4% CPU savings
		transformation_x = new Mat();
		transformation_y = new Mat();
        transformation_x.create(frame.size(), CvType.CV_16SC2);
        transformation_y.create(frame.size(), CvType.CV_16UC1);
        Imgproc.convertMaps(map_x, map_y, transformation_x, transformation_y, CvType.CV_16SC2, false);

// If the fixed-point representation causes issues, replace it with this code
//transformation_x = map_x.clone();
//transformation_y = map_y.clone();
    }

    private void applyPerspective(){
        Imgproc.warpPerspective(frame, frame, perspectiveTransform, new Size(perspectiveWidth, perspectiveHeight));
        //Imgproc.remap(frame, frame, transformation_x, transformation_y, Imgproc.INTER_LINEAR);
    }

    private void determineCroppedArea() {
        try {
            backgroundContour = backgroundRect(frame);
            croppedArea = Imgproc.boundingRect(backgroundContour);
            croppingAreaKnown = true;
        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("Error determining cropping area.");
        }
    }

    private void determineRobotSearchArea(boolean debug) {

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

        if (Simulation.DEBUG_CV_ROBOT_ANGLE_DETECTION || debug) {
            bgsWindowRobot.setImage(diff);
        }
    }

    private void determineAngleSearchArea(boolean debug) {

        if (center != null && radius != null) {
            prev = center.clone();
            prevRadius = radius[0];
            Rect rect = rectSearch(frame, (int) prev.x, (int) prev.y, (int) (radius[0] * 3));
            r = frame.submat(rect);
            diff = bgsAngle(r);
            r.release();
        } else {
            diff = bgsAngle(frame);
        }

        if (Simulation.DEBUG_CV_ROBOT_ANGLE_DETECTION || debug) {
            bgsWindowAngle.setImage(diff);
        }
    }

    private void determineObjectSearchArea(boolean debug) {

        if (object != null && radius != null) {
            prev = object.clone();
            prevRadius = radius[0];
            Rect rect = rectSearch(frame, (int) prev.x, (int) prev.y, (int) (radius[0] * 3));
            r = frame.submat(rect);
            diff = bgsObject(r);//bgsAngle(r);
            r.release();
        } else {
            diff = bgsObject(frame);
            ax = 0;
            ay = 0;
        }

        if (Simulation.DEBUG_CV_OBJECT || debug) {
            bgsWindowObject.setImage(diff);
        }
    }

    public MatOfPoint backgroundRect(Mat fullimage) {
        Imgproc.cvtColor(fullimage, hsv, Imgproc.COLOR_BGR2HSV);

        Imgproc.GaussianBlur(hsv, hsv, new Size(9, 9), 2, 2);

        //Core.inRange(hsv, new Scalar(70, 10, 20), new Scalar(200, 150, 55), tmp_mask1);
        //Core.inRange(hsv, new Scalar(0, 10, 20), new Scalar(40, 150, 55), tmp_mask2);

        Core.inRange(hsv, backgroundScalar1,backgroundScalar2, tmp_mask1);
        Core.inRange(hsv, backgroundScalar1, backgroundScalar2, tmp_mask2);

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

        return contours.get(biggestContourIndex);
    }

    private Rect rectSearch(Mat img, int x, int y, int searchSpace) {
        Rect rect = null;
        int maxX = img.cols();
        int maxY = img.rows();
        ax = 0;
        ay = 0;
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

        Core.inRange(cc, robotBgs1_1, robotBgs1_2, mask);
        //Core.inRange(cc, robotBgs2_1, robotBgs2_2, tmp_mask2);

		// values for jordys place
		//Core.inRange(cc, new Scalar(0, 90, 130), new Scalar(10, 200, 255), tmp_mask1);
        //Core.inRange(cc, new Scalar(170, 90, 130), new Scalar(180, 200, 255), tmp_mask2);

        //Core.add(tmp_mask1, tmp_mask2, mask);
        kernel = Mat.ones(5, 5, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        cc.release();
        //tmp_mask1.release();
       // tmp_mask2.release();
        kernel.release();

        return mask;
    }

    private Mat bgsAngle(Mat m1) {
        //Format I encountered is 360 (degrees), 100 (percent), 100 (percent)
        //HSV in OpenCV is 180, 255, 255 format.
        mask.release();
        m1.copyTo(cc);
        Imgproc.cvtColor(cc, cc, Imgproc.COLOR_BGR2HSV);

		// values for jordys place
        //Core.inRange(cc, new Scalar(120, 35, 120), new Scalar(170, 85, 180), mask);
		
		Core.inRange(cc, angleScalar1, angleScalar2, mask);

        kernel = Mat.ones(7, 7, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        cc.release();
        kernel.release();

        return mask;
    }

    private Mat bgsObject(Mat m1) {
        //Format I encountered is 360 (degrees), 100 (percent), 100 (percent)
        //HSV in OpenCV is 180, 255, 255 format.
        mask.release();
        m1.copyTo(cc);
        Imgproc.cvtColor(cc, cc, Imgproc.COLOR_BGR2HSV);

        // values for jordys place
        //Core.inRange(cc, new Scalar(120, 35, 120), new Scalar(170, 85, 180), mask);

        Core.inRange(cc, objectBgs1_1, objectBgs1_2, mask);

        kernel = Mat.ones(7, 7, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_DILATE, kernel);

        cc.release();
        kernel.release();

        return mask;
    }

    private MatOfPoint contours(Mat img, boolean counturWindow, int contourDialtation) {
        Imgproc.cvtColor(img, hsv, Imgproc.COLOR_BGR2HSV);

        Imgproc.GaussianBlur(hsv, hsv, new Size(15, 15), 2, 2);

        Core.inRange(hsv, contourScalar1, contourScalar2, mask);

		// values for jordys place
        //Core.inRange(hsv, new Scalar(10, 20, 160), new Scalar(30, 165, 255), mask);

		// old second range
        //Core.inRange(hsv, new Scalar(165, 30, 50), new Scalar(180, 190, 230), tmp_mask2);
        //Core.add(tmp_mask1, tmp_mask2, mask);
        kernel = Mat.ones(7, 7, CvType.CV_8UC1);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        //Need to make sure that robot is removed.
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


        while (contours.size() != 1) {
            for (int i = 0; i < contours.size(); i++) {
                MatOfPoint contour = contours.get(i);
                if (Imgproc.contourArea(contour) < 1000) {
                    Imgproc.fillConvexPoly(mask, contour, new Scalar(0));
                }
            }
            contours.clear();
            hierarchy.release();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        }

        //iterations = radius
        if (contourDialtation>0)Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), contourDialtation);
        contours.clear();
        hierarchy.release();
        if (counturWindow) ImgWindow.newWindow(mask.clone());
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
        currentContours = contours(bg,true,17);
        bg.release();
    }

    public void findAnglePosition(boolean debug) {
        if (found) {
            determineAngleSearchArea(debug);
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

    public void findObjectPostion(boolean debug) {
            determineObjectSearchArea(debug);
            boolean foundPoint = false;

            contours.clear();

            Imgproc.findContours(diff, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            int biggestContour = 0;

            if (!contours.isEmpty() && !hierarchy.empty()) {
                for (int j = 0; j < contours.size(); j++) {
                    if (contours.get(j).size().area() >= contours.get(biggestContour).size().area()) {
                        foundPoint = true;
                        biggestContour = j;
                    }
                }
            }

            if (!foundPoint) {
                object = null;
                ax = 0;
                ay = 0;
            } else if (contours.get(biggestContour).toArray().length >= 5){
                RotatedRect r = Imgproc.fitEllipse(new MatOfPoint2f(contours.get(biggestContour).toArray()));
                r.center.x += ax;
                r.center.y += ay;
                if (Simulation.DEBUG_CV_OBJECT) {
                    Imgproc.ellipse(frame, r, new Scalar(0, 255, 255), 3);
                }
                object = new Point(r.center.x, r.center.y);
            } else {
                Point p = new Point(contours.get(biggestContour).toArray()[0].x, contours.get(biggestContour).toArray()[0].y);
                p.x += ax;
                p.y += ay;
                if (Simulation.DEBUG_CV_OBJECT) {
                    Imgproc.circle(frame, p, 3, new Scalar(0, 255, 255));
                }
                object = new Point(p.x, p.y);
            }

            hierarchy.release();
            diff.release();
    }

    public void findRobotPosition(boolean debug) {
        determineRobotSearchArea(debug);

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

    public Point getObject() {
        return object;
    }

    public Point getPrev() {
        return prev;
    }

    public MatOfPoint getCurrentContours() {
        readNextFrame();
        currentContoursWithoutDialation = contours(frame.clone(),true, 0);
        currentContours = contours(frame,true, 21);


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


    /**
     * the position of the tip of the triangle on top of the epuc
     * @return
     */
    public Point getAnglePoint() {
        return angle;
    }


    public boolean isRobotDetected() {

            Point currentPosition = center;
            boolean detected;
            if (currentPosition == null) {
                System.out.println("Robot not found in initial frame, program will crash.");
                detected = false;
            } else {
                detected = true;
            }
            return detected;
        }

    public RoboPos getRoboPosFromCurrentPositionAndSetAngle(boolean debug) {

            Point currentPosition = center;
            int robotX = (int) (currentPosition.x);
            int robotY = (int) (currentPosition.y);
            int robotR = (int) (getRadius() / 2);

            RoboPos rp = new RoboPos(robotX, robotY, robotR);

            // determine the current position of the angle and calculate rotation
            findAnglePosition(debug);
            rp.setDirection(angle);

            return rp;

    }

    public Mat getSubFrame() {
        Mat currentFrame = getFrame();

        currentFrame = currentFrame.submat(Imgproc.boundingRect(backgroundRect(currentFrame)));

        return currentFrame;
    }

    public void setByPass(boolean byPassCamera, File[] files) {
        BufferedImage[] imaages = new BufferedImage[files.length];
        Mat[] images = new Mat[files.length];
        for (int i = 0; i < files.length; i++) {
            try {
                imaages[i] = ImageIO.read(files[i]);
                images[i] = ImgWindow.bufferedImmageToMat(imaages[i]);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        this.byPassImages = images;
        this.byPassCamera = byPassCamera;

    }

    public MatOfPoint getContourWithoutDialation() {
        return currentContoursWithoutDialation;
    }
}

