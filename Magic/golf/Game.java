package Magic.golf;


import Game.Actors.Bots.*;
import Game.Actors.HumanPlayer;
import Game.Actors.Player;
import Game.Model.*;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

/**
 * Created by tony on 16/03/2016.
 */
public class Game {
    public static Course course;
    public static JFrame frame;
    private static JPanel RightSidebar;
    private static JPanel infoSidebar;
    private static JPanel LeftSidebar;
    private static JToggleButton select;
    private static JToggleButton obectFormSelect;
    private static boolean editorVisible;
    private static boolean infoVisible;
    private static boolean variablesVisible;
    private static JLabel scoreInfo;


    private static PhysicsEngine physics;
    public static AIPlayer AI= new AngryBot("Player 2",64);
    public static AIPlayer AI2= new AngryBot("Player 3",64);
    public static DrawPanel dp;

    private static ArrayList<Player> pp;

    private static boolean selectNextPlayer;
    private static int currentPlayer=0;

    private static Thread gameThread;
    private static boolean courseLoading;
    private static Course course1,course2, course3;
    private static Course previewMiniCourse;

    private static boolean pause = false;

    public static PhysicsEngine getAlternativBoardForTest(){
        return physics.getAlternativBoardForTest();
    }

    public Game(){
        //JOptionPane.showMessageDialog(frame, "Wait for profiler");
        prepareCourses();
        prepareView();
        preparePlayers();

        loadCourse(course);
        gameThread = createGameThread();
        gameThread.run();



    }

    public static Dimension getFrameDimension() {
        Dimension d = new Dimension(Config.getWidth() + (variablesVisible ? 1 : 0) * Config.sidebarwidth + (editorVisible ? 1 : 0) * Config.sidebarwidth, Config.getHeight() + Config.OFFSET_Y_GAME);
        return d;
    }

    private void preparePlayers() {
        pp = new ArrayList<Player>(2);
        //Player p = new HumanPlayer("Player 1");
        Player p2  =  new AngryBot("Player 3",32) {
        };

        //pp.add(p);
        pp.add(p2);
        //pp.add(AI2);
    }

    private void prepareView() {
        if (course == null) course = createStandartCourse(0);


        frame = new JFrame();
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        dp = new DrawPanel();

        frame.setSize(course.getWidth(), course.getHeight() + Config.OFFSET_Y_GAME);
        frame.add(dp, BorderLayout.CENTER);
        infoSidebar = new JPanel();
        addMenues(frame);
        frame.setVisible(true);
        addKeyListenerToFrame(frame);

    }

    private static void drawScoreInfo(){
        if(pp!=null) {
            StringBuilder scores = new StringBuilder();
            for (int i = 0; i < pp.size(); i++) {
                Player p = pp.get(i);
                if(pp.get(currentPlayer).getName()==p.getName()) {
                    scores.append("CURRENTPLAYER: " + System.lineSeparator()).append(p.getName()).append("  Total Strokes: ").append(p.getTotalStrokes()).append(" Current Strokes: ").append(p.getCurrentStrokes()).append(System.lineSeparator()).append("        ");

                }
                else{
                    scores.append(p.getName()).append("  Total Strokes: ").append(p.getTotalStrokes()).append(" Current Strokes: ").append(p.getCurrentStrokes()).append(System.lineSeparator()).append("        ");

                }

            }
            //JOptionPane.showMessageDialog(null, scores.toString(), "Scores", JOptionPane.INFORMATION_MESSAGE);
            if(scoreInfo==null){
                scoreInfo = new JLabel(scores.toString());
                infoSidebar.add(scoreInfo);
            }

            else {
                if(scoreInfo.getText()!= scores.toString()){
                    scoreInfo.setText(scores.toString());
                    infoSidebar.repaint();
                }

            }


        }
    }

    private void prepareCourses() {
        course1 = Course.loadCourse2_5d(Config.CourseLocation + "GolfDeluxe1.gol");
        //course2 = Course.loadCourse2_5d(Config.CourseLocation + "GolfDeluxe2.gol");
        //course3 = Course.loadCourse2_5d(Config.CourseLocation +"GolfDeluxe3.gol");
        if (course1 == null) {
            course1 = createStandartCourse(1);
        }
        if (course2 == null) {
            //course2 = createStandartCourse(2);
        }
        if (course3 == null) {
            //course3 = createStandartCourse(3);
        }

        course = course1;
    }

    private Course createStandartCourse(int standartCourse) {
        Course c;
        switch (standartCourse){
            case 1:
                Course course1;
                course1 = new Course("GolfDeluxe1", Config.getWidth(), Config.getHeight(), Config.getDepth(), Type.Grass, 1);
                course1.addFrustrum(0,0,0,Config.getWidth(),Config.getHeight(),10,0,0,0,0,Type.Grass);
                course1.addFrustrum(200,0,10,160,440,20,2,0,0,0,Type.OBJECT);
                course1.addFrustrum(100,600,10,160,100,20,2,0,0,0,Type.OBJECT);
                course1.addFrustrum(420,220,10,160,340,20,1,-3,1,-1,Type.OBJECT);
                course1.addFrustrum(620,320,10,330,240,40,2,-1,1,-4,Type.OBJECT);
                course1.addFrustrum(620,120,10,330,140,60,15,-4,15,-4,Type.OBJECT);
                //course.addFrustrum(650,440,0,110,140,200,15,0,0,-10,Type.OBJECT);
                //course.addFrustrum(520,120,0,160,140,20,2,0,0,0,Type.OBJECT);
                //course.addFrustrum(520,120,0,160,140,20,2,0,0,0,Type.OBJECT);
                //course.addFrustrum(520,120,0,160,140,20,2,0,0,0,Type.OBJECT);
                course1.setTile(800, 650, 1, Type.Hole);
                course1.setTile(100, 100, 9, Type.Start);
                // course.addRectangle(600, 400, 50, 100, 0, Type.OBJECT);
                // course.addRectangle(400, 400, 50, 100, 1, Type.OBJECT);
                // course.addCuboid(400, 400, 50, 100, 30, 20, Type.OBJECT);
                //course.addPyramid(50, 50, 0, 100, 30, 20, Type.OBJECT);
                // course.addHill(152, 152, 150, 1.5, 0, 20, Type.OBJECT);
                // course.addPyramid(400, 400, 0, 200, 200, 100, Type.OBJECT);
                course1.finalise();
                course1.saveCourse();
                c = course1;
                break;
            case 2:
                Course course2;
                course2 = new Course("GolfDeluxe2", Config.getWidth(), Config.getHeight(), Config.getDepth(), Type.Grass, 1);
                course2.addFrustrum(0,0,0,Config.getWidth(),Config.getHeight(),10,0,0,0,0,Type.Grass);
                course2.addFrustrum(200,0,10,160,440,20,2,0,0,0,Type.OBJECT);
                //course.addFrustrum(420,220,10,160,340,20,1,-3,1,-1,Type.OBJECT);
                //course.addFrustrum(620,320,10,330,240,40,2,-1,1,-4,Type.OBJECT);
                course2.addFrustrum(620,120,10,330,140,60,15,-4,15,-4,Type.OBJECT);

                course2.setTile(850, 650, 1, Type.Hole);
                course2.setTile(100, 100, 9, Type.Start);

                course2.finalise();


                course2.saveCourse();
                c = course2;
                break;
            case 3:
                Course course3;
                course3 = new Course("GolfDeluxe3", Config.getWidth(), Config.getHeight(), Config.getDepth(), Type.Grass, 1);
                course3.addFrustrum(0,0,0,Config.getWidth(),Config.getHeight(),10,0,0,0,0,Type.Grass);
                //course.addFrustrum(0,0,0,Config.getWidth(),Config.getHeight(),10,0,0,0,0,Type.Grass);
                //course.addFrustrum(200,0,10,160,440,20,2,0,0,0,Type.OBJECT);
                //course.addFrustrum(420,220,10,160,340,20,1,-3,1,-1,Type.OBJECT);
                //course.addFrustrum(620,320,10,330,240,40,2,-1,1,-4,Type.OBJECT);
                //course.addFrustrum(620,120,10,330,140,60,15,-4,15,-4,Type.OBJECT);
                //course.addFrustrum(650,440,0,110,140,200,15,0,0,-10,Type.OBJECT);
                //course.addFrustrum(520,120,0,160,140,20,2,0,0,0,Type.OBJECT);
                //course.addFrustrum(520,120,0,160,140,20,2,0,0,0,Type.OBJECT);
                //course.addFrustrum(520,120,0,160,140,20,2,0,0,0,Type.OBJECT);

                course3.setTile(750, 600, 1, Type.Hole);
                course3.setTile(100, 100, 9, Type.Start);
                //course.addRectangle(600, 400, 50, 100, 0, Type.OBJECT);
                //course3.addRectangle(400, 400, 50, 100, 100, Type.OBJECT);
                // course.addCuboid(400, 400, 50, 100, 30, 20, Type.OBJECT);
                //course.addPyramid(50, 50, 0, 100, 30, 20, Type.OBJECT);
                // course.addHill(152, 152, 150, 1.5, 0, 20, Type.OBJECT);
                // course.addPyramid(400, 400, 0, 200, 200, 100, Type.OBJECT);
                course3.finalise();
                course3.saveCourse();
                c = course3;
                break;
            default:
                Course courseD;
                courseD = new Course("GolfDeluxeStandart", Config.getWidth(), Config.getHeight(), Config.getDepth(), Type.Grass, 1);
                courseD.addFrustrum(0,0,0,Config.getWidth(),Config.getHeight(),10,0,0,0,0, Type.Grass);
                courseD.setTile(800, 600, 1, Type.Hole);
                courseD.setTile(100, 100, 4, Type.Start);
                courseD.finalise();
                courseD.saveCourse();
                c = courseD;
                break;

        }
        return c;
    }

    private void addKeyListenerToFrame(JFrame frame) {
        frame.addKeyListener(new KeyListener() {
            @Override
            public void keyTyped(KeyEvent e) {

            }

            @Override
            public void keyPressed(KeyEvent e) {
                if(e.getKeyChar()=='q'){
                    for (Player p: pp){
                        //p.getBall().printBallInfo();
                        System.out.println();
                    }
                }
                if(e.getKeyChar()=='w'){
                    physics.processPhysics(Config.STEPSIZE,Config.NOISEPERCENTAGE,Config.WINDPERCENTAGEPERSECOND);
                    dp.repaint();
                }
                if(e.getKeyChar()=='p'){
                    pause = !pause;
                }
            }

            @Override
            public void keyReleased(KeyEvent e) {

            }
        });
    }

    private static Thread createGameThread() {
        Thread t =  new Thread(){
            long lastTime = System.currentTimeMillis();

            public void run(){
                while (true) {

                    long currentTime = System.currentTimeMillis();
                    long elapsedTime = currentTime - lastTime;
                    //System.out.println("time:" +currentTime + "elapsed:" +elapsedTime);
                    lastTime = currentTime;
                    if (courseLoading) continue;
                    // || pp.get(currentPlayer).getBall().isMoving())&&!pp.get(currentPlayer).getBall().inHole
                    if ((physics.atLeastOneBallMoving())) {
                        selectNextPlayer = true;
                        ;
                        if (!pause) physics.processPhysics(Config.STEPSIZE, Config.NOISEPERCENTAGE,Config.WINDPERCENTAGEPERSECOND); //
                            selectNextPlayer = true;
                        try {
                            dp.repaint();
                            Thread.sleep(1);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                    } else {
                        dp.resetAIPreview();
                        drawScoreInfo();
                        if (selectNextPlayer) {
                            if (!IsGameStillOn()) {
                                selectNextPlayer = false;
                                //JOptionPane.showMessageDialog(null, "Round Finished", "End Round", JOptionPane.INFORMATION_MESSAGE);
                                //showScoreOnScreen();
                            }
                            selectNextPlayer = false;

                            do {
                                currentPlayer = (currentPlayer + 1) % (pp.size());
                            }
                            while (!pp.get(currentPlayer).getBall().inPlay);
                            ArrayList<Ball> balls = new ArrayList<>(8);


                            pp.get(currentPlayer).getBall().setPreGame(false);
                            dp.setCurrentPlayer(pp.get(currentPlayer));
                            pp.get(currentPlayer).nextMove(physics);

                            dp.repaint();

                        }



                        }


                }
            }
        };
        return t;
    }

    public static void main(String[] args) {
        Game g = new Game();


    }

    private static boolean IsGameStillOn() {

        for (int i = 0; i < pp.size(); i++) {
            if (pp.get(i).isInPlay()) return true;
        }
        return false;
    }

    private static void addMenues(JFrame frame) {
        JMenuBar JMB = new JMenuBar();
        frame.setJMenuBar(JMB);
        JMenu setting = new JMenu("Players");
        JMB.add(setting);


        JMenuItem addPlayer = new JMenuItem("Add Player");
        JMenuItem resetCourse = new JMenuItem("Reset Course");
        JMenuItem removePlayer = new JMenuItem("Remove Player");

        JMenuItem resetStrokes = new JMenuItem("Reset Strokes");
        JMenuItem selectCourse = new JMenuItem("Select Course");
        setting.add(addPlayer);
        setting.add(removePlayer);
        setting.add(resetCourse);
        setting.add(selectCourse);
        setting.add(resetStrokes);
        addListenerToAddPlayer(addPlayer);
        addListenerToRemovePlayer(removePlayer);
        addListenerToResetCourse(resetCourse);
        addListenerToSelectCourse(selectCourse);
        addListenerToResetStrokes(resetStrokes);
        JMenuItem showScore = new JMenuItem("Show Scores");
        addListenerToShowScore(showScore);
        setting.add(showScore);

        JMenuItem showEditorButton = new JMenuItem("Show Editor");
        showEditorButton.setVisible(true);
        //showEditorButton.setMinimumSize(setting.getSize());
        //showEditorButton.setPreferredSize(setting.getSize());
        //showEditorButton.setMaximumSize(setting.getSize());
        showEditorButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                showEditor();
            }
        });


        JMenu jm3 = new JMenu("Courses");
        JMenuItem courseSelect1 = new JMenuItem("Course1");
        courseSelect1.addActionListener(e -> loadCourse(course1));
        JMenuItem courseSelect2 = new JMenuItem("Course2");
        courseSelect2.addActionListener(e -> loadCourse(course2));
        JMenuItem courseSelect3 = new JMenuItem("Course3");
        courseSelect3.addActionListener(e -> loadCourse(course3));

        jm3.add(resetCourse);
        jm3.add(selectCourse);
        jm3.add(resetStrokes);
        jm3.add(courseSelect1);
        jm3.add(courseSelect2);
        jm3.add(courseSelect3);
        JMB.add(jm3);
        JMB.add(showEditorButton);
        showInfo();

        JMenuItem showVariablesButton = new JMenuItem("Show Variables");
        showVariablesButton.setVisible(true);
        showVariablesButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                showVariables();
            }


        });
        JMB.add(showVariablesButton);
    }

    private static void showVariables() {
        variablesVisible = !variablesVisible;
        frame.setSize(Config.getWidth() + (variablesVisible ? 1 : 0) * Config.sidebarwidth + (editorVisible ? 1 : 0) * Config.sidebarwidth, Config.getHeight() + Config.OFFSET_Y_GAME);
        if (variablesVisible) {



            LeftSidebar = new JPanel();
            Dimension d = new Dimension(Config.sidebarwidth, Config.getHeight() + Config.OFFSET_Y_GAME);
            LeftSidebar.setMinimumSize(d);
            LeftSidebar.setPreferredSize(d);
            LeftSidebar.setMaximumSize(d);
            //RightSidebar.add(SidePann)
            LeftSidebar.setLayout(new BoxLayout(LeftSidebar, BoxLayout.PAGE_AXIS));

            JPanel label = new miniDraw(new Dimension(Config.sidebarwidth, Config.getHeight() + Config.OFFSET_Y_GAME - 400));
            LeftSidebar.add(label);

            JLabel BallRadius = new JLabel("BallRadius");
            JTextField BallRadiusT = new JTextField("" + Config.ballRadius);



            JLabel collitionSurfacePointRatio = new JLabel("collitionSurfacePointRatio");
            JTextField collitionSurfacePointRatioT = new JTextField("" + Config.collitionSurfacePointRatio);

            JLabel hoverSurfacePointRatio = new JLabel("hoverSurfacePointRatio");
            JTextField hoverSurfacePointRatioT = new JTextField("" + Config.hoverSurfacePointRatio);

            JLabel AirFriction = new JLabel("AirDrag");
            JTextField AirFrictionT = new JTextField("" + Config.AIR_FRICTION);

            JLabel GrassFriction = new JLabel("GrassFriction");
            JTextField GrassFrictionT = new JTextField("" + Config.GRASS_FRICTION);


            JLabel GrassDampness = new JLabel("GrassDampness");
            JTextField GrassDampnessT = new JTextField("" + Config.GRASS_DAMPNESS);


            JLabel ObjectFriction = new JLabel("ObjectFriction");
            JTextField ObjectFrictionT = new JTextField("" + Config.OBJECT_FRICTION);


            JLabel ObjectDampness = new JLabel("ObjectDampness");
            JTextField ObjectDampnessT = new JTextField("" + Config.OBJECT_DAMPNESS);

            JLabel Gravity = new JLabel("Gravity");
            JTextField GravityT = new JTextField("" + Config.GRAVITY_FORCE);


            JButton select = new JButton("Physic apply");
            select.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {


                        Config.collitionSurfacePointRatio = Double.parseDouble(collitionSurfacePointRatioT.getText());
                        Config.hoverSurfacePointRatio = Double.parseDouble(hoverSurfacePointRatioT.getText());
                        Config.AIR_FRICTION = Double.parseDouble(AirFrictionT.getText());
                        Config.GRASS_FRICTION = Double.parseDouble(GrassFrictionT.getText());

                        Config.GRASS_DAMPNESS = Double.parseDouble(GrassDampnessT.getText());

                        Config.OBJECT_FRICTION = Double.parseDouble(ObjectFrictionT.getText());

                        Config.OBJECT_DAMPNESS = Double.parseDouble(ObjectDampnessT.getText());

                        Config.GRAVITY_FORCE = Double.parseDouble(GravityT.getText());

                        Type.reset();
                    } catch (Exception ex) {
                        ex.printStackTrace();
                    }
                }
            });

            JButton recalcBall = new JButton("recalcBall");
            recalcBall.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    try {

                        Config.ballRadius = Double.parseDouble(BallRadiusT.getText());
                        Config.collitionSurfacePointRatio = Double.parseDouble(collitionSurfacePointRatioT.getText());
                        Config.hoverSurfacePointRatio = Double.parseDouble(hoverSurfacePointRatioT.getText());

                        for (Player p : pp) {

                            p.resetBall();

                        }
                        physics.init(pp, course);
                        dp.precalcBallImage();
                        loadCourse(course);
                    } catch (Exception ex) {
                        ex.printStackTrace();
                    }
                    dp.repaint();
                }
            });

            LeftSidebar.add(AirFriction);
            LeftSidebar.add(AirFrictionT);

            LeftSidebar.add(BallRadius);
            LeftSidebar.add(BallRadiusT);

            LeftSidebar.add(collitionSurfacePointRatio);
            LeftSidebar.add(collitionSurfacePointRatioT);

            LeftSidebar.add(hoverSurfacePointRatio);
            LeftSidebar.add(hoverSurfacePointRatioT);

            LeftSidebar.add(GrassFriction);
            LeftSidebar.add(GrassFrictionT);

            LeftSidebar.add(GrassDampness);
            LeftSidebar.add(GrassDampnessT);

            LeftSidebar.add(ObjectFriction);
            LeftSidebar.add(ObjectFrictionT);


            LeftSidebar.add(ObjectDampness);
            LeftSidebar.add(ObjectDampnessT);

            LeftSidebar.add(Gravity);
            LeftSidebar.add(GravityT);


            LeftSidebar.add(select);
            LeftSidebar.add(recalcBall);


        } else {

            if (LeftSidebar != null) LeftSidebar.setVisible(false);


        }
        //


        frame.add(LeftSidebar, BorderLayout.WEST);
    }

    private static void showInfo() {

        frame.setSize(getFrameDimension());

            Dimension d = new Dimension(Config.sidebarwidth, 50);
            infoSidebar.setMinimumSize(d);
            infoSidebar.setPreferredSize(d);
            infoSidebar.setMaximumSize(d);
            infoSidebar.setLayout(new BoxLayout(infoSidebar, BoxLayout.LINE_AXIS));

        frame.add(infoSidebar, BorderLayout.SOUTH);
        dp.repaint();
        frame.repaint();
    }

    private static void showEditor() {
        editorVisible = !editorVisible;

        frame.setSize(getFrameDimension());
        if (editorVisible) {


            RightSidebar = new JPanel();
            Dimension d = new Dimension(Config.sidebarwidth, Config.getHeight() + Config.OFFSET_Y_GAME);
            RightSidebar.setMinimumSize(d);
            RightSidebar.setPreferredSize(d);
            RightSidebar.setMaximumSize(d);
            //RightSidebar.add(SidePann)
            RightSidebar.setLayout(new BoxLayout(RightSidebar, BoxLayout.PAGE_AXIS));

            JPanel label = new miniDraw(new Dimension(Config.sidebarwidth, Config.getHeight() + Config.OFFSET_Y_GAME - 400));
            RightSidebar.add(label);


            JLabel widthL = new JLabel("Width");
            JTextField widhtT = new JTextField("200");
            JLabel heightL = new JLabel("Height");
            JTextField heightT = new JTextField("50");
            JLabel depthL = new JLabel("depth");
            JTextField depthT = new JTextField("50");
            JLabel zL = new JLabel("z");
            JTextField zT = new JTextField("10");
            JLabel radiusL = new JLabel("Radius");
            JTextField deltaRadiusT = new JTextField("-1");
            JLabel deltaRadiusL = new JLabel("delta Radius per Layer");
            JTextField radiusT = new JTextField("30");
            JLabel deltaXLL = new JLabel("deltaX left per Layer");
            JTextField deltaXL_T = new JTextField("1");
            JLabel deltaXRL = new JLabel("deltaX right per Layer");
            JTextField deltaXR_T = new JTextField("-1");
            JLabel deltaYTL = new JLabel("deltaY top per Layer");
            JTextField deltaYT_T = new JTextField("1");
            JLabel deltaYBL = new JLabel("deltaY bottom per Layer");
            JTextField deltaYB_T = new JTextField("-1");

            JComboBox<String> objectFormBox = new JComboBox<>(Arrays.toString(ObjectForm.values()).replaceAll("^.|.$", "").split(", "));
            //obectFormSelect = new JToggleButton("select");
            JComboBox<String> typeBox = new JComboBox<>(Arrays.toString(Type.values()).replaceAll("^.|.$", "").split(", "));
            select = new JToggleButton("select");
            addSelectActionListener(select, widhtT, heightT, depthT, zT,radiusT, deltaRadiusT, deltaXL_T, deltaXR_T, deltaYT_T, deltaYB_T, typeBox,objectFormBox);
            //JToggleButton select, JTextField widhtT, JTextField heightT, JTextField depthT, JTextField zT, JTextField radiusT, JTextField deltaRT,JTextField deltaXLL, JTextField deltaXRL, JTextField deltaYT_t, JTextField deltaYB_t, JComboBox<String> typeBox, JComboBox<String> obectFormBox)
            JButton finalizeCourse = new JButton("FinalizeCourse");
            finalizeCourse.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    course.finalise();
                    dp.repaint();

                }

            });
            JButton saveCourse = new JButton("SaveCourse");
            saveCourse.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    String s = (String) JOptionPane.showInputDialog(
                            frame,
                            "Course Name",
                            "Customized Dialog",
                            JOptionPane.PLAIN_MESSAGE,
                            null,
                            null,
                            "Course" + new Random().nextInt(100));
                    course.setName(s);
                    course.saveCourse();
                    dp.repaint();

                }

            });
            RightSidebar.add(widthL);
            RightSidebar.add(widhtT);
            RightSidebar.add(heightL);
            RightSidebar.add(heightT);
            RightSidebar.add(depthL);
            RightSidebar.add(depthT);
            RightSidebar.add(zL);
            RightSidebar.add(zT);
            RightSidebar.add(radiusL);
            RightSidebar.add(radiusT);
            RightSidebar.add(deltaRadiusL);
            RightSidebar.add(deltaRadiusT);
            RightSidebar.add(deltaXLL);
            RightSidebar.add(deltaXL_T);
            RightSidebar.add(deltaXRL);
            RightSidebar.add(deltaXR_T);
            RightSidebar.add(deltaYTL);
            RightSidebar.add(deltaYT_T);
            RightSidebar.add(deltaYBL);
            RightSidebar.add(deltaYB_T);

            RightSidebar.add(objectFormBox);
            //RightSidebar.add(obectFormSelect);
            RightSidebar.add(typeBox);
            RightSidebar.add(select);

            RightSidebar.add(finalizeCourse);
            RightSidebar.add(saveCourse);


        } else {


            if (RightSidebar != null) RightSidebar.setVisible(false);

        }
        //


        frame.add(RightSidebar, BorderLayout.EAST);
    }

    private static void addSelectActionListener(JToggleButton select, JTextField widhtT, JTextField heightT, JTextField depthT, JTextField zT, JTextField radiusT, JTextField deltaRT,JTextField deltaXLL, JTextField deltaXRL, JTextField deltaYT_t, JTextField deltaYB_t, JComboBox<String> typeBox, JComboBox<String> obectFormBox) {
        select.addActionListener(new ActionListener() {

            @Override
            public void actionPerformed(ActionEvent e) {
                try {
                    if (select.isSelected()) {
                        int width = Integer.parseInt(widhtT.getText());
                        int height = Integer.parseInt(heightT.getText());
                        int depth = Integer.parseInt(depthT.getText());
                        int z = Integer.parseInt(zT.getText());
                        double deltaXL = Double.parseDouble(deltaXLL.getText());
                        double deltaXR = Double.parseDouble(deltaXRL.getText());
                        double deltaYT = Double.parseDouble(deltaYT_t.getText());
                        double deltaYB = Double.parseDouble(deltaYB_t.getText());
                        double deltaR = Double.parseDouble(deltaRT.getText());
                        int radius = Integer.parseInt(radiusT.getText());

                        Type t = Type.valueOf(typeBox.getSelectedItem().toString());
                        ObjectForm of = ObjectForm.valueOf(obectFormBox.getSelectedItem().toString());

                        switch (of){
                            case Cuboid:
                                previewMiniCourse = new Course("preview", width, height, course.getDepth(), Type.Empty, 0);
                                previewMiniCourse.addFrustrum(0, 0, z, width, height, depth, 0, 0, 0, 0, t);
                                break;
                            case Fructum:
                                previewMiniCourse = new Course("preview", width, height, course.getDepth(), Type.Empty, 0);
                                previewMiniCourse.addFrustrum(0, 0, z, width, height, depth, deltaXL, deltaXR, deltaYT, deltaYB, t);
                               // previewMiniCourse.addFrustrum(0,0,10,160,440,20,2,0,0,0,Type.OBJECT);
                                break;
                            case Cylinder:
                                previewMiniCourse = new Course("preview", radius*2, radius*2, course.getDepth(), Type.Empty, 0);
                                previewMiniCourse.addHill(radius, radius, z, depth, radius,0, t);
                                break;
                            case Hill:
                                previewMiniCourse = new Course("preview", radius*2, radius*2, course.getDepth(), Type.Empty, 0);
                                previewMiniCourse.addHill(radius, radius, z, depth, radius,deltaR, t);
                                break;
                        }

                        previewMiniCourse.calculateHeightMapSafe();
                        previewMiniCourse.calculateSurfaceNormalsSafe();
                        previewMiniCourse.calculateShadingMap();
                        previewMiniCourse.setBufferedImage(dp.createImage(previewMiniCourse));
                        dp.setPreviewObject(previewMiniCourse.getManagedBufferedImage());
                    } else {
                        previewMiniCourse = null;
                        dp.setPreviewObject(null);
                    }
                } catch (Exception ex) {
                    ex.printStackTrace();
                }

            }
        });
    }

    private static void addListenerToResetCourse(JMenuItem resetCourse) {
        resetCourse.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                loadCourse(course);


            }
        });
    }

    private static void addListenerToShowScore(JMenuItem showScore) {


        showScore.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                showScoreOnScreen();

            }
        });

    }

    private static void showScoreOnScreen() {
        StringBuilder scores = new StringBuilder();
        for (int i = 0; i < pp.size(); i++) {
            Player p = pp.get(i);
            scores.append("Player: ").append(p.getName()).append("  Total Strokes: ").append(p.getTotalStrokes()).append(" Current Strokes: ").append(p.getCurrentStrokes()).append(System.lineSeparator());
        }
        JOptionPane.showMessageDialog(null, scores.toString(), "Scores", JOptionPane.INFORMATION_MESSAGE);


    }

    private static void addListenerToResetStrokes(JMenuItem resetStrokes) {
        resetStrokes.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                for (int i = 0; i < pp.size(); i++) {
                    pp.get(i).resetStrokes();

                }
            }
        });

    }

    private static void addListenerToSelectCourse(JMenuItem selectCourse) {
        selectCourse.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                String path = "";

               JFileChooser chooser = new JFileChooser();
               String p = System.getProperty("user.dir") + File.separator + Config.CourseLocation;
               File projectDir = new File(p);
               chooser.setCurrentDirectory(projectDir);
               int returnVal = chooser.showOpenDialog(frame);
               if (returnVal == JFileChooser.APPROVE_OPTION) {
                   path = chooser.getSelectedFile().getAbsolutePath();

                    loadCourse(path);

                }
            }
        });

    }

    private static void loadCourse(String path) {
        Course course = Course.loadCourse2_5d(path);

        loadCourse(course);
    }

    private static void loadCourse(Course coursel) {
        course = coursel;
        courseLoading = true;
        ArrayList<Ball> balls = new ArrayList<>(2);

        selectNextPlayer = false;
        for (int i = 0; i < pp.size(); i++) {
            Player p = pp.get(i);

            p.resetBall();
            p.resetCurrentStrokes();
            p.setInPlay(true);
            p.getBall().setPreGame(true);
            balls.add(p.getBall());
            Tile t = course.getStartTile();
            p.setBallPositionToCoordinateAndSetSpeedToZero(t.x, t.y, t.z + p.getBall().getRadius());
        }
        if (pp.size() != 0) pp.get(0).getBall().setPreGame(false);

        dp.setCourse(course);
        AI.setCourse(course);

        currentPlayer = 0;
        dp.setCurrentPlayer(pp.get(currentPlayer));
        dp.setPlayers(pp);

        if (physics == null) physics = new PhysicsEngine();
        physics.init(pp, course, new Wind(Config.MINWIND, Config.MAXWIND, 0, 2 * Math.PI));

        dp.repaint();

        courseLoading = false;

    }

    private static void addListenerToRemovePlayer(JMenuItem removePlayer) {
        removePlayer.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                String[] possibilities = getPlayerNames(pp);
                if (possibilities == null || possibilities.length == 0) return;
                String s = (String) JOptionPane.showInputDialog(
                        frame,
                        "Name", "Enter Name",
                        JOptionPane.PLAIN_MESSAGE,
                        null,
                        possibilities,
                        possibilities[0]);

                for (int i = pp.size() - 1; i >= 0; i--) {
                    if (pp.get(i).getName().equals(s)) pp.remove(i);
                }
            }
        });
    }

    private static String[] getPlayerNames(ArrayList<Player> pp) {
        if (pp == null) return null;
        String[] names = new String[pp.size()];
        for (int i = 0; i < pp.size(); i++) {
            names[i] = pp.get(i).getName();
        }
        return names;
    }

    private static void addListenerToAddPlayer(JMenuItem addPlayer) {
        addPlayer.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                String s = (String) JOptionPane.showInputDialog(
                        frame,
                        "Name", "Enter Name",
                        JOptionPane.PLAIN_MESSAGE,
                        null,
                        null,
                        "Hans");

                Player p = new HumanPlayer(s);
                Game.addPlayer(p);


                Tile t = course.getStartTile();
                p.setBallPositionToCoordinateAndSetSpeedToZero(t.x, t.y, t.z);
                pp.add(p);
            }
        });
    }

    private static void addPlayer(Player p) {
        pp.add(p);
        dp.setPlayers(pp);
        physics.init(pp,course);
    }

    public static void placeObject(int x, int y) {
        course.integrate(previewMiniCourse, x, y);
        select.setSelected(false);
        previewMiniCourse = null;
        dp.setPreviewObject(null);
        dp.repaint();
    }

    public static Coordinate getWind() {
        return physics.getWind();
    }


    private static class miniDraw extends JPanel {
        public miniDraw(Dimension dimension) {

        }
    }
}

