package Magic.golf;

import Game.Actors.Bots.Stroke2Bot;
import Game.Actors.Move;
import Game.Actors.Player;
import Game.Model.*;
import Game.Unused.Editor;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.image.BufferedImage;
import java.beans.Transient;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;

import static Game.Game.AI;

/**
 * Created by nibbla on 16.03.16.
 */
public class DrawPanel extends JPanel {
    BufferedImage courseImage;
    Course course;
    public ArrayList<Player> players = new ArrayList<>();

    public int firstXClick = 0;
    public int firstYClick = 0;
    public Player currentPlayer;
    public static BufferedImage grassTexture;
    public static BufferedImage sandTexture;
    public static BufferedImage waterTexture;
    public static HashMap<Point, Color> GrassText = new HashMap<>();
    public static HashMap<Point, Color> SandText= new HashMap<>();
    public static HashMap<Point, Color> WaterText= new HashMap<>();

    public static BufferedImage holeTexture;
    public static BufferedImage objectTexture;
    public HashMap<Point, Color> HoleText= new HashMap<>();
    public static HashMap<Point, Color> ObjectText = new HashMap<>();
    public static TexturePaint holeP;
    public static TexturePaint ballP;
    public static BufferedImage ballTexture;
    public static BufferedImage ballImage;
    public boolean prepareShoot;
    private BufferedImage previewObject;
    private Graphics2D g2;

    private Move previewMove;
    private Move[] previewMoves;



    public void setPlayers(ArrayList<Player> p) {
        this.players = p;
    }

    public void setCurrentPlayer(Player p) {

        this.currentPlayer = p;

    }

    public DrawPanel() {
        loadTextures();

        this.addMouseMotionListener(new MouseMotionListener() {
            @Override
            public void mouseDragged(MouseEvent e) {
                repaint();
            }

            @Override
            public void mouseMoved(MouseEvent e) {
                if (previewObject!=null) repaint();
            }
        });
        DrawPanel dp = this;
        this.addMouseListener(new MouseListener() {
            @Override
            public void mouseClicked(MouseEvent e) {
                if(e.getButton()== MouseEvent.BUTTON3){
                    System.out.print("SurfaceNormal:" + course.getSurfaceNormals()[e.getX()][e.getY()].toString());
                    System.out.print("Shading%:" + course.getShadingMap()[e.getX()][e.getY()]);
                    Editor.addObject(e.getPoint());

                }

                if (previewObject != null){
                    Point mp = MouseInfo.getPointerInfo().getLocation();
                    Point pp = dp.getLocationOnScreen();

                    Point mousePosition = new Point(mp.x-pp.x,mp.y-pp.y);

                    Game.placeObject(mousePosition.x,mousePosition.y);



                }

            }

            @Override
            public void mousePressed(MouseEvent e) {
                if(e.getButton()!= MouseEvent.BUTTON3) {
                    setFirstXClick(e.getX());
                    setFirstYClick(e.getY());
                    prepareShoot=true;
                }
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                if(e.getButton()!= MouseEvent.BUTTON3) {
                    /*System.out.println("first x: " + getFirstXClick());
                    System.out.println("first Y: " + getFirstYClick());
                    System.out.println("second x: " + e.getX());
                    System.out.println("second Y: " + e.getY());
                    System.out.println("deltaX: " + (getFirstXClick() - e.getX()) / 10);
                    System.out.println("deltaY: " + (getFirstYClick() - e.getY()));
                    */
                    if (currentPlayer != null)

                        currentPlayer.shootBall((-(getFirstXClick() - e.getX())) / 2, -(getFirstYClick() - e.getY()), 0);
                        prepareShoot = false;
                }

            }

            @Override
            public void mouseEntered(MouseEvent e) {

            }

            @Override
            public void mouseExited(MouseEvent e) {

            }
        });
    }

    public static void loadTextures() {



        try {
            String current = new File( "." ).getCanonicalPath();
            System.out.println("Current dir:"+current);
            String t = Config.getTexturePath();
            File f =new File(t+"grass.jpg");
            grassTexture = ImageIO.read(f);
            sandTexture = ImageIO.read(new File(t+"sand.jpg"));
            waterTexture = ImageIO.read(new File(t+"water.jpg"));
            holeTexture = ImageIO.read(new File(t+"hole.jpg"));
            objectTexture = ImageIO.read(new File(t+ "object.jpg"));
            ballTexture = ImageIO.read(new File(t+"ball.jpg"));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        for (int x = 0; x < 32; x++) {
            for (int y = 0; y < 32; y++) {
                Point p = new Point(x,y);
                GrassText.put(p,new Color(grassTexture.getRGB(x , y )));
                SandText.put(p,new Color(sandTexture.getRGB(x, y )));
                WaterText.put(p,new Color(waterTexture.getRGB(x , y)));
                ObjectText.put(p,new Color(objectTexture.getRGB(x , y)));

            }
        }

        holeP = new TexturePaint(holeTexture,new Rectangle(0,0,32,32));
        ballP = new TexturePaint(ballTexture,new Rectangle(0,0,32,32));
        precalcBallImage();
    }

    public static void precalcBallImage() {
        ballImage = new BufferedImage((int)(Config.getBallRadius()*2),(int)(Config.getBallRadius()*2),BufferedImage.TYPE_INT_ARGB);
        double radius = Config.getBallRadius();
        Graphics2D g2 = ballImage.createGraphics();
        Color[][] colorMap = new Color[(int) radius*2][(int) radius*2];
        for (int x = (int) -radius; x < radius; x++) {
            for (int y = (int) -radius; y < radius; y++) {
                for (int z = 0; z < radius; z++) {
                    double length = Math.sqrt(x*x+y*y+z*z);
                    double xt = x;
                    double yt = y;
                    double zt = z;
                    if (length<=radius){

                        xt /= length;
                        yt /= length;
                        zt /= length;
                        float value = (float) Math.acos(Config.getLightningVector3d()[0] * xt + Config.getLightningVector3d()[1] *  yt+Config.getLightningVector3d()[2] *  zt);
                        System.out.println("x: " + x + " y: " + y + " has the angle two north west " + Math.toDegrees(value) );
                        value = (float)(value/Math.PI);
                        colorMap[(int)(x+radius)][(int)(y+radius)] = new Color(0,0,0,value*0.7f);
                    }
                }
            }

        }

        for (int x = 0; x < colorMap.length; x++) {
            for (int y = 0; y < colorMap[0].length; y++) {
                Color c = colorMap[x][y];
                if (c!=null){
                    g2.setColor(c);
                    g2.fillRect(x, y, 1, 1);
                    g2.fillRect(x, y, 1, 1);
                }

            }

        }

    }


    public void setCourse(Course c) {
        this.course = c;
        courseImage = c.getManagedBufferedImage();
    }

    public static BufferedImage createImageStandart(Course course) {
        if (grassTexture==null)DrawPanel.loadTextures();
        Type[][][] pf = course.getPlayfield();
        int[][] hm = course.getHeightMap();
        if (hm==null){
            course.calculateHeightMapSafe();
            hm = course.getHeightMap();
        }
        float[][] sm = course.getShadingMap();
        if (sm ==null) {
            course.calculateShadingMap();
            sm = course.getShadingMap();
        }
        Coordinate[][] normals = course.getSurfaceNormals();
        if (normals == null){
            course.calculateSurfaceNormalsSafe();
            normals =course.getSurfaceNormals();

        }
        int[] d = course.getDimension();
        BufferedImage bufferedImage =
                new BufferedImage(d[0], d[1], BufferedImage.TYPE_INT_ARGB);
        Graphics2D g = bufferedImage.createGraphics();

        Random r = new Random(0);
        HashMap<Integer,Color> hightColorFromZ = new HashMap<>(100);
        HashMap<Float,Color> shadingColorFromZ = new HashMap<>(100);
        Color shadingC;
        Color zshadow;

            for (int x = 0; x < d[0]; x++) {
                for (int y = 0; y < d[1]; y++) {
                    int z = hm[x][y];
                    Type t = pf[x][y][z];
                    if (t == Type.Empty) continue;
                    Point p = new Point((x)%32,(y)%32);
                    if (t == Type.Grass) g.setColor(GrassText.get(p));
                    if (t == Type.Sand) g.setColor(SandText.get(p));
                    if (t == Type.Water) g.setColor(WaterText.get(p));
                    if (t  == Type.OBJECT) g.setColor(ObjectText.get(p));


                    g.fillRect(x, y, 1, 1);

                    zshadow = hightColorFromZ.get(z);
                    if (zshadow == null){
                        float f = (float) ((z*1f/d[2])*0.3);
                        zshadow = new Color(0,0,0,f);
                        hightColorFromZ.put(z,zshadow);
                    }
                    g.setColor(zshadow);

                   g.fillRect(x, y, 1, 1);

                    //schraffurShaddow
                    float ss =  sm[x][y];
                    shadingC = shadingColorFromZ.get(ss);
                    if (shadingC == null){
                        shadingC = new Color(0,0,0,ss*0.9f);

                        shadingColorFromZ.put(ss,shadingC);
                    }


                    g.setColor(shadingC);
                     g.fillRect(x, y,  1,  1);

                }

            }





        g.dispose();
        return bufferedImage;
    }

    public static BufferedImage createImage(Course course) {
        if (grassTexture==null)DrawPanel.loadTextures();
        Type[][][] pf = course.getPlayfield();
        int[][] hm = course.getHeightMap();
        float[][] sm = course.getShadingMap();
        Coordinate[][] normals = course.getSurfaceNormals();
        if (hm==null){
            course.calculateHeightMapSafe();
            hm = course.getHeightMap();
        }

        if (sm ==null) {
            course.calculateShadingMap();
            sm = course.getShadingMap();
        }

        if (normals == null){
            course.calculateSurfaceNormalsSafe();
            normals =course.getSurfaceNormals();

        }

        int[] d = course.getDimension();
        BufferedImage bufferedImage =
                new BufferedImage(d[0], d[1], BufferedImage.TYPE_INT_ARGB);

        Graphics2D g = bufferedImage.createGraphics();

        Random r = new Random();
        HashMap<Integer,Color> hightColorFromZ = new HashMap<>(100);
        HashMap<Float,Color> shadingColorFromZ = new HashMap<>(100);
        Color shadingC;
        Color zshadow;



        for (int x = 0; x < d[0]; x++) {
            for (int y = 0; y < d[1]; y++) {
                int z = hm[x][y];
                Type t = pf[x][y][z];
                if (t == Type.Empty) continue;
                Point p = new Point((x)%32,(y)%32);
                if (t == Type.Grass) g.setColor(GrassText.get(p));
                if (t == Type.Sand) g.setColor(SandText.get(p));
                if (t == Type.Water) g.setColor(WaterText.get(p));
                if (t  == Type.OBJECT) g.setColor(ObjectText.get(p));


                g.fillRect(x, y, 1, 1);

            }
        }





        for (int x = 0; x < d[0]; x++) {
            for (int y = 0; y < d[1]; y++) {
                int z = hm[x][y];
                zshadow = hightColorFromZ.get(z);
                if (zshadow == null){
                    float f = (float) ((z*1f/d[2])*0.3);
                    zshadow = new Color(0,0,0,f);
                    hightColorFromZ.put(z,zshadow);
                }
                g.setColor(zshadow);

                g.fillRect(x, y, 1, 1);

                //schraffurShaddow
                float ss =  sm[x][y];
                shadingC = shadingColorFromZ.get(ss);
                if (shadingC == null){
                    shadingC = new Color(0,0,0,ss*0.9f);

                    shadingColorFromZ.put(ss,shadingC);
                }


                g.setColor(shadingC);
                g.fillRect(x, y,  1,  1);

            }

        }





        g.dispose();
        return bufferedImage;
    }

    @Override
    protected void paintComponent(Graphics g) {

        super.paintComponent(g);

        g2 = (Graphics2D)g;//this new object g2,will get the
        Stroke oldStroke = g2.getStroke();

        if (courseImage != null) g.drawImage(course.getManagedBufferedImage(), 0, 0, null);
        drawHole(g);

        if(AI instanceof Stroke2Bot){
            Stroke2Bot stroke2Bot = (Stroke2Bot) AI;
            Coordinate c = stroke2Bot.getShootLocation();
            if(c!=null){


                g2.setStroke(new BasicStroke(3));
                g2.setColor(Color.red);
                g2.drawLine((int) (c.getX()-12) ,(int) (c.getY()-6),(int) (c.getX()+12),(int) (c.getY()+6));
                g2.drawLine((int) (c.getX()+12) ,(int) (c.getY()-6),(int) (c.getX()-12),(int) (c.getY()+6));

            }
        }


        g2.setColor(Color.BLUE);
        if(previewMoves != null){
            for (int i = 0; i < previewMoves.length; i++) {
                Coordinate c = previewMoves[i].attainedTarget;
                if (c == null) continue;
                g2.setStroke(new BasicStroke(3));
                g2.drawLine((int) (c.getX()-12) ,(int) (c.getY()-6),(int) (c.getX()+12),(int) (c.getY()+6));
                g2.drawLine((int) (c.getX()+12) ,(int) (c.getY()-6),(int) (c.getX()-12),(int) (c.getY()+6));

                g2.setStroke(new BasicStroke(1));

                Coordinate c2 = previewMoves[i].c;
                c = previewMoves[i].startCoordinate;
                g2.drawLine((int) c.getX() ,(int) c.getY(),(int) (c.getX()+c2.getX()),(int) (c.getY()+c2.getY()));

            }


        }
        g2.setStroke(new BasicStroke(3));

        if(previewMove != null){
                g2.setColor(Color.magenta);
                Coordinate c = previewMove.attainedTarget;

                g2.drawLine((int) (c.getX()-12) ,(int) (c.getY()-6),(int) (c.getX()+12),(int) (c.getY()+6));
                g2.drawLine((int) (c.getX()+12) ,(int) (c.getY()-6),(int) (c.getX()-12),(int) (c.getY()+6));

                g2.setColor(Color.red);
                Coordinate c2 = previewMove.c;
                c = previewMove.startCoordinate;
                g2.drawLine((int) c.getX() ,(int) c.getY(),(int) (c.getX() + c2.getX()),(int) (c.getY()+ c2.getY()));



        }
        g2.setStroke(oldStroke);
        for (int i = 0; i < players.size(); i++) {
            if (players.get(i).getCurrentStrokes()==0&&!currentPlayer.equals(players.get(i)))
                continue;
            Ball b = players.get(i).getBall();
            if (!b.inHole)drawBallShadow(g,b);
        }
        for (int i = 0; i < players.size(); i++) {
            if (players.get(i).getCurrentStrokes()==0&&!currentPlayer.equals(players.get(i)))
                continue;
            Ball b = players.get(i).getBall();
            if (!b.inHole)drawBall(g,b);
        }

        if (currentPlayer != null){
            drawPowerLine(g,currentPlayer.getBall());
            Coordinate c = currentPlayer.getBall().getCoordinate();
            double radius = currentPlayer.getBall().getRadius();
            //if (c != null)
            g.setColor(Color.BLACK);
            g.drawOval((int) (c.getX() - radius), (int) (c.getY() - radius), (int) radius * 2, (int) radius * 2);
        }

        if (previewObject!=null){
            Point mp = MouseInfo.getPointerInfo().getLocation();
            Point pp = this.getLocationOnScreen();

            Point mousePosition = new Point(mp.x-pp.x,mp.y-pp.y);
            g.drawImage(previewObject, mousePosition.x, mousePosition.y, null);

        }
        Coordinate wind = Game.getWind();
        if (wind!= null) drawWindpanel(g2,wind);

    }

    private void drawWindpanel(Graphics2D g2, Coordinate wind) {
        int x = this.getWidth()*15/16;
        int y = this.getHeight()*1/16;

        int radius = 30;

        g2.setPaint(Color.BLACK);
        g2.fillOval(x-radius,y-radius,2*radius,2*radius);
        radius -= 1;
        g2.setPaint(Color.WHITE);
        g2.fillOval(x-radius,y-radius,2*radius,2*radius);
        radius -= 3;

        int lengthX = (int)(  radius *  wind.getX()/ Config.MAXWIND);
        int lengthY = (int) (radius * wind.getY()/ Config.MAXWIND);
        g2.setPaint(Color.MAGENTA);
        g2.setStroke(new BasicStroke(3));

        int x2 = (int) (this.getWidth()*15/16 + lengthX);
        int y2 = (int) (this.getHeight()*1/16+ lengthY);;

        g2.drawLine(x,y,x2,y2);




    }


    private void drawBallShadow(Graphics g, Ball b) {

        Graphics2D g2 = (Graphics2D) g;
        int[] dim = course.getDimension();
        int[][] bbb = b.getSurfacePoints();
        g.setColor(new Color(0,0,0,0.9f));
        int[] xs = new int[bbb.length] ;
        int[] ys = new int[bbb.length] ;
        ArrayList<Point> shadow = new ArrayList<>(bbb.length);

        outerloop:for (int i =  bbb.length-1; i >=0; i--) {

            double[] d = Config.getLightningVector3d();
            double x = bbb[i][0];
            double y = bbb[i][1];
            double z = bbb[i][2];

            while (true){
                x -= d[0];
                y -= d[1];
                z -= d[2];

                if (x<0||x>dim[0]||y<0||y>dim[1]||z<0){
                    continue outerloop;
                }
                if (z>dim[2]){
                    continue;
                }


                if (course.getTile((int)x,(int)y,(int)z) !=Type.Empty){
                    shadow.add(new Point((int) x, (int) y));


                    continue outerloop;
                }

            }

        }



        QuickHull qh = new QuickHull();
        ArrayList<Point> p = qh.quickHull(shadow);
        int[] xs2 = new int[p.size()];
        int[] ys2 = new int[p.size()];
        int n = p.size();

        for (int i = 0; i < p.size(); i++) {
            Point ps =  p.get(i);
            xs2[i] = ps.x;
            ys2[i] = ps.y;
        }
        g2.setColor(new Color(0,0,0,0.3f));
        g2.fillPolygon(xs2,ys2,n);
    }

    public void drawPowerLine(Graphics g, Ball b) {
        if (!prepareShoot) return;

        Point mp = MouseInfo.getPointerInfo().getLocation();
        Point pp = this.getLocationOnScreen();

        Point mousePosition = new Point(mp.x-pp.x,mp.y-pp.y);


        Graphics2D g2 = (Graphics2D) g;
        g2.setStroke(new BasicStroke(Config.POWERLINEWIDTH));
        g2.setColor(Color.red);
        Coordinate c = b.getCoordinate();
        g2.drawLine(firstXClick,firstYClick, mousePosition.x,mousePosition.y);



    }

    public void drawBall(Graphics g, Ball b) {
        Graphics2D g2 = (Graphics2D) g;

        Coordinate c = b.getCoordinate();
        double radius = b.getRadius();
        //if (c != null)
        g2.setPaint(ballP);
        g2.fillOval((int) (c.getX() - radius), (int) (c.getY() - radius), (int) radius*2, (int) radius*2);

        g2.drawImage(ballImage, (int) (c.getX() - radius), (int) (c.getY() - radius),null );


    }

    public void drawHole(Graphics g) {
        Graphics2D g2 = (Graphics2D) g;




        Hole t = course.getHole();
        g2.setPaint(Color.BLACK);
        g2.fillOval((int) (t.getX() - t.radius+2), (int) (t.getY() - t.radius+2), (int) (t.radius*2-4), (int) (t.radius*2-4));


    }


    @Override
    @Transient
    public Dimension getPreferredSize() {
        return new Dimension(courseImage.getWidth(), courseImage.getHeight());
    }

    public int getFirstXClick() {

        return firstXClick;
    }

    public void setFirstXClick(int firstXClick) {
        this.firstXClick = firstXClick;
    }

    public int getFirstYClick() {

        return firstYClick;
    }

    public void setFirstYClick(int firstYClick) {
        this.firstYClick = firstYClick;
    }

    public void setPreviewObject(BufferedImage previewObject) {
        this.previewObject = previewObject;
    }



    public void setPreviewMove(Move previewMove) {
        this.previewMove = previewMove;
    }

    public void resetAIPreview() {

        previewMoves=null;
        previewMove = null;
    }

    public void setPreviewMoves(Move[] previewMoves) {
        this.previewMoves = previewMoves;
    }


}
