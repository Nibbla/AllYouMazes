package Magic.golf; /**
 * Created by nibbla on 14.03.16.
 */

import Game.*;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.StringTokenizer;

/** A course is defined by a 3 dimensional grid containing of tiles
 * A typical course would be 800*600*100 tiles, mostly empty
 * A tile is 1cm³
 * @author maternal insult
 *
 */
public class Course {
    public String name;
    public Coordinate[][]surfaceNormals;
    public HashMap<IntCoordinate,Coordinate> surfaceNormalsInGood;
    private IntCoordinate lookup = new IntCoordinate(0,0,0);
    private Coordinate zeroInt = new Coordinate(0,0,0);
    public int[][]heightMap;



    private float[][] shadingMap;


    public Type[][][] playfield;

    int[] dimension;

    public int par;
    public Tile startTile;
    public Hole hole;
    //public Stack<Tile> oldTiles = new Stack<Tile>();
    public BufferedImage managedBufferedImage;



    /**
     * Constructor of the Course class
     * which creates the new playfield with 3 dimensions
     * and filles the playfield with empty tiles
     * and which stores objects in an arraylist of the arraylist of the new playfield
     *  @param name Name of the course
     * @param length length of the new Tile
     * @param width width of the new Tile
     * @param height height of the new Tile
     * @param standartType the lowest level of the level will be filled with this tile
     * @param par
     */
    public Course(String name, int length, int width, int height, Type standartType, int par){
        this.name = name;
        playfield = new Type[length][width][height];
        dimension = new int[3]; dimension[0] =length; dimension[1] =width; dimension[2]=height;



        //fills the playfield with empty tiles
        for (int x = 0; x < length; x++) {
            for (int y = 0; y < width; y++) {
                for (int z = 0; z < height; z++) {
                    playfield[x][y][z] = Type.Empty;
                }
            }
        }
        Random r = new Random(0);
        for (int x = 0; x < length; x++) {
            for (int y = 0; y < width; y++) {
                    Type typ;
                    if (standartType == null) typ = Type.values()[r.nextInt(Type.values().length)];
                    else typ =standartType;

                    playfield[x][y][0] = typ;
            }
        }


        this.par = par;

    }

    /**
     * setter to set the tiles
     * no boundary check
     * @param x
     * @param y
     * @param z
     * @param t
     */
    public void setTileFast(int x, int y, int z, Type t){


        if (t== Type.Hole){
            hole = new Hole(Config.getHoleRadius(), x,y,z) ;
            for (int zd = 0+z; zd < 10; zd++) {
                for (int xd = (int) -Config.getHoleRadius()+x; xd < Config.getHoleRadius()+x; xd++) {
                    for (int yd = (int) -Config.getHoleRadius()+y; yd < Config.getHoleRadius()+y; yd++) {
                        if (Math.sqrt((x-xd)*(x-xd)+(y-yd)*(y-yd))< Config.getHoleRadius()){
                            setTile(xd,yd,zd,Type.Empty);
                        }
                    }
                }
            }
            playfield[x][y][z] = t;
        }else if (t == Type.Start) {
            startTile = new Tile(t,x,y,z);
            playfield[x][y][z] = t;
        }else{
            playfield[x][y][z] = t;
        }

    }

    /**
     * setter to set the tiles
     * @param x
     * @param y
     * @param z
     * @param t
     */
    public void setTile(int x, int y, int z, Type t){
        if (x < 0) {
            return;
        }

        if (y < 0) {
            return;
        }
        if (z < 0) {
            return;
        }

        if (x >= dimension[0]) {
            return;
        }
        if (y >= dimension[1]) {
            return;
        }
        if (z >= dimension[2]) {
            return;
        }


        setTileFast(x,y,z,t);

    }

    public Type getTile(int x, int y, int z){
        return playfield[x][y][z];
    }

    /**
     * getter to get the playfield
     * @return playfield
     */
    public Type[][][] getPlayfield(){
        return playfield;
    }

    /**
     * method loadCourse created the new course
     * @return c, the new course
     */
    public static Course loadCourse2_5d(String path){
        System.out.println("Working Directory = " +
                System.getProperty("user.dir"));
        Course c = null;
        StringTokenizer stringTokenizer;
        try {
            BufferedReader b = new BufferedReader(new FileReader(path));

       // String content = Utils.readFile(path);
       // if (content==null||content=="")return null;
      //  String[] lines = content.split(System.lineSeparator());

        String name = b.readLine().split(":")[1];
            String version = b.readLine().split(":")[1];
            if (!version.equals(Config.version)) return null;

        int par = Integer.parseInt(b.readLine().split(":")[1]);
        int length = Integer.parseInt(b.readLine().split(":")[1]);
        int width = Integer.parseInt(b.readLine().split(":")[1]);
        int height = Integer.parseInt(b.readLine().split(":")[1]);
        c = new Course(name,length,width,height, Type.Empty,par);
            String currentLine;

            while ((currentLine = b.readLine()) != null) {
                stringTokenizer = new StringTokenizer(currentLine,";");


            int indexPosX = currentLine.indexOf("x");
            int indexPosY = currentLine.indexOf("y");
            int indexPosZ = currentLine.indexOf("z");
            int indexType = currentLine.indexOf("T");
            int indexSTOP = currentLine.indexOf("$");

            int x = Integer.parseInt( stringTokenizer.nextToken());
            int y = Integer.parseInt(stringTokenizer.nextToken());
            int z = Integer.parseInt(stringTokenizer.nextToken());
            Type type = Type.values()[Integer.parseInt(stringTokenizer.nextToken())];

                for (int zt = 0; zt <= z; zt++) {
                    c.setTileFast(x,y,zt,type);
                }

            if (type==Type.Start){
                c.startTile  = new Tile(Type.Start,x, y, z);
            }
                if (type==Type.Hole) {
                    c.hole = new Hole(Config.getHoleRadius(), x, y, 0);
                }
        }
        if (c.startTile == null){

            c.setTile(20, 20, 1, Type.Start);
            c.startTile = new Tile(Type.Start,20, 20, 1);
        }
        if (c.hole == null){
            int x = (int) (length*0.8);
            int y = (int) (width*0.8);
            c.setTile(x, y, 0, Type.Hole);
            c.hole = new Hole(Config.getHoleRadius(),x, y, 0);
        }






        c.calculateSurfaceNormalsSafe();

        System.out.println("try to read " + c.getName()+".png");
        BufferedImage bi = null;
            try {

            bi = ImageIO.read(new File(Config.CourseLocation + c.getName() + ".png"));
            }catch (IOException e){
            c.calculateHeightMapSafe();
            bi = DrawPanel.createImage(c);
            Utils.saveManagedBufferedImage(Config.CourseLocation + c.name + ".png",bi);
         }finally {
            c.setBufferedImage(bi);
         }
        } catch (FileNotFoundException e) {
            e.printStackTrace();

        } catch (IOException e) {
            System.out.println("tried to read " + c.getName()+".png");
            e.printStackTrace();

        }
        return c;

    }


    /**
     * method loadCourse created the new course
     * @return c, the new course
     * @deprecated we desided to have no holes in our course,
     * to speed up the loading. use LoadCourse3d instead
     */
    @Deprecated
    public static Course loadCourse(String path){
        System.out.println("Working Directory = " +
                System.getProperty("user.dir"));
        Course c = null;
        try {
            BufferedReader b = new BufferedReader(new FileReader(path));

            // String content = Utils.readFile(path);
            // if (content==null||content=="")return null;
            //  String[] lines = content.split(System.lineSeparator());

            String name = b.readLine().split(":")[1];

            int par = Integer.parseInt(b.readLine().split(":")[1]);
            int length = Integer.parseInt(b.readLine().split(":")[1]);
            int width = Integer.parseInt(b.readLine().split(":")[1]);
            int height = Integer.parseInt(b.readLine().split(":")[1]);
            c = new Course(name, length,width,height, Type.Empty,par);
            String currentLine;

            while ((currentLine = b.readLine()) != null) {

                int indexPosX = currentLine.indexOf("x");
                int indexPosY = currentLine.indexOf("y");
                int indexPosZ = currentLine.indexOf("z");
                int indexType = currentLine.indexOf("T");
                int indexSTOP = currentLine.indexOf("$");

                int x = Integer.parseInt(currentLine.substring(indexPosX + 1, indexPosY));
                int y = Integer.parseInt(currentLine.substring(indexPosY+1,indexPosZ));
                int z = Integer.parseInt(currentLine.substring(indexPosZ+1,indexType));
                Type type = Type.values()[Integer.parseInt(currentLine.substring(indexType + 1, indexSTOP))];

                c.setTile(x,y,z,type);
                if (type==Type.Start){
                    c.startTile  = new Tile(Type.Start,x, y, z);
                }
                if (type==Type.Hole) {
                    c.hole = new Hole(Config.getHoleRadius(), x, y, 0);
                }
            }
            if (c.startTile == null){

                c.setTile(20, 20, 1, Type.Start);
                c.startTile = new Tile(Type.Start,20, 20, 1);
            }
            if (c.hole == null){
                int x = (int) (length*0.8);
                int y = (int) (width*0.8);
                c.setTile(x, y, 0, Type.Hole);
                c.hole = new Hole(Config.getHoleRadius(),x, y, 0);
            }
            c.calculateSurfaceNormalsSafe();
            System.out.println("try to read " + c.getName()+".png");
            BufferedImage bi = ImageIO.read(new File(Config.CourseLocation + c.getName()+".png"));
            c.setBufferedImage(bi);


        } catch (FileNotFoundException e) {
            e.printStackTrace();
            return null;
        } catch (IOException e) {
            System.out.println("tried to read " + c.getName()+".png");
            e.printStackTrace();
            return null;
        }



        return c;

    }

    /**
     * method saveCourse saves the course as a .txt file
     */
    public void saveCourse(){
        Utils.saveCourse2_5D(this);




    }

    /**
     * getter to get the name of the course
     * @return name
     */
    public String getName() {
        return name;
    }



    /**
     * getter to get the dimension
     * @return dimension
     */
    public int[] getDimension() {
        return dimension;
    }

    /**
     * getter to get the length of the course
     * @return dimension[2]
     */
    public int getDepth() {
        return dimension[2];
    }

    /**
     * getter to get the width of the course
     * @return dimension[0]
     */
    public int getWidth() {
        return dimension[0];
    }

    /**
     * getter to get the height of the course
     * @return dimension[1]
     */
    public int getHeight() {
        return dimension[1];
    }

    /**
     * getter to get the StartTile
     * @return startTile
     */
    public Tile getStartTile() {
        return startTile;
    }

    /**
     * method to add a rectangle to the course
     * To set a tile to an object
     * @param x1 x coordinate in the course to place the object
     * @param y1 y coordinate in the course to place the object
     * @param width of the course
     * @param height of the course
     * @param type
     */
    private void addRectangle(int x1, int y1, int z1, int width, int height, Type type) {

        int initialX=x1;
        int initalY=y1;
        for (int x = x1; x < initialX+width; x++) {
            for (int y = y1; y < initalY+height; y++) {
                setTile(x,y,z1,type);
            }
        }

    }

    /**
     * method to add a rectangle to the course
     * To set a tile to an object
     * @param x1 x coordinate in the course to place the object
     * @param y1 y coordinate in the course to place the object
     * @param width of the course
     * @param height of the course
     * @param type
     */
    public void addCuboid(int x1, int y1, int z1, int length, int width, int height, Type type) {
        addFrustrum(x1,y1,z1,length,width,height,0,0,0,0,type);

    }

    /**
     * method to add a squircle to the course
     * @param a
     * @param b
     * @param r
     * @param n
     * @param z
     * @param t
     * @deprecated  dont expect it to work
     */@Deprecated
    public void addSquircle(int a, int b, int r, double n, int z, Type t) {

        for (int x = -r+a; x < r+a; x++) {
            for (int y = -r+b; y < r+b; y++) {
              if(Math.pow(x-a,n)+Math.pow(y-b,n)<Math.pow(r,n)){

                  setTile(x,y,z,t);
              }
            }

        }


    }

    /**
     * getter to get the hole
     * @return hole
     */
    public Hole getHole() {
        return hole;
    }

    /**
     * setter to set the name of the course
     * @param name
     */
    public void setName(String name) {
        this.name = name;
    }

    public void removeLastObject() {
       // if (oldTiles.size() == 0) return;
       // Tile t = oldTiles.pop();

      //  do{


           // this.setTile(t.getX(), t.getY(), t.getZ(), t.getType());
           // t = oldTiles.pop();

       // }while(t.t != Type.$MARKER);
    }

    /**
     *  we draw layer after layer squares from botom to top
     you define x,y,z of the ground point in the upper left courner, then length, width, depth and then the steepnis.
     this is done by giving 4 integers. xDeltaLeft and xDeltaRight. and yDeltaTop and yDeltaBottom
     xDeltaLeft = 2 means that each layer the left boundery moves two to the right.
     xDeltaRight =-2 means that each layer the left boundery moves two to the left.
     yDeltaTop and Bottom do it respectifly
     * @param initialX
     * @param initialY
     * @param initialZ
     * @param length
     * @param height
     * @param depth
     * @param leftXDeltaPerLayer
     * @param rightXDeltaPerLayer
     * @param topYDeltaPerLayer
     * @param bottomYDeltaPerLayer
     * @param t
     */
    public void addFrustrum(int initialX, int initialY, int initialZ, double length, double height, int depth, double leftXDeltaPerLayer, double rightXDeltaPerLayer, double topYDeltaPerLayer, double bottomYDeltaPerLayer, Type t){

        double x=initialX;
        double y=initialY;

        for (int z = initialZ; z < initialZ+depth; z++) {

            addRectangle((int)x,(int)y,z,(int)length,(int)height,t);


            x+=leftXDeltaPerLayer;
            if (leftXDeltaPerLayer>0)length-=leftXDeltaPerLayer; else length+=leftXDeltaPerLayer;
            if (rightXDeltaPerLayer<0)length+=rightXDeltaPerLayer; else length-=rightXDeltaPerLayer;
            y+=topYDeltaPerLayer;
            if (topYDeltaPerLayer>0) height-=topYDeltaPerLayer; else  height+=topYDeltaPerLayer;
            if  (bottomYDeltaPerLayer<0) height+=bottomYDeltaPerLayer; else height-=bottomYDeltaPerLayer;

            if (height<=0||length<=0)break;
        }

    }


    public void addHill(int initialX, int initialY, int initialZ,int depth ,double r, double DeltaRPerLayer, Type t) {

        double x=initialX;
        double y=initialY;

        for (int z = initialZ; z < initialZ+depth; z++) {
            addSquircle((int)x,(int)y,(int)r,2,z,t);

            r +=DeltaRPerLayer;
        }

    }


    public BufferedImage getManagedBufferedImage() {
        return managedBufferedImage;
    }

    public void setBufferedImage(BufferedImage bufferedImage) {
        this.managedBufferedImage = bufferedImage;
    }

    public Coordinate getNormal(int x, int y, int z) {


        double avgX = 0;
        double avgY = 0;
        double avgZ = 0;


        for (int dx = -3; dx <= 3; dx++) {
            for (int dy = -3; dy <= 3; dy++) {
                for (int dz = -3; dz <= 3; dz++) {
                    int checkX = x+dx;
                    int checkY = y+dy;
                    int checkZ = z+dz;
                    if (checkX<0||checkX>=dimension[0]||checkY<0||checkY>=dimension[1]||checkZ<0||checkZ>=dimension[2]
                            ||playfield[checkX][checkY][checkZ]!= Type.Empty) {
                        avgX -= dx;
                        avgY -= dy;
                        avgZ -= dz;
                    }
                }
            }
        }
        if (avgX==0&&avgY==0&&avgZ==0)return new Coordinate(0, 0, 0);
       /* double length =  Utils.fastInverseSqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);//Math.sqrt(); // distance from avg to the center

        avgX *= length;
        avgZ *= length;
        avgY *= length;*/
        double length =  Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);//Math.sqrt(); // distance from avg to the center

        avgX /= length;
        avgZ /= length;
        avgY /= length;
        return new Coordinate(avgX, avgY, avgZ);
    }
    public void calculateSurfaceNormals(){
        calculateSurfaceNormalsMap();
        surfaceNormals = new Coordinate[dimension[0]][dimension[1]];

        for (int x = 0; x <dimension[0];x++){
            for (int y = 0; y <dimension[1];y++){
                for (int z = 0; z <dimension[2]-1;z++){
                    if (playfield[x][y][z+1] == Type.Empty){
                        surfaceNormals[x][y] = getNormal(x,y,z);
                        break;
                    }
                }
            }
            ;
        }

    }
    public void calculateSurfaceNormalsSafe(){
        calculateSurfaceNormalsMap();
        surfaceNormals = new Coordinate[dimension[0]][dimension[1]];

        for (int x = 0; x <dimension[0];x++){
            yLoop:for (int y = 0; y <dimension[1];y++){
                for (int z = dimension[2]-1; z >=0;z--){
                    if (playfield[x][y][z] != Type.Empty){
                        surfaceNormals[x][y] = getNormal(x,y,z);
                        continue yLoop;
                    }
                }
                surfaceNormals[x][y] = getNormal(x,y,0);
            }
;
        }

    }

    private void calculateSurfaceNormalsMap() {
        surfaceNormalsInGood = new HashMap<IntCoordinate,Coordinate>((int) (dimension[0]*dimension[1]*3.4));
        for (int x = 0; x <dimension[0];x++){
            yLoop:for (int y = 0; y <dimension[1];y++){
                int z= dimension[2]-1;
                while (true){
                    if (z<=0) continue yLoop;
                    if (playfield[x][y][z] != Type.Empty){
                        Coordinate c = getNormal(x, y, z);
                        if (c.getLength()==0) {
                            continue yLoop;
                        }
                        surfaceNormalsInGood.put(new IntCoordinate(x, y, z), getNormal(x, y, z));
                    }
                    z--;
                }
            }

        }
    }

    public void calculateHeightMap(){
        heightMap = new int[dimension[0]][dimension[1]];

        for (int x = 0; x <dimension[0];x++){
            for (int y = 0; y <dimension[1];y++){
                for (int z = 0; z <dimension[2]-1;z++){
                    if (playfield[x][y][z+1] == Type.Empty){
                        heightMap[x][y] = z;
                        break;
                    }
                }
            }

        }

    }

    public void calculateHeightMapSafe() {
        heightMap = new int[dimension[0]][dimension[1]];

        for (int x = 0; x <dimension[0];x++){
            for (int y = 0; y <dimension[1];y++){
                for (int z = dimension[2]-1; z >= 0;z--){
                    if (playfield[x][y][z] != Type.Empty){
                        heightMap[x][y] = z;
                        break;
                    }
                }
            }

        }
    }
    public void calculateShadingMap(){
        double[] lv = Config.getLightningVector3d(); //lighting vector
        shadingMap = new float[dimension[0]][dimension[1]];
        if (surfaceNormals==null) calculateSurfaceNormalsSafe(); //this should actually never be executed
        for (int x = 0; x <dimension[0];x++){
            for (int y = 0; y <dimension[1];y++){
                    //angle = acos(v1•v2)
                     Coordinate c = surfaceNormals[x][y];
                     double[] v = {c.getX(),c.getY(),c.getZ()};
                     //normaliseV
                    double length = Math.sqrt(v[0] * v[0] + v[1] * v[1]+v[2] * v[2]);
                    v[0] /= length;
                    v[1] /= length;
                    v[2] /= length;
                    double value = Math.acos(lv[0] *  v[0] + lv[1] *  v[1]+lv[2] *  v[2]);
                    System.out.println("x: " + x + " y: " + y + " has the angle two north west " + Math.toDegrees(value) );
                    value = value/Math.PI;

                   if (Double.isNaN(value)) shadingMap[x][y] = 0; else shadingMap[x][y] = (float) value;
            }

        }

    }

    public Coordinate[][] getSurfaceNormals() {
        return surfaceNormals;
    }

    public int[][] getHeightMap() {
        return heightMap;
    }

    public float[][] getShadingMap() {
        return shadingMap;
    }


    public void finalise() {
        System.out.println("CalculateSurfaceNormals");
        calculateSurfaceNormalsSafe();
        System.gc();
        System.out.println("CalculateHightMap");
        calculateHeightMapSafe();
        System.gc();
        System.out.println("CalculateShadingMap");
        calculateShadingMap();
        System.gc();
        System.out.println("CalculateDrawPanel");
        System.gc();
        setBufferedImage(DrawPanel.createImage(this));
        System.gc();
    }

    public void integrate(Course previewMiniCourse, int x, int y) {
        Type[][][] pmc = previewMiniCourse.getPlayfield();
        int[] pmcD = previewMiniCourse.getDimension();
        BufferedImage bi = previewMiniCourse.getManagedBufferedImage();

        int insideX = 0;
        int insideY = 0;



        for ( insideX = 0; insideX+x<dimension[0]&&insideX<pmcD[0];insideX++) {
            for ( insideY = 0; insideY + y < dimension[1] && insideY < pmcD[1]; insideY++) {
                for (int z= 0; z<dimension[2];z++){
                    if (pmc[insideX][insideY][z]!=Type.Empty){
                    playfield[insideX+x][insideY+y][z] = pmc[insideX][insideY][z];
                    managedBufferedImage.setRGB(insideX+x,insideY+y,bi.getRGB(insideX,insideY));}
                }

            }
        }

    }

    public Type getType(Coordinate newPosition) {

        return null;
    }
    public boolean wayIsObstacleFree(Coordinate origin, Coordinate destination){
        //Coordinate destination2 = new Coordinate(destination.getX(),destination.getY(),heightMap[(int)destination.getX()][(int)destination.getY()]+Config.ballRadius);
        System.out.println("wayIsObstacleFree: " + destination.toString());
        ArrayList<int[]> all = Coordinate.getPixelBetween((int)origin.getX(), (int)origin.getY(), (int)destination.getX(), (int)destination.getY());
        for (int[]coord:all) {
            Coordinate c = new Coordinate(coord[0], coord[1], heightMap[coord[0]] [coord[1]]);
            Type t = playfield[(int)c.getX()][(int)c.getY()][(int)(c.getZ())];
            if (t == ((Type.OBJECT))){
                System.out.println("Would colide with " + playfield[(int) c.getX()][(int) c.getY()][(int) (c.getZ())] + "\n");
                return false;

            }
        }
        System.out.println("Target approved");
        System.out.println("---------------");
        return true;
    }

    public boolean wayIsObstacleFree(Coordinate cord1, Coordinate cord2, boolean liftCoord1,  boolean liftCoord2){
        Coordinate c1 = new Coordinate(cord1.getX(),cord1.getY(),cord1.getZ()+ ((liftCoord1)? 1 : 0)*(10+ Config.ballRadius));
        Coordinate c2 = new Coordinate(cord2.getX(),cord2.getY(),cord2.getZ()+ ((liftCoord2)? 1 : 0)*(10+ Config.ballRadius));

        ArrayList<Coordinate> all = Coordinate.getPixelBetweenToPoints(c1, c2);
        for (int i = 1; i<all.size()-1; i++){
            Coordinate c = all.get(i);
            System.out.println(c.getX() + " q" + c.getY() + " " + c.getZ() + "Type" +playfield[(int)c.getX()][(int)c.getY()][(int)(c.getZ() )].name());
            Type t = playfield[(int)c.getX()][(int)c.getY()][(int)(c.getZ() )];
            if (t != ((Type.Empty))){
                System.out.println("Would colide with " + playfield[(int) c.getX()][(int) c.getY()][(int)(c.getZ())]);
                return false;

            }
        }
        return true;
    }


    public Coordinate getNormalQuick(int x, int y, int z) {
        if (surfaceNormalsInGood==null) calculateSurfaceNormalsMap();
        lookup.set(x, y, z);
        Coordinate c = surfaceNormalsInGood.get(lookup);
        if (c==null) return zeroInt;
        return c;
    }
}