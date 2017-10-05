package Magic.golf;

import Game.Actors.Player;
import Game.Config;
import org.apache.commons.math3.distribution.NormalDistribution;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Created by tony and Markus on 16/03/2016.
 */
public class PhysicsEngine {
    private Course course;

    private ArrayList<Ball> balls = new ArrayList<>(2);
    private Coordinate wind;

    public PhysicsEngine getAlternativBoardForTest(){
        return clone();
    }



    public void calculateUntilNoBallIsMoving(double noisePercentage, double windPercentage){
        do {
            processPhysics(Config.STEPSIZE, noisePercentage,windPercentage);

        }while(atLeastOneBallMoving());
    }

    public boolean atLeastOneBallMoving() {

        for (int i = 0; i < balls.size(); i++) {
           if (balls.get(i).isMoving()) return true;
        }
        return false;
    }

    public PhysicsEngine clone(){
        PhysicsEngine p = new PhysicsEngine();
        p.course = course;
        p.wind = wind.clone();

        p.balls = new ArrayList<>();
        for (Ball b:balls){
            p.balls.add(b.clone());
        }
        return p;
    }

    public ArrayList<Ball> getBalls(){
        return balls;
    }

    public Ball getBallOfPlayer(Player p){
        for(Ball b:balls){
            if (b.getPlayer()==p) return b;
        }
        return null;
    }
    public String getBallOfBot(){
        return getBalls().get(0).getCoordinate().toString();
    }

    public int getBallIndexOfPlayer(Player p){
        for (int i = 0; i < balls.size(); i++) {
            if (balls.get(i).getPlayer()==p) return i;
        }
        return -1;
    }

    public void init(Course course, ArrayList<Ball> balls, Coordinate wind) {
        this.course = course;
        this.balls.clear();
        this.balls = balls;
        this.wind = wind;
    }

    public void init(ArrayList<Player> players, Course course) {
        if (wind == null) wind = new Wind(0,0,0,0);
        this.course = course;
        this.balls.clear();

        for (int i = 0; i < players.size(); i++) {
            balls.add(players.get(i).getBall());
        }
    }

    public void init(ArrayList<Player> players, Course course, Coordinate wind) {
        this.course = course;
        this.balls.clear();
        this.wind = wind;
        for (int i = 0; i < players.size(); i++) {
            balls.add(players.get(i).getBall());
        }

    }
    private double getNoise(){

        double noise= Double.parseDouble(null);
        NormalDistribution normalDistribution = new NormalDistribution();

        return noise;
    }

    public void processPhysics(double elapsedTime, double noisePercentage, double windPercentagePerSecond) {


        Type[][][] playfield = course.getPlayfield();
        HashMap<IntCoordinate, Coordinate> surfaceHashMap = course.surfaceNormalsInGood;
        Coordinate[][] normals = course.surfaceNormals;
        for (int i = 0; i < balls.size(); i++) {

            Ball b = balls.get(i);
            if (!b.inPlay||b.isPregame()) continue;
            gravity(b);

            hover(b,elapsedTime,playfield,course.getDimension(),surfaceHashMap,normals);
            collide(b,elapsedTime,playfield,course.getDimension(),noisePercentage,surfaceHashMap,normals);
            drag(b);
            checkborder(b);
            accelerate(b,elapsedTime);
            resetA(b);


            //check for Collision with different balls
            for (int j = i+1; j < balls.size(); j++) {
                //check for collision between two balls
                Ball bd = balls.get(j);
                if (bd.inHole||!bd.inPlay||bd.isPregame()) continue;
               ballCollision(b,bd);
            }

            inertia(b,elapsedTime);
            checkIfInHole(b);
            adjustWind(windPercentagePerSecond*Config.STEPSIZE);
           // setUp(b);

        }

        for (int i = 0; i < balls.size(); i++) {
            balls.get(i).checkBallStopped();

        }


    }

    private void adjustWind(double windAdjustPercentage) {
        if (windAdjustPercentage<=0) return;
        Coordinate.modify2d(wind,windAdjustPercentage);
    }

    private void drag(Ball b) {
        double air= Config.AIR_FRICTION;

        double speed = b.getSpeed();

        if (speed!=0) {
            double drag = air * (speed * speed);

            double nx = drag * b.getSpeedX() / speed;
            double ny = drag * b.getSpeedY() / speed;
            double nz = drag * b.getSpeedZ() / speed;


            b.aX -= nx;
            b.aY -= ny;
            b.aZ -= nz;
        }
    }

    private void setUp(Ball b) {
        int upthing = 0;
        if (b.z-b.radius>=course.getDimension()[2])return;
        while (course.getTile((int)b.x,(int)b.y,(int)(b.z-b.radius))!=Type.Empty){
            b.z=b.z+1.1;
            b.previousZ+=1.1;;
        }


    }

    private void checkIfInHole(Ball b) {
        Hole h = course.getHole();
        double dx =b.x-h.x;
        double dy =b.y-h.y;
        double dz =b.z-h.z;

        if ((h.isBallIntersectingHole(b))&&b.getSpeed()<0.2){
            b.setInHole(true);
            b.isMoving=false;

            //System.out.println(b.getPlayer().getName() + " put the ball into the hole");
        }


    }
    private void checkborder(Ball b){
        int height = Game.course.getHeight();
        int width = Game.course.getWidth();
        int depth = Game.course.getDepth();

        double dx = Math.abs(b.getX()-b.getPreviousX());
        double dy = Math.abs(b.getY()-b.getPreviousY());
        double dz = Math.abs(b.getZ()-b.getPreviousZ());

        if(b.getZ()+b.getRadius()<= 0){
           // System.out.println("out of Z border: ");

            b.z = 1+b.getRadius();
            b.previousZ=b.z-dz;
            //b.printBallInfo();

        }

        if(b.getX()+b.getRadius()>width-1){
            //System.out.println("out of X border: ");

            b.x = width-1-b.getRadius();
            b.previousX=b.x+dx;
            //b.printBallInfo();
        }

        if(b.getX()-b.getRadius()<=0){
            //System.out.println("out of X border: ");
            b.x = b.getRadius()+1;
            b.previousX=b.x-dx;
           // b.printBallInfo();
        }

        if(b.getY()+b.getRadius()>height-1){
            //System.out.println("out of Y border: ");
            b.y = height-1-b.getRadius();
            b.previousY=b.y+dy;
            //b.printBallInfo();
        }

        if(b.getY()-b.getRadius()<=0){
           // System.out.println("out of Y border: " );
            b.y = 1+b.getRadius();
            b.previousY=b.y-dy;

        }



    }

    private void collide(Ball b, double elapsedTime, Type[][][] playfield, int[] dimension, double noisePercentage, HashMap<IntCoordinate, Coordinate> surfaceHashMap, Coordinate[][] normals) {
        double normalX = 0;
        double normalY = 0;
        double normalZ = 0;
        int count = 0;
        double g = 1* Config.GRAVITY_FORCE;
        double BounceFriction = 0;
        int[][] surfaceBall = b.getSurfacePoints();
        int l = surfaceBall.length;

        double vOld = b.getSpeed();

        for (int i = 0; i < l; i++) {
            int x = surfaceBall[i][0];
            int y = surfaceBall[i][1];
            int z = surfaceBall[i][2];
            boolean skipCheck = false;
            if (y<0||x<0||z<0||x>=dimension[0]||y>=dimension[1]||z>=dimension[2]) {
                skipCheck = true;
                if (x < 0) {
                    //normalX += 1;

                }

                if (y < 0) {
                    //normalY += 1;

                }
                if (z < 0) {
                   // normalZ += 1;

                }

                if (x >= dimension[0]) {
                   // normalX -= 1;

                }
                if (y >= dimension[1]) {
                   // normalY -= 1;

                }
                if (z >= dimension[2]) {
                    //aZ -= 1;
                   // count--;
                    //count++;
                }
               // BounceFriction+=Type.Grass.getBounceDampness();
                skipCheck = true;
               // count++;
            }

            if (!skipCheck){
                Type t = playfield[x][y][z];
                if (t!= Type.Empty) {
                    BounceFriction += t.getBounceDampness();
                    Coordinate c;
                    if (noisePercentage == 0)  c=course.getNormalQuick(x, y, z);//c  = normals[x][y];//
                    else{
                        c =  course.getNormalQuick(x, y, z).clone();;
                        Coordinate.modify3d(c,noisePercentage);
                        c.normalise();

                    }


                    if (!c.isNullVector()){
                        normalX += c.getX();
                        normalY += c.getY();
                        normalZ += c.getZ();
                        count++;

                    }
                }

            }
        }

        if (count!=0) {
            normalX /= count;
            normalY /= count;
            normalZ /= count;
            BounceFriction /= count;
            //addedFriction = (1-addedFriction*elapsedTime);
            double dx = b.x - b.previousX;
            double dy = b.y - b.previousY;
            double dz = b.z - b.previousZ;

            // Project velocity onto the normal, multiply by 2, and subtract it from velocity

            // project velocity onto the normal using dot product
            double scalarProjection = dx * normalX + dy * normalY + dz * normalZ;
            if (scalarProjection<0) {


                double dxNew = dx - normalX * scalarProjection * 2;
                double dyNew = dy - normalY * scalarProjection * 2;
                double dzNew = dz - normalZ * scalarProjection * 2;
                // if (Math.sqrt(dxNew*dxNew+dyNew*dyNew+dzNew*dzNew)< Math.sqrt(dx*dx+dy*dy+dz*dz)) {
                BounceFriction = 0;
                b.previousX = b.x - dxNew * (1 - BounceFriction * elapsedTime);
                b.previousY = b.y - dyNew * (1 - BounceFriction * elapsedTime);
                b.previousZ = b.z - dzNew * (1 - BounceFriction * elapsedTime);
            }
            //System.out.println("woop");
            // }
            // }
            //friction should come in here


        }

    }

    private void hover(Ball b, double elapsedTime, Type[][][] playfield, int[] dimension, HashMap<IntCoordinate, Coordinate> surfaceHashMap, Coordinate[][] normals) {
        double aX = 0;
        double aY = 0;
        double aZ = 0;
        int count = 0;

        double g = 1* Config.GRAVITY_FORCE;
        double addedFriction = 0;
        int[][] surfaceBall = b.getSurfacePointsBig();
        int l = surfaceBall.length;
        for (int i = 0; i < l; i++) {
            int x = surfaceBall[i][0];
            int y = surfaceBall[i][1];
            int z = surfaceBall[i][2];
            boolean skipCheck = false;
            if (y<0||x<0||z<0||x>=dimension[0]||y>=dimension[1]||z>=dimension[2]) {
                skipCheck = true;
                /*if (x < 0) {
                    aX += 1;
                }

                if (y < 0) {
                    aY += 1;

                }
                if (z < 0) {
                    aZ += Config.UPPush;

                }

                if (x >= dimension[0]) {
                    aX -= 1;

                }
                if (y >= dimension[1]) {
                    aY -= 1;

                }
                if (z >= dimension[2]) {
                    //aZ -= 1;
                    count--;
                    //count++;
                }*/
                //addedFriction+=Type.Grass.getFriction();
                skipCheck = true;
                //count++;

            }

            if (!skipCheck){
                Type t = playfield[x][y][z];
                 if (t!= Type.Empty) {


                     Coordinate c;
                     c = course.getNormalQuick(x, y, z);
                     //c = normals[x][y];
                     if (!c.isNullVector()) {
                         addedFriction+=t.getFriction();
                        // System.out.println(c.toString());
                         aX += c.getX();
                         aY += c.getY();
                         aZ += c.getZ();
                         count++;
                     }
                 }else{

                 }

            }
        }

       if (count!=0){

           aX/=count;
           aY/=count;
           aZ/=count;
           addedFriction/=count;


           //Vector3 planeOrigin;
           //Vector3 planeNormal;
           //Vector3 point;

           //Vector3 v = point - planeOrigin;
           //Vector3 d = Vector3.Project(v, planeNormal.normalized);
           //Vector3 projectedPoint = point - d;

           double dx = b.previousX - b.x;
           double dy = b.previousY - b.y;
           double dz = b.previousZ - b.z;
           addedFriction = (1-addedFriction*elapsedTime);
           b.previousX = b.x +  dx*addedFriction;
           b.previousY = b.y +  dy*addedFriction;
           b.previousZ = b.z +  dz*addedFriction;

           b.aX+=aX* Config.UPPush*g;
           b.aY+=aY* Config.UPPush*g;
           b.aZ+=aZ* Config.UPPush*g;


       }else{
           //wind force
           b.aX+= wind.getX();
           b.aY+= wind.getY();
           b.aZ+= wind.getZ();
       }



    }

    private void accelerate(Ball b, double elapsedTime) {
        double e = elapsedTime*elapsedTime;
        b.x+=b.aX*e;
        b.y+=b.aY*e;
        b.z+=b.aZ*e;

    }


    private void ballCollision(Ball b, Ball bd) {

        double dx = b.getX()-bd.getX();
        double dy = b.getY()-bd.getY();
        double dz = b.getZ()-bd.getZ();

        double slength = dx*dx+dy*dy+dz*dz;
        double length = Math.sqrt(slength);
        double target = b.getRadius()+bd.getRadius();

        //if spheres are closer then radii combined
        if(length<target){
            // record previous velocity
            double v1x = b.x - b.previousX;
            double v1y = b.y - b.previousY;
            double v1z = b.z - b.previousZ;

            double v2x = bd.x - bd.previousX;
            double v2y = bd.y - bd.previousY;
            double v2z = bd.z - bd.previousZ;

            // resolve the body overlap conflict
            double factor = (length-target)/length;
            b.x -= dx*factor*0.5;
            b.y -= dy*factor*0.5;
            b.z -= dy*factor*0.5;

            bd.x += dx*factor*0.5;
            bd.y += dy*factor*0.5;
            bd.z += dz*factor*0.5;

            // compute the projected component factors
            double f1 = (Config.DAMPING*(dx*v1x+dy*v1y+dz*v1z))/slength;
            double f2 = (Config.DAMPING*(dx*v2x+dy*v2y+dz*v2z))/slength;


            // swap the projected components
            v1x += f2*dx-f1*dx;
            v2x += f1*dx-f2*dx;
            v1y += f2*dy-f1*dy;
            v2y += f1*dy-f2*dy;
            v1z += f2*dy-f1*dy;
            v2z += f1*dz-f2*dz;

            // the previous position is adjusted
            // to represent the new velocity
            b.previousX = b.x - v1x;
            b.previousY = b.y - v1y;
            b.previousZ = b.z - v1z;

            bd.previousX = bd.x - v2x;
            bd.previousY = bd.y - v2y;
            bd.previousZ = bd.z - v2z;
        }

    }

    private void inertia(Ball b, double elapsedTime) {
        double x = b.getX()*2-b.getPreviousX();
        double y = b.getY()*2-b.getPreviousY();
        double z = b.getZ()*2-b.getPreviousZ();
        b.previousX = b.x;
        b.previousY = b.y;
        b.previousZ = b.z;
        b.x = x;
        b.y = y;
        b.z = z;

    }

    private void resetA(Ball b) {
        b.setaZ(0);
        b.setaX(0);
        b.setaY(0);
    }

    private void gravity(Ball b) {
            b.setaZ(-Config.GRAVITY_FORCE);
    }


    public double calculateAngle(double speedX, double speedY) {
        double angle;
        angle = Math.atan(speedY / speedX);
        return angle;
    }


    public Coordinate getWind() {
        return wind;
    }
}
