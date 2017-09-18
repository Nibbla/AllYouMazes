import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;
import java.util.Queue;
import java.util.Random;


/** This class defines the basic structure of our program
 * It is defined by the model view controller architecture.
 * View is responsible for processing the image stream and showing it on screen
 * Model is responsible for converting the visual to the internal data model
 * and it creates pathways
 * Control then converts the path order in orders fors the robot.
 */
public class Modul {

    public static final int BYTESIZE = (int) Math.pow(2, 8);
    private static Random rnd = new RandomCount();
    private final int baseUnit = 100;
    private static final int CUBE_UNIT = (int) Math.ceil(Math.pow(baseUnit, 1 / 3.));


    private static View factoryView = new View();
    private static Model factoryModel = new Model(rnd);
    private static Control factoryControl = new Control();

    public View view;
    public Model model;
    public Control control;

    public Modul(View view , Model model, Control control) {
        this.view = view;
        this.model = model;
        this.control = control;
    }


    public static void main(String[] args){

        try {
            Modul modul = new Modul(factoryView.getInstance(),factoryModel.getInstance(),factoryControl.getInstance());
            byte[] noiseStream = modul.model.quickSimpleAccessor.getNoiseStream();
        } catch (CloneNotSupportedException e) {
            e.printStackTrace();
        }



    }

    public static class View{

        public View() {

        }

        public View getInstance() throws CloneNotSupportedException {
            return (View) factoryView.clone();
        }

        public class Pixel{

        }

        public class Workflow{

        }


    }

    public static class Model{

        private final NoiseFloor noiseFloor;
        private final NoiseFloor.Accessor quickSimpleAccessor;



        public Model(Random r) {
            this.noiseFloor = new NoiseFloor(r);
            quickSimpleAccessor = noiseFloor.getAccessor();
        }

        public byte[][] getNoiseFloor(){
            return quickSimpleAccessor.getNoiseFloor();
        }

        public Model getInstance() throws CloneNotSupportedException {
            return (Model) factoryModel.clone();
        }



        public enum ObjectClasses{
            AGENTS,BACKGROUND,WALL
        }

        public abstract class Node{
            Node parent;
            List<Node> children;

            public int length(){
                return 0;
            }
        }

        public class typeNode extends Node{
            ObjectClasses type;

            @Override
            public int length() {
                return 0;
            }
        }

        public class Idol{

        }

        private class NoiseFloor {
            private final long[] seeds;
            Accessor accessor;

            private Random[] randoms;

            public NoiseFloor clone(){
                return new NoiseFloor(seeds);
            }

            private NoiseFloor(long[] seeds) {
                randoms = new Random[baseUnit];
                for (int i = 0; i < baseUnit; i++) {
                    randoms[i] = new Random(seeds[i]);
                }

                this.seeds = Arrays.copyOf(seeds, baseUnit);
            }

            public NoiseFloor(Random l) {
                randoms = new Random[baseUnit];
                seeds = new long[baseUnit];
                for (int i = 0; i < baseUnit; i++) {
                    long seed = l.nextLong();
                    seeds[i]= seed;
                    randoms[i] = new Random(seed);
                }
            }

            public Accessor getAccessor() {
                if (accessor == null) return quickSimpleAccessor;
                return accessor;
            }

            private class Accessor {

                public byte[] getNoiseStream(){
                    byte[] floor = new byte[baseUnit];
                    for (int i = 0; i < baseUnit; i++) {
                        floor[i] = (byte) randoms[i].nextInt(BYTESIZE);
                    }
                    return floor;
                }
                public byte[][] getNoiseFloor() {
                    int unit = (int) Math.ceil(Math.sqrt(baseUnit));
                    byte[][] floor = new byte[unit][unit];

                    for (int i = 0; i < baseUnit; i++) {
                        int x = i % unit;
                        int y = i / unit;
                        floor[x][y] = (byte) randoms[i].nextInt(BYTESIZE);
                    }

                    return floor;
                }

                public byte getNoiseFromCube(int x, int y, int z){
                    int unit = CUBE_UNIT;
                    int sliceSize = unit*unit;

                    int i=x+y*unit+z*sliceSize;
                    return (byte) randoms[i].nextInt(BYTESIZE);
                }

                public byte getNoiseFromCube(int i,byte[][][] noiseCube){
                    int unit = CUBE_UNIT;
                    int sliceSize = unit*unit;

                    int x = i % unit;
                    int y = (i / unit) % sliceSize;
                    int z = i / sliceSize;
                    return noiseCube[x][y][z];
                }

                public byte getNoiseFromCube(int x, int y, int z, byte[][][] noiseCube){
                    return noiseCube[x][y][z];
                }

                public byte[][][] getNoiseCube() {
                    int unit = CUBE_UNIT;
                    int sliceSize = unit*unit;
                    byte[][][] floor = new byte[unit][unit][unit];

                    for (int i = 0; i < baseUnit; i++) {
                        int x = i % unit;
                        int y = (i / unit) % sliceSize;
                        int z = i / sliceSize;
                        floor[x][y][z] = (byte) randoms[i].nextInt(BYTESIZE);
                    }

                    return floor;
                }

                public byte[] getNoiseStream(Path p) throws Exception {
                    byte[] stream = new byte[p.length()];
                    p.fill(stream);

                    return stream;
                }

                protected   class Path extends Node{
                    KartesianCoordinates start;
                    List<PolarCoordinates16> line;

                    private Path() {
                        start = null;
                    }

                    public View.Pixel[][] getPainting(){
                        return null;
                    }

                    public int length() {
                        if (parent != null) return parent.length();
                        return 0;
                    }

                    public void fill(byte[] stream) throws Exception {

                        int count = 0;
                        int lineSize = line.size();
                        KartesianCoordinates currentPossition = start.clone();

                        for (int i = 0; i < lineSize; i++) {
                            PolarCoordinates16 d = line.get(i);
                            KartesianCoordinates convert = d.toKartesianCoordinates();

                            KartesianCoordinates end = new KartesianCoordinates();
                            List<KartesianCoordinates> pixels = fordFulkerson(currentPossition, convert, d, end);
                            int pixelsSize = 0 + pixels.size();
                            for (int j = 0; j < pixelsSize; j++) {
                                KartesianCoordinates kp = pixels.get(j);
                                stream[j + count] = getNoiseFromCube((int) kp.x, (int) kp.y, (int) kp.z);
                            }
                            count += pixelsSize;
                            currentPossition = end;
                        }
                        if (length() != count){
                            throw new Exception("fillCountIncorrect\ncount: " + count + "\nlength:" + length());
                        }
                    }





                }

                private List<KartesianCoordinates> fordFulkerson(KartesianCoordinates start, KartesianCoordinates convert, PolarCoordinates16 d, KartesianCoordinates end) {
                    return null;
                }

                private class KartesianCoordinates {
                    float x;
                    float y;
                    float z;

                    public KartesianCoordinates clone() {
                        KartesianCoordinates kc = new KartesianCoordinates();
                        kc.x = x;
                        kc.y = y;
                        kc.z = z;
                        return kc;
                    }
                }

                private class PolarCoordinates16 {
                    byte phi;
                    byte theta;
                    float r;

                    public KartesianCoordinates toKartesianCoordinates() {
                        KartesianCoordinates k = new KartesianCoordinates();
                        k.x = Tangential.sin(theta) * Tangential.cos(phi);
                        k.y = r *Tangential.sin(phi)*Tangential.sin(theta);
                        k.z = r *  Tangential.cos(phi);
                        return k;
                    }
                }
            }

            public class AccessorNode extends Node{
            }
        }


    }

    private static class Control {

        public Control getInstance() throws CloneNotSupportedException {
            return (Control) factoryControl.clone();
        }
    }

    /**
     * extendes random so the time it is executed is measured
     */
    private static class RandomCount extends Random {
        private static long rndCount = 0l;

        @Override
        public double nextDouble() {
            rndCount++;
            return super.nextDouble();
        }

        @Override
        public long nextLong() {
            rndCount++;
            return super.nextLong();
        }

        @Override
        public boolean nextBoolean() {
            rndCount++;
            return super.nextBoolean();
        }

        @Override
        public int nextInt() {
            rndCount++;
            return super.nextInt();
        }

        @Override
        public int nextInt(int bound) {
            rndCount++;
            return super.nextInt(bound);
        }
    }

    private static class Tangential {
        private final static float[] sins = new float[BYTESIZE];
        private final static float[] coss = new float[BYTESIZE];

        public static float sin(byte theta) {
            return sins[theta];
        }

        public static float cos(byte phi) {
            return coss[phi];
        }
    }
}
