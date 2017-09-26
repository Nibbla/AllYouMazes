import Interfaces.IControl;
import Interfaces.IView;

import java.util.Arrays;
import java.util.List;
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

    public static class View implements IView {

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



        public class typeNode extends Node{
            ObjectClasses type;


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

                public byte[] getNoiseStream(Model.Path p) throws Exception {
                    byte[] stream = new byte[p.length()];
                    p.fill(stream);

                    return stream;
                }



                private List<Model.KartesianCoordinates> fordFulkerson(Model.KartesianCoordinates start, KartesianCoordinates convert, PolarCoordinates16 d, KartesianCoordinates end) {
                    return null;
                }




            }

            public class AccessorNode extends Node{
            }
        }


    }

    private static class Control implements IControl {

        public Control getInstance() throws CloneNotSupportedException {
            return (Control) factoryControl.clone();
        }

        @Override
        public boolean setPath(Model.NoiseFloor.Accessor.Path pathway) {
            return false;
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


}
