package Model;

import Interfaces.IModel;

import java.awt.*;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

public class Model  implements IModel {

    private final NoiseFloor noiseFloor;
    private final NoiseFloor.Accessor quickSimpleAccessor;
    public static final int BYTESIZE = (int) Math.pow(2, 8);

    private static final int baseUnit = 100;
    private static final int CUBE_UNIT = (int) Math.ceil(Math.pow(baseUnit, 1 / 3.));
    private static Model factoryModel = new Model(new Random(0));
    private Point robotPosition;


    public Model(Random r) {
        this.noiseFloor = new NoiseFloor(r);
        quickSimpleAccessor = noiseFloor.getAccessor();
    }

    public byte[][] getNoiseFloor() {
        return quickSimpleAccessor.getNoiseFloor();
    }

    public Model getInstance() throws CloneNotSupportedException {
        return (Model) factoryModel.clone();
    }

    public Point getRobotPosition() {
        return robotPosition;
    }


    public enum ObjectClasses {
        AGENTS, BACKGROUND, WALL
    }


    public class typeNode  {
        ObjectClasses type;


        public int length() {
            return 0;
        }
    }

    public class Idol {

    }

    private class NoiseFloor {
        private final long[] seeds;
        Accessor accessor;

        private Random[] randoms;

        public NoiseFloor clone() {
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
                seeds[i] = seed;
                randoms[i] = new Random(seed);
            }
        }

        public Accessor getAccessor() {
            if (accessor == null) return quickSimpleAccessor;
            return accessor;
        }

        private class Accessor {

            public byte[] getNoiseStream() {
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

            public byte getNoiseFromCube(int x, int y, int z) {
                int unit = CUBE_UNIT;
                int sliceSize = unit * unit;

                int i = x + y * unit + z * sliceSize;
                return (byte) randoms[i].nextInt(BYTESIZE);
            }

            public byte getNoiseFromCube(int i, byte[][][] noiseCube) {
                int unit = CUBE_UNIT;
                int sliceSize = unit * unit;

                int x = i % unit;
                int y = (i / unit) % sliceSize;
                int z = i / sliceSize;
                return noiseCube[x][y][z];
            }

            public byte getNoiseFromCube(int x, int y, int z, byte[][][] noiseCube) {
                return noiseCube[x][y][z];
            }

            public byte[][][] getNoiseCube() {
                int unit = CUBE_UNIT;
                int sliceSize = unit * unit;
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


            private List<KartesianCoordinates> fordFulkerson(KartesianCoordinates start, KartesianCoordinates convert, PolarCoordinates d, KartesianCoordinates end) {
                return null;
            }


        }


    }
}