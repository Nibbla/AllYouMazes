package SpecialSettingsEtc;

public class Tangential {
    private final static float[] sins = new float[2^8];
    private final static float[] coss = new float[2^8];

    public static float sin(byte theta) {
        return sins[theta];
    }

    public static float cos(byte phi) {
        return coss[phi];
    }

    public enum Direction {forward(1.,1.),backward(-1.,-1.),turnLeftForward(0.75,1.),turnRightForward(1.,0.75),turnLeftBackward(-0.75,-1),TurnRightBackward(-1.,-0.75),RotateLeft(-1.,1.),RotateRight(1.,-1.);


        public final double[] powerLevel;

        Direction(double v, double v1) {
            this.powerLevel = new double[]{v,v1};
        }
    }
}
