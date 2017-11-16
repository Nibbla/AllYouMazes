package SpecialSettingsEtc;

public class Tangential {
    private final static float[] sins = new float[2 ^ 8];
    private final static float[] coss = new float[2 ^ 8];

    public static float sin(byte theta) {
        return sins[theta];
    }

    public static float cos(byte phi) {
        return coss[phi];
    }

    public enum Direction {
        forward(1., 0), backward(-1., 0), turnLeftForward(1., 1.), turnRightForward(1., -1.), turnLeftBackward(-1, 1), TurnRightBackward(-1., -1), RotateLeft(0, 1.), RotateRight(0, -1.);


        public final double linearSpeed, angularSpeed;

        Direction(double v, double v1) {
            this.linearSpeed = v;
            this.angularSpeed = v1;
        }
    }
}

