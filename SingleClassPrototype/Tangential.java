public class Tangential {
    private final static float[] sins = new float[2^8];
    private final static float[] coss = new float[2^8];

    public static float sin(byte theta) {
        return sins[theta];
    }

    public static float cos(byte phi) {
        return coss[phi];
    }
}
