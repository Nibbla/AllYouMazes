package Model;

import view.Pixel;

import java.util.List;

public class PathDeprecated {
    KartesianCoordinates start;
    List<PolarCoordinates> line;

    private PathDeprecated() {
        start = null;
    }

    public Pixel[][] getPainting(){
        return null;
    }

    public int length() {
      //  if (parent != null) return parent.length();
        return 0;
    }

    public void fill(byte[] stream) throws Exception {

        int count = 0;
        int lineSize = line.size();
        KartesianCoordinates currentPossition = start.clone();

        for (int i = 0; i < lineSize; i++) {
            PolarCoordinates d = line.get(i);
            KartesianCoordinates convert = d.toKartesianCoordinates();

            KartesianCoordinates end = new KartesianCoordinates();
            List<KartesianCoordinates> pixels = fordFulkerson(currentPossition, convert, d, end);
            int pixelsSize = 0 + pixels.size();
            for (int j = 0; j < pixelsSize; j++) {
                KartesianCoordinates kp = pixels.get(j);
                //stream[j + count] = getNoiseFromCube((int) kp.x, (int) kp.y, (int) kp.z);
            }
            count += pixelsSize;
            currentPossition = end;
        }
        if (length() != count){
            throw new Exception("fillCountIncorrect\ncount: " + count + "\nlength:" + length());
        }
    }

    private List<KartesianCoordinates> fordFulkerson(KartesianCoordinates currentPossition, KartesianCoordinates convert, PolarCoordinates d, KartesianCoordinates end) {
        return null;
    }


}