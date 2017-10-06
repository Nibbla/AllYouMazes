package SpecialSettingsEtc;

import java.util.LinkedList;
import java.util.Queue;

/**
 * Created by Nibbla on 06.10.2017.
 */
public class Archivar {
    static boolean printout = true;
    private static Queue<String> archive = new LinkedList<>();
    private static boolean store = true;

    public static void shout(String s) {
        if (printout) System.out.println(s);

        if (store) archive.add(s);
    }

    public static void recall() {
        String latest = archive.poll();
        while (latest!=null){
            System.out.println(latest);
            latest = archive.poll();
        }
    }

    public static void liveOutput(boolean b) {
        printout = b;
    }

    public static void setStoreShouts(boolean b) {
        store = b;
    }
}
