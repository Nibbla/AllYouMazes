package Magic.golf;

import Game.Model.Type;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.*;

/**
 * Created by nibbla on 14.03.16.
 */
public class Utils {

    static String readFile(String path){
        BufferedReader br = null;
        try {
            br = new BufferedReader(new FileReader(path));
        } catch (FileNotFoundException e) {

            e.printStackTrace();
            return null;
        }
        String everything = "";
        try {
            StringBuilder sb = new StringBuilder();
            String line = br.readLine();

            while (line != null) {
                sb.append(line);
                sb.append(System.lineSeparator());
                line = br.readLine();
            }
            everything = sb.toString();
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            try {
               if (br!=null) br.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        return everything;
    }

    static void saveFile(String path, String content){
        BufferedWriter writer = null;
        try
        {
            writer = new BufferedWriter( new FileWriter(path));
            writer.write(content);

        }
        catch ( IOException e)
        {
            e.printStackTrace();
        }
        finally
        {
            try
            {
                if ( writer != null)
                    writer.close( );
            }
            catch ( IOException e)
            {
            }
        }
    }



    public static void saveManagedBufferedImage(String path, BufferedImage image) {
        try {

            File outputfile = new File(path);
            ImageIO.write(image, "png", outputfile);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void saveCourse2_5D(Course course) {
        BufferedWriter writer = null;
        try
        {
            writer = new BufferedWriter( new FileWriter(Config.CourseLocation +course.name+".gol"));

            int length = course.playfield.length;
            int width = course.playfield[0].length;
            int height = course.playfield[0][0].length;
            StringBuilder s = new StringBuilder(50);
            s.append("Name:").append(course.name).append("\n");
            s.append("Version:").append(Config.version).append("\n");
            s.append("Par:").append(course.par).append("\n");
            s.append("length:").append(length).append("\n");
            s.append("width:").append(width).append("\n");
            s.append("height:").append(height).append("\n");
            writer.write(s.toString());
            int[][] hm = course.heightMap;
            StringBuffer b = new StringBuffer(30);
            for (int x = 0; x < length; x++) {
                for (int y = 0; y < width; y++) {


                    b.setLength(0);
                    b.append(x).append(";").append(y).append(";").append(hm[x][y]).append(";").append(course.playfield[x][y][hm[x][y]].ordinal()).append("\n");
                    writer.write(b.toString());

                }
            }


        }
        catch ( IOException e)
        {
            e.printStackTrace();
        }
        finally
        {
            try
            {
                if ( writer != null)
                    writer.close( );
            }
            catch ( IOException e)
            {
            }
        }



        if (course.managedBufferedImage==null){//(true){//
            course.managedBufferedImage = DrawPanel.createImage(course);
        }
        Utils.saveManagedBufferedImage(Config.CourseLocation + course.name + ".png",course.managedBufferedImage);


    }


    public static void saveCourse(Course course) {
        BufferedWriter writer = null;
        try
        {
            writer = new BufferedWriter( new FileWriter(Config.CourseLocation +course.name+".gol"));

            int length = course.playfield.length;
            int width = course.playfield[0].length;
            int height = course.playfield[0][0].length;
            StringBuilder s = new StringBuilder(50);
            s.append("Name:").append(course.name).append("\n");
            s.append("Version:").append(Config.version).append("\n");
            s.append("Par:").append(course.par).append("\n");
            s.append("length:").append(length).append("\n");
            s.append("width:").append(width).append("\n");
            s.append("height:").append(height).append("\n");
            writer.write(s.toString());

            for (int x = 0; x < length; x++) {
                for (int y = 0; y < width; y++) {

                    for (int z = 0; z < height; z++) {
                        if (!course.playfield[x][y][z].equals(Type.Empty)) {
                            StringBuilder b = new StringBuilder(10);
                            b.append("x").append(x).append("y").append(y).append("z").append(z).append("T").append(course.playfield[x][y][z].ordinal()).append("$\n");
                            writer.write(b.toString());
                        }
                    }
                }
            }

        }
        catch ( IOException e)
        {
            e.printStackTrace();
        }
        finally
        {
            try
            {
                if ( writer != null)
                    writer.close( );
            }
            catch ( IOException e)
            {
            }
        }



        if (course.managedBufferedImage==null){//(true){//
            course.managedBufferedImage = DrawPanel.createImage(course);
        }
        Utils.saveManagedBufferedImage(Config.CourseLocation + course.name + ".png",course.managedBufferedImage);


    }
}
