package view;/**
 * Created by on 28-11-2017.
 */

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.io.File;
import java.io.IOException;

public class HackyTackyGUI extends Application {

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage stage) {
        Parent root = null;
        try {
            root = FXMLLoader.load(new File(("SingleClassPrototype" + File.separator + "view" + File.separator + "scene.fxml")).toURI().toURL());
        } catch (IOException e) {
            e.printStackTrace();
        }

        Scene scene = new Scene(root, 150, 275);

        stage.setTitle(":)");
        stage.setScene(scene);
        stage.show();
    }
}
