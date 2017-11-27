package view;

import Model.ComputerVision;
import SpecialSettingsEtc.Settings;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import org.opencv.core.Mat;

/**
 * Created by on 28-11-2017.
 */
public class Controller {

    @FXML
    private Button piLoop;

    @FXML
    private Button contourTest;

    public void initialize() {
        piLoop.setOnAction(ae -> {
            ComputerVision.bgsLoop();
        });

        //static img for now
        contourTest.setOnAction(ae -> {
            ComputerVision.CONTOUR_TEST = true;
            Mat rImg = ComputerVision.resize(ComputerVision.readImg(Settings.getDefaultInputPath() + "latestScreen.jpg"));
            Mat gray = ComputerVision.grayScale(rImg);
            ComputerVision.retrieveContour(gray, null);
            ComputerVision.contourv2(rImg);
            ComputerVision.CONTOUR_TEST = false;
        });
    }

}
