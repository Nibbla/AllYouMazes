package view.classifier;

/**
 * Created by Nibbla on 09.12.2017.
 */
public enum ColorChannel {
    R("R"),G("G"),B("B"),RG("R/G"),RB("R/B"),GB("G/B"),Hue("Hue"),Sat("Sat"),Brg("Brg");

    private final String text;

    ColorChannel(String r) {
        this.text = r;
    }

    public String getText() {
        return text;
    }
}