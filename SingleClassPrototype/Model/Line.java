package Model;

/**
 * Created by Jyr on 12/6/2017.
 */
public class Line {

    private Node a, b;

    public Line(Node a, Node b) {
        this.a = a;
        this.b = b;
    }

    public Node getA() {
        return a;
    }

    public Node getB() {
        return b;
    }

    public String toString() {
        return "Line: [" + a + ", " + b + "]";
    }

    public void reverseNodes() {
        Node c = this.b;
        this.b = this.a;
        this.a = c;
    }
}
