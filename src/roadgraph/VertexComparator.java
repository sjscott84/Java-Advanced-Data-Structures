package roadgraph;
import java.util.Comparator;

public class VertexComparator implements Comparator<Vertex> {

    @Override
    public int compare(Vertex n1, Vertex n2) {
        if (n1.getDistance() > n2.getDistance()) {
            return 1;
        } else {
            return -1;
        }
    }
}
