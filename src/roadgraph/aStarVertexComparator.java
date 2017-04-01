package roadgraph;
import java.util.Comparator;

public class aStarVertexComparator implements Comparator<Vertex> {

    @Override
    public int compare(Vertex n1, Vertex n2) {
        if (n1.getEstDistance() > n2.getEstDistance()) {
            return 1;
        } else {
            return -1;
        }
    }
}
