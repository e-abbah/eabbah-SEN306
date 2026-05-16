// ============================================================
// ArrayListQueue.java  –  Implementation 1: backed by ArrayList
// ============================================================
import java.util.ArrayList;
import java.util.List;

public class ArrayListQueue implements QueueADT {

    // private – information hiding: client cannot touch this
    private List<Integer> list = new ArrayList<>();

    @Override
    public void enqueue(int element) {
        list.add(element);          // add to back
    }

    @Override
    public int dequeue() {
        if (isEmpty()) throw new RuntimeException("Queue is empty");
        return list.remove(0);      // remove from front
    }

    @Override
    public boolean isEmpty() {
        return list.isEmpty();
    }

    @Override
    public int size() {
        return list.size();
    }
}
