// ============================================================
// QueueADT.java  –  The ADT contract (interface)
// ============================================================
public interface QueueADT {
    void enqueue(int element);
    int  dequeue();
    boolean isEmpty();
    int  size();
}
