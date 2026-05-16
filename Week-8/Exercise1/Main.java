// ============================================================
// Main.java  –  Exercise 1: Queue ADT client code
// Swap ArrayListQueue <-> LinkedQueue without changing anything else
// ============================================================
public class Main {

    public static void main(String[] args) {

        System.out.println("=== Exercise 1: Queue ADT ===\n");

        // Change this one line to switch implementations – nothing else changes
        QueueADT queue = new LinkedQueue();   // try: new ArrayListQueue()

        queue.enqueue(10);
        queue.enqueue(20);
        queue.enqueue(30);

        System.out.println("Size     : " + queue.size());        // 3
        System.out.println("Dequeue  : " + queue.dequeue());     // 10 (FIFO)
        System.out.println("Dequeue  : " + queue.dequeue());     // 20
        System.out.println("isEmpty  : " + queue.isEmpty());     // false
        System.out.println("Dequeue  : " + queue.dequeue());     // 30
        System.out.println("isEmpty  : " + queue.isEmpty());     // true
    }
}
