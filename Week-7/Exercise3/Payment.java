public class Payment {

    public boolean charge(String customerEmail, double amount) {
        System.out.println("Payment processed: " + amount);
        return true;
    }
}