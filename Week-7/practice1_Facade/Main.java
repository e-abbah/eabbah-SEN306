public class Main {
    public static void main(String[] args) {

        OrderFacade facade = new OrderFacade();

        boolean success = facade.placeOrder(
                "alice@example.com",
                "Laptop",
                999.99,
                "123 Main St"
        );

        System.out.println(success ? "Order success!" : "Order failed");
    }
}



class OrderFacade {
    private Inventory inventory;
    private Payment payment;
    private Shipping shipping;
    private Email email;

    public OrderFacade() {
        this.inventory = new Inventory();
        this.payment = new Payment();
        this.shipping = new Shipping();
        this.email = new Email();
    }

    public boolean placeOrder(String userId, String productId,
                              double price, String address) {
        if (!inventory.checkStock(productId)) return false;
        if (!payment.charge(userId, price)) return false;

        inventory.reserve(productId);
        String label = shipping.createLabel(address);
        shipping.schedulePickup(label);
        email.send(userId, "Order Confirmed", "On its way!");
        return true;
    }
}

// ======================= SUBSYSTEMS =======================

class Inventory {
    public boolean checkStock(String productId) {
        return true;
    }

    public void reserve(String productId) {
        System.out.println("Inventory reserved: " + productId);
    }
}

class Payment {
    public boolean charge(String userId, double price) {
        System.out.println("Payment processed for " + userId);
        return true;
    }
}

class Shipping {
    public String createLabel(String address) {
        return "LABEL-" + address;
    }

    public void schedulePickup(String label) {
        System.out.println("Pickup scheduled with: " + label);
    }
}

class Email {
    public void send(String userId, String subject, String message) {
        System.out.println("Email sent to " + userId + ": " + subject);
    }
}