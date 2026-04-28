public class Main {
    public static void main(String[] args) {

        Inventory inv = new Inventory();
        Payment pay = new Payment();
        Shipping ship = new Shipping();
        Email email = new Email();

        if (inv.checkStock("Laptop")) {
            if (pay.charge("alice@example.com", 999.99)) {
                inv.reserve("Laptop");

                String label = ship.createLabel("123 Main St");
                ship.schedulePickup(label);

                email.send("alice@example.com", "Order Confirmed", "Your order is on the way");

                System.out.println("Order success!");
            } else {
                System.out.println("Payment failed");
            }
        } else {
            System.out.println("Out of stock");
        }
    }
    
}
//class implementations
class Inventory {
    boolean checkStock(String item) {
        return true; // simulate in stock
    }

    void reserve(String item) {
        System.out.println("Item reserved: " + item);
    }
}

class Payment {
    boolean charge(String email, double amount) {
        System.out.println("Charging " + email);
        return true; // simulate success
    }
}

class Shipping {
    String createLabel(String address) {
        return "LABEL123";
    }

    void schedulePickup(String label) {
        System.out.println("Pickup scheduled: " + label);
    }
}

class Email {
    void send(String to, String subject, String message) {
        System.out.println("Email sent to " + to);
    }
}