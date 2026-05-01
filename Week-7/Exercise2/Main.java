import java.time.LocalDateTime;



public class Main {
    public static void main(String[] args) {

        BookingFacade facade = new BookingFacade();

        boolean success = facade.bookRoom("Johnny", "Deluxe", 200.0);

        System.out.println(success ? "Booking successful!" : "Booking failed");
    }
}


class BookingFacade {
    private RoomService rooms;
    private PaymentService payment;
    private LoyaltyPoints loyalty;
    private EmailService email;

    // NEW SUBSYSTEMS
    private TaxCalculator taxCalculator;
    private Logger logger;

    public BookingFacade() {
        this.rooms = new RoomService();
        this.payment = new PaymentService();
        this.loyalty = new LoyaltyPoints();
        this.email = new EmailService();

        // Initialize new subsystems
        this.taxCalculator = new TaxCalculator();
        this.logger = new Logger();
    }

    public boolean bookRoom(String guest, String roomType, double price) {

        
        if (!rooms.isAvailable(roomType)) {
            logger.log(guest, false);
            return false;
        }

    
        String state = "CA"; 
        double tax = taxCalculator.calculateTax(state, price);
        double totalPrice = price + tax;

        
        if (!payment.charge(guest, totalPrice)) {
            logger.log(guest, false);
            return false;
        }

        
        rooms.book(roomType, guest);
        loyalty.addPoints(guest, (int) totalPrice);

        
        email.sendConfirmation(guest, roomType + " | Total Paid: $" + totalPrice);

        
        logger.log(guest, true);

        return true;
    }
}


class TaxCalculator {
    public double calculateTax(String state, double amount) {
        if ("CA".equals(state)) {
            return amount * 0.08;
        }
        return 0;
    }
}

class Logger {
    public void log(String userId, boolean success) {
        System.out.println(
            "[" + LocalDateTime.now() + "] User: " + userId +
            " | Status: " + (success ? "SUCCESS" : "FAIL")
        );
    }
}



class RoomService {
    public boolean isAvailable(String roomType) {
        return true;
    }

    public void book(String roomType, String guest) {
        System.out.println(roomType + " booked for " + guest);
    }
}

class PaymentService {
    public boolean charge(String guest, double price) {
        System.out.println("Charged " + guest + " $" + price);
        return true;
    }
}

class LoyaltyPoints {
    public void addPoints(String guest, int points) {
        System.out.println("Added " + points + " points to " + guest);
    }
}

class EmailService {
    public void sendConfirmation(String guest, String details) {
        System.out.println("Email sent to " + guest + " for " + details);
    }
}