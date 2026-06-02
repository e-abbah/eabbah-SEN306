public class Main {
    public static void main(String[] args) {

        BookingFacade facade = new BookingFacade();

        boolean success = facade.bookRoom("Johnny", "Deluxe", 200.0);

        System.out.println(success ? "Booking successful!" : "Booking failed");
    }
}

// ===================== FACADE =====================

class BookingFacade {
    private RoomService rooms;
    private PaymentService payment;
    private LoyaltyPoints loyalty;
    private EmailService email;

    public BookingFacade() {
        this.rooms = new RoomService();
        this.payment = new PaymentService();
        this.loyalty = new LoyaltyPoints();
        this.email = new EmailService();
    }

    public boolean bookRoom(String guest, String roomType, double price) {
        if (!rooms.isAvailable(roomType)) return false;
        if (!payment.charge(guest, price)) return false;

        rooms.book(roomType, guest);
        loyalty.addPoints(guest, (int) price);
        email.sendConfirmation(guest, roomType);
        return true;
    }
}

// ===================== SUBSYSTEMS =====================

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
    public void sendConfirmation(String guest, String roomType) {
        System.out.println("Email sent to " + guest + " for " + roomType + " booking");
    }
}

//Reasons for a facade
// 1. Loose coupling. The main class does not depend directly on RoomService, PaymentService, etc. It only interacts with the BookingFacade. This allows us to change the underlying services without affecting the main class.
// 2. Simplified interface 
// 3. Improved readability and usability
