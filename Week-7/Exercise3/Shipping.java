public class Shipping {

    public String createLabel(String deliveryAddress) {
        return "LABEL-" + deliveryAddress.hashCode();
    }

    public void schedulePickup(String label) {
        System.out.println("Pickup scheduled for: " + label);
    }
}