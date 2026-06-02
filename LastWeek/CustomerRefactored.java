public class CustomerRefactored {

    // Mock method to represent the existing infrastructure function
    private static void sendEmail(String email, String message) {
        System.out.println("[Email Sent to " + email + "]: " + message);
    }

    // =========================================================================
    // THE MAIN METHOD (The entry point Java was looking for)
    // =========================================================================
    public static void main(String[] args) {
        // 1. Create an instance of our refactored orchestrator class
        CustomerRefactored processor = new CustomerRefactored();

        // 2. Instantiate a mock Customer object (Alice, Tier 2 discount, VIP status)
        Customer mockCustomer = new Customer("Alice Smith", "123 Dev Lane", 2, "alice@email.com", true);

        // 3. Set up a sample array of item order amounts
        double[] mockOrders = {25.50, 10.00, 64.50}; // Total should be 100.0
        int mockOrderCount = mockOrders.length;

        System.out.println("--- Running Refactored Customer Routine ---");
        
        // 4. Run the routine
        processor.processCustomer(mockCustomer, mockOrders, mockOrderCount);

        // 5. Verify that the object's inner state was successfully modified (Fixing the bug)
        System.out.println("Verified Customer State - Updated Total in Object: $" + mockCustomer.getDiscountedTotal());
    }

    // =========================================================================
    // CUSTOMER CLASS
    // =========================================================================
    public static class Customer {
        private final String name;
        private final String address;
        private final int tier;
        private final String email;
        private final boolean isVip;
        private double discountedTotal; 

        public Customer(String name, String address, int tier, String email, boolean isVip) {
            this.name = name;
            this.address = address;
            this.tier = tier;
            this.email = email;
            this.isVip = isVip;
            this.discountedTotal = 0.0;
        }

        public String getName() { return name; }
        public String getAddress() { return address; }
        public int getTier() { return tier; }
        public String getEmail() { return email; }
        public boolean isVip() { return isVip; }
        public double getDiscountedTotal() { return discountedTotal; }
        public void setDiscountedTotal(double discountedTotal) { 
            this.discountedTotal = discountedTotal; 
        }
    }

    // =========================================================================
    // CORE ORCHESTRATOR
    // =========================================================================
    public void processCustomer(Customer customer, double[] orders, int orderCount) {
        validateCustomerData(customer, orders, orderCount);

        double rawSum = orderSum(orders, orderCount);
        double discountRate = applicableDiscountRate(customer.getTier());
        double finalTotal = rawSum - (rawSum * discountRate);

        customer.setDiscountedTotal(finalTotal);

        String receiptMessage = customerReceiptMessage(customer, finalTotal);
        dispatchNotifications(customer, receiptMessage);
    }

    // =========================================================================
    // SUB-ROUTINES 
    // =========================================================================

    private void validateCustomerData(Customer customer, double[] orders, int count) {
        if (customer == null) {
            throw new IllegalArgumentException("Customer cannot be null.");
        }
        if (orders == null || count < 0 || count > orders.length) {
            throw new IllegalArgumentException("Invalid order array or count boundaries.");
        }
        if (customer.getTier() < 0) {
            throw new IllegalArgumentException("Customer tier cannot be negative.");
        }
        for (int i = 0; i < count; i++) {
            if (orders[i] < 0) {
                throw new IllegalArgumentException("Order amounts cannot be negative. Found: " + orders[i]);
            }
        }
    }

    private double orderSum(double[] orders, int count) {
        double sum = 0;
        for (int i = 0; i < count; i++) {
            sum += orders[i];
        }
        return sum;
    }

    private double applicableDiscountRate(int tier) {
        if (tier == 1) return 0.1;
        if (tier == 2) return 0.2;
        return 0.0;
    }

    private String customerReceiptMessage(Customer customer, double total) {
        String msg = "Hello " + customer.getName() + " of " + customer.getAddress() + ", your total is " + total;
        if (customer.isVip()) {
            msg += " (VIP)";
        }
        return msg;
    }

    private void dispatchNotifications(Customer customer, String message) {
        System.out.println(message);
        if (customer.getEmail() != null) {
            sendEmail(customer.getEmail(), message);
        }
    }
}