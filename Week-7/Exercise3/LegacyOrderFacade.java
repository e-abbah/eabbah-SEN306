public class LegacyOrderFacade {

    private final LegacyOrderProcessor processor;

    public LegacyOrderFacade() {
        this.processor = new LegacyOrderProcessor();
    }

    public LegacyOrderFacade(LegacyOrderProcessor processor) {
        this.processor = processor;
    }

    public void placeOrder(String customerEmail,
            String itemCode,
            double amount,
            String deliveryAddress) {

        validateBasicInputs(customerEmail, itemCode, amount, deliveryAddress);

        processor.processOrder(customerEmail, itemCode, amount, deliveryAddress);
    }

    private void validateBasicInputs(String customerEmail,
            String itemCode,
            double amount,
            String deliveryAddress) {

        if (customerEmail == null || customerEmail.isBlank()
                || itemCode == null || itemCode.isBlank()
                || deliveryAddress == null || deliveryAddress.isBlank()) {
            throw new IllegalArgumentException("Required fields must not be empty");
        }

        if (amount <= 0) {
            throw new IllegalArgumentException("Amount must be greater than zero");
        }
    }
}