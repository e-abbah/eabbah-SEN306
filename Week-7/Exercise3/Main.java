public class Main {

    public static void main(String[] args) {

        LegacyOrderFacade facade = new LegacyOrderFacade();

        facade.placeOrder(
                "test@email.com",
                "ITEM123",
                100.0,
                "123 Main Street"
        );
    }
}