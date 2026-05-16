// ============================================================
// Main.java  –  Exercise 2: OverdraftAccount client code
// ============================================================
public class Main {

    public static void main(String[] args) {

        System.out.println("=== Exercise 2: OverdraftAccount ===\n");

        OverdraftAccount account = new OverdraftAccount();

        account.deposit(100);     // balance: £100
        account.withdraw(50);     // balance: £50   – normal withdrawal
        account.withdraw(400);    // balance: -£350 – overdraft allowed
        account.withdraw(200);    // denied  – would go below -£500

        System.out.println("Final balance: £" + account.getBalance()); // -350.0
    }
}
