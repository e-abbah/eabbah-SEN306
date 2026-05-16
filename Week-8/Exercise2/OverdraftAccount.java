// ============================================================
// OverdraftAccount.java  –  EXERCISE 2: Inheritance example
//
// WHY inheritance here (not composition)?
//   BankAccount.withdraw() blocks negative balances.
//   We need to OVERRIDE that behaviour – only inheritance
//   lets us do that without modifying the parent.
//
// TRADE-OFF (fragile base class):
//   This subclass directly accesses the protected field
//   `balance`.  If BankAccount later makes `balance` private
//   and uses a getter, this class BREAKS – that's the
//   fragile base class problem.
// ============================================================
public class OverdraftAccount extends BankAccount {

    private static final double OVERDRAFT_LIMIT = -500.0;

    // ----------------------------------------------------------
    // Override withdraw to allow balance to go negative
    // down to OVERDRAFT_LIMIT, and log every transaction.
    // ----------------------------------------------------------
    @Override
    public void withdraw(double amount) {
        if (amount <= 0) return;                    // ignore bad input

        double newBalance = balance - amount;       // direct access to protected field

        if (newBalance >= OVERDRAFT_LIMIT) {
            balance = newBalance;                   // allow overdraft
            System.out.println("Withdrew £" + amount
                    + " | New balance: £" + balance);
        } else {
            System.out.println("Overdraft limit exceeded. Withdrawal of £"
                    + amount + " denied.");
        }
    }

    // ----------------------------------------------------------
    // Override deposit to add logging
    // ----------------------------------------------------------
    @Override
    public void deposit(double amount) {
        super.deposit(amount);                      // reuse parent logic
        System.out.println("Deposited £" + amount
                + " | New balance: £" + balance);
    }
}
