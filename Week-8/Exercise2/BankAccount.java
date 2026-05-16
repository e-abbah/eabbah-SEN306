// ============================================================
// BankAccount.java  –  Given base class (DO NOT MODIFY)
// ============================================================
public class BankAccount {

    protected double balance = 0.0;   // protected: visible to subclasses

    public void deposit(double amount) {
        if (amount > 0) balance += amount;
    }

    public void withdraw(double amount) {
        // Parent only allows withdrawal up to current balance
        if (amount > 0 && amount <= balance) balance -= amount;
    }

    public double getBalance() { return balance; }
}
