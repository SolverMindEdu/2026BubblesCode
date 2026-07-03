package frc.robot;

import java.util.ArrayList;
import java.util.List;

public class PrecheckReport {
    private final List<String> failures = new ArrayList<>();

    public void fail(String item) {
        failures.add(item);
    }

    public boolean allPassed() {
        return failures.isEmpty();
    }

    public String failureSummary() {
        return failures.isEmpty() ? "OK" : String.join(", ", failures);
    }

    public List<String> getFailures() {
        return failures;
    }
}