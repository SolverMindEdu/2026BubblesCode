package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RebuiltShiftLogic {

    // Helper class to return multiple values
    public static class ShiftStatus {
        public final boolean isActive;
        public final double timeLeftInShift;
        public final int currentShift; // 1-4, or 0 for Auto/Transition/Endgame

        public ShiftStatus(boolean isActive, double timeLeftInShift, int currentShift) {
            this.isActive = isActive;
            this.timeLeftInShift = timeLeftInShift;
            this.currentShift = currentShift;
        }
    }

    public ShiftStatus getStatus() {
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        var alliance = DriverStation.getAlliance();

        // 1. Initial State: Auto, Transition, or no FMS data yet
        if (matchTime > 120 || matchTime < 0) {
            return new ShiftStatus(true, Math.max(0, matchTime - 120), 0);
        }

        // 2. Identify the Auto Winner (R or B)
        boolean weWonAuto = false;
        if (gameData.length() > 0 && alliance.isPresent()) {
            char winner = gameData.charAt(0);
            if ((alliance.get() == Alliance.Red && winner == 'R') ||
                (alliance.get() == Alliance.Blue && winner == 'B')) {
                weWonAuto = true;
            }
        }

        // 3. Shift Calculation (Teleop shifts start at 120s remaining)
        double timeIntoTeleop = 120 - matchTime;
        int shiftNumber;
        double shiftRemaining;

        if (timeIntoTeleop < 25) {
            shiftNumber = 1;
            shiftRemaining = 25 - timeIntoTeleop;
        } else if (timeIntoTeleop < 50) {
            shiftNumber = 2;
            shiftRemaining = 50 - timeIntoTeleop;
        } else if (timeIntoTeleop < 75) {
            shiftNumber = 3;
            shiftRemaining = 75 - timeIntoTeleop;
        } else if (timeIntoTeleop < 100) {
            shiftNumber = 4;
            shiftRemaining = 100 - timeIntoTeleop;
        } else {
            // Endgame (Last 20 seconds) - Both Hubs always Active
            return new ShiftStatus(true, matchTime, 0);
        }

        // 4. Activation Logic
        // Winner of Auto: Inactive on Odd shifts (1, 3), Active on Even (2, 4)
        // Loser of Auto: Active on Odd shifts (1, 3), Inactive on Even (2, 4)
        boolean active;
        if (weWonAuto) {
            active = (shiftNumber % 2 == 0);
        } else {
            active = (shiftNumber % 2 != 0);
        }

        return new ShiftStatus(active, shiftRemaining, shiftNumber);
    }
}