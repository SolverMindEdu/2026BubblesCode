package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RebuiltShiftLogic {

    public enum Phase {
        AUTO,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        ENDGAME,
        UNKNOWN
    }

    public static class ShiftStatus {
        public final boolean isActive;
        public final double countdown;
        public final int currentShift;
        public final Phase phase;
        public final String phaseName;

        public ShiftStatus(boolean isActive, double countdown, int currentShift, Phase phase, String phaseName) {
            this.isActive = isActive;
            this.countdown = countdown;
            this.currentShift = currentShift;
            this.phase = phase;
            this.phaseName = phaseName;
        }
    }

    public ShiftStatus getStatus() {
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        var alliance = DriverStation.getAlliance();

        if (matchTime < 0) {
            return new ShiftStatus(false, 0.0, 0, Phase.UNKNOWN, "UNKNOWN");
        }

        // Your requested display logic:
        // AUTO:       150 -> 130   (20s)
        // TRANSITION: 130 -> 120   (10s)
        // SHIFT 1:    120 -> 95    (25s)
        // SHIFT 2:    95  -> 70    (25s)
        // SHIFT 3:    70  -> 45    (25s)
        // SHIFT 4:    45  -> 20    (25s)
        // ENDGAME:    20  -> 0     (20s by this math)
        //
        // If you truly want ENDGAME to display 30s instead, that is a custom display choice.
        // See note below.

        if (matchTime > 130.0) {
            return new ShiftStatus(true, matchTime - 130.0, 0, Phase.AUTO, "AUTO");
        }

        if (matchTime > 120.0) {
            return new ShiftStatus(false, matchTime - 120.0, 0, Phase.TRANSITION, "TRANSITION");
        }

        boolean weWonAuto = false;
        if (!gameData.isEmpty() && alliance.isPresent()) {
            char winner = gameData.charAt(0);
            weWonAuto =
                (alliance.get() == Alliance.Red && winner == 'R') ||
                (alliance.get() == Alliance.Blue && winner == 'B');
        }

        if (matchTime > 95.0) {
            return buildShiftStatus(1, 95.0, matchTime, weWonAuto);
        } else if (matchTime > 70.0) {
            return buildShiftStatus(2, 70.0, matchTime, weWonAuto);
        } else if (matchTime > 45.0) {
            return buildShiftStatus(3, 45.0, matchTime, weWonAuto);
        } else if (matchTime > 20.0) {
            return buildShiftStatus(4, 20.0, matchTime, weWonAuto);
        } else {
            return new ShiftStatus(true, matchTime, 0, Phase.ENDGAME, "ENDGAME");
        }
    }

    private ShiftStatus buildShiftStatus(int shiftNumber, double lowerBound, double matchTime, boolean weWonAuto) {
        double countdown = matchTime - lowerBound;

        boolean active;
        if (weWonAuto) {
            active = (shiftNumber == 2 || shiftNumber == 4);
        } else {
            active = (shiftNumber == 1 || shiftNumber == 3);
        }

        Phase phase = switch (shiftNumber) {
            case 1 -> Phase.SHIFT_1;
            case 2 -> Phase.SHIFT_2;
            case 3 -> Phase.SHIFT_3;
            case 4 -> Phase.SHIFT_4;
            default -> Phase.UNKNOWN;
        };

        return new ShiftStatus(active, countdown, shiftNumber, phase, "SHIFT " + shiftNumber);
    }
}