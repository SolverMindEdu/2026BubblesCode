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
        public final boolean isWarning;
        public final double countdown;
        public final int currentShift;
        public final Phase phase;
        public final String phaseName;

        public ShiftStatus(
                boolean isActive,
                boolean isWarning,
                double countdown,
                int currentShift,
                Phase phase,
                String phaseName) {
            this.isActive = isActive;
            this.isWarning = isWarning;
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
            return new ShiftStatus(false, false, 0.0, 0, Phase.UNKNOWN, "UNKNOWN");
        }

        // AUTO: 0:20 -> 0:00
        // DriverStation teleop-style matchTime approximation usually maps this as 20 -> 0
        if (DriverStation.isAutonomousEnabled()) {
            double countdown = clampNonNegative(matchTime);
            return new ShiftStatus(true, countdown <= 10.0, countdown, 0, Phase.AUTO, "AUTO");
        }

        // If not teleop enabled yet, we cannot reliably classify further
        if (!DriverStation.isTeleopEnabled()) {
            return new ShiftStatus(false, false, 0.0, 0, Phase.UNKNOWN, "UNKNOWN");
        }

        // TELEOP official phase boundaries:
        // Transition: 2:20 -> 2:10  = 140 -> 130
        // Shift 1:    2:10 -> 1:45  = 130 -> 105
        // Shift 2:    1:45 -> 1:20  = 105 -> 80
        // Shift 3:    1:20 -> 0:55  = 80  -> 55
        // Shift 4:    0:55 -> 0:30  = 55  -> 30
        // End Game:   0:30 -> 0:00  = 30  -> 0
        //
        // WPILib notes matchTime is approximate, so use game data for hub order. 
        // If game data is missing early in teleop, assume both are effectively safe to treat as active
        // until the shift logic can be determined.

        boolean weWonAuto = false;
        boolean haveWinner = false;

        if (!gameData.isEmpty() && alliance.isPresent()) {
            char winner = gameData.charAt(0);
            if (winner == 'R' || winner == 'B') {
                haveWinner = true;
                weWonAuto =
                    (alliance.get() == Alliance.Red && winner == 'R') ||
                    (alliance.get() == Alliance.Blue && winner == 'B');
            }
        }

        // Transition Shift: 140 -> 130
        if (matchTime > 130.0) {
            double countdown = matchTime - 130.0;
            return new ShiftStatus(true, countdown <= 10.0, countdown, 0, Phase.TRANSITION, "TRANSITION");
        }

        // Shift 1: 130 -> 105
        if (matchTime > 105.0) {
            return buildShiftStatus(1, 105.0, matchTime, haveWinner, weWonAuto);
        }

        // Shift 2: 105 -> 80
        if (matchTime > 80.0) {
            return buildShiftStatus(2, 80.0, matchTime, haveWinner, weWonAuto);
        }

        // Shift 3: 80 -> 55
        if (matchTime > 55.0) {
            return buildShiftStatus(3, 55.0, matchTime, haveWinner, weWonAuto);
        }

        // Shift 4: 55 -> 30
        if (matchTime > 30.0) {
            return buildShiftStatus(4, 30.0, matchTime, haveWinner, weWonAuto);
        }

        // End Game: 30 -> 0
        double countdown = clampNonNegative(matchTime);
        return new ShiftStatus(true, countdown <= 10.0, countdown, 0, Phase.ENDGAME, "ENDGAME");
    }

    private ShiftStatus buildShiftStatus(
            int shiftNumber,
            double lowerBound,
            double matchTime,
            boolean haveWinner,
            boolean weWonAuto) {

        double countdown = clampNonNegative(matchTime - lowerBound);

        // If no valid game data yet, assume active until FMS data appears.
        boolean active;
        if (!haveWinner) {
            active = true;
        } else if (weWonAuto) {
            // Winner inactive in Shift 1, then alternate
            active = (shiftNumber == 2 || shiftNumber == 4);
        } else {
            active = (shiftNumber == 1 || shiftNumber == 3);
        }

        boolean warning = countdown <= 10.0;

        Phase phase = switch (shiftNumber) {
            case 1 -> Phase.SHIFT_1;
            case 2 -> Phase.SHIFT_2;
            case 3 -> Phase.SHIFT_3;
            case 4 -> Phase.SHIFT_4;
            default -> Phase.UNKNOWN;
        };

        return new ShiftStatus(active, warning, countdown, shiftNumber, phase, "SHIFT " + shiftNumber);
    }

    private double clampNonNegative(double value) {
        return Math.max(0.0, value);
    }
}