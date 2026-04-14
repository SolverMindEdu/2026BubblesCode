package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {

    public static final class Intake {
      public static final int ROLLERS_ID = 11;
      public static final int SLAPDOWN_ID = 13;

      public static final double UP_LIMIT_ROT = 0.0;
      public static final double DOWN_LIMIT_ROT = 19.5;

      public static final double UP_ROT = 0.05;
      public static final double TRAVEL_ROT = 16.0;
      public static final double SHOOT_ROT = 20.0;
      public static final double DOWN_ROT = 19.5;

      public static final double kP = 10.5;
      public static final double kI = 0.0;
      public static final double kD = 0.1;

      public static final double TOLERANCE_ROT = 0.05;

      public static final double ROLLER_RPS = 65.0;
      public static final double INDEXER_PERCENT = 0.2;

      public static final double CRUISE_VEL_RPS = 75.0;
      public static final double ACCEL_RPS2 = 60.0;

      public static final double SLOW_UP_CRUISE_VEL_RPS = 7.5;
      public static final double SLOW_UP_ACCEL_RPS2 = 9.0;
  }

    public static final class Shooter {
        public static final int LEFT_ID = 12;
        public static final int MIDDLE_ID = 16;
        public static final int RIGHT_ID = 17;

        public static final double KICKER_PERCENT = 0.8;
        public static final double INDEXER_PERCENT = 0.7;

        public static final double DEFAULT_TARGET_RPS = 50.5;
        public static final double DEFAULT_HOOD_DEG = 0.0;

        public static final double PULSE_PERIOD_SEC = 1.0;
        public static final double PULSE_HALF_SEC = PULSE_PERIOD_SEC / 4.0;
        public static final double SHOOT_ASSIST_INTAKE_RPS = 30.0;
        public static final double INTAKE_RAISE_DELAY_SEC = 2.5;

        public static final double LATCH_MIN_TIME_SEC = 0.8;
    }

    public static final class Indexer {
        public static final int MOTOR_ID = 14;
    }

    public static final class Kickers {
        public static final int LEFT_ID = 21;
        public static final int RIGHT_ID = 20;
    }

    public static final class Hood {
        public static final int MOTOR_ID = 19;

        public static final double MIN_DEG = 0.2;
        public static final double MAX_DEG = 320.0;
        public static final double STEP_DEG = 3.0;

        public static final double CRUISE_VEL_RPS = 40.0;
        public static final double ACCEL_RPS2 = 30.0;
    }

    public static final class LED {
      public static final int CANDLE_ID = 18;

      public static final int START = 0;
      public static final int END = 30;

      public static final double REVERSE_STROBE_SECONDS = 0.4;
    }

    public class fieldPoses{
        public static final Pose2d blueAllianceHub = new Pose2d(
            FieldConstants.Hub.topCenterPoint.getX(),
            FieldConstants.Hub.topCenterPoint.getY(),
            new Rotation2d()
        );

        public static final Pose2d redAllianceHub = new Pose2d(
            FieldConstants.Hub.oppTopCenterPoint.getX(),
            FieldConstants.Hub.oppTopCenterPoint.getY(),
            new Rotation2d()
        );
    }

    public static final Transform3d robotToShooter =
        new Transform3d(
            new Translation3d(0.5,-0.254, 0.6096),
            new Rotation3d(0,0,180)
        );

    public static final class shooterConstants {
    public static final double hubOffset = 0.0;
    }
}