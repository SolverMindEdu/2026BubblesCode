package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

public interface DriverControlsIO {
  @AutoLog
  public static class DriverControlsIOInputs {
    public double leftY = 0.0;
    public double leftX = 0.0;
    public double rightX = 0.0;
    public boolean a = false;
    public boolean b = false;
  }

  void updateInputs(DriverControlsIOInputs inputs);
}