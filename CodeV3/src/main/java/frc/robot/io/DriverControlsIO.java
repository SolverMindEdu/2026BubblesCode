package frc.robot.io;

public interface DriverControlsIO {
  class DriverControlsIOInputs {
  public double leftY = 0.0;
  public double leftX = 0.0;
  public double rightX = 0.0;

  public double leftTrigger = 0.0;
  public double rightTrigger = 0.0;

  public boolean a = false;
  public boolean b = false;
  public boolean x = false;
  public boolean y = false;
  public boolean leftBumper = false;

  public boolean leftBumperPressed = false;
}

  void updateInputs(DriverControlsIOInputs inputs);
}