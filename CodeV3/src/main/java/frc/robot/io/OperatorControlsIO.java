package frc.robot.io;

public interface OperatorControlsIO {

  class OperatorControlsIOInputs {
    public boolean b = false;
    public boolean y = false;
  }

  void updateInputs(OperatorControlsIOInputs inputs);
}