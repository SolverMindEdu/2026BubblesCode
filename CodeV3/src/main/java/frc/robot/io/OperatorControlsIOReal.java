package frc.robot.io;

import edu.wpi.first.wpilibj.XboxController;

public class OperatorControlsIOReal implements OperatorControlsIO {

  private final XboxController controller;

  public OperatorControlsIOReal(int port) {
    controller = new XboxController(port);
  }

  @Override
  public void updateInputs(OperatorControlsIOInputs inputs) {
    inputs.b = controller.getBButton();
    inputs.y = controller.getYButton();
  }
}