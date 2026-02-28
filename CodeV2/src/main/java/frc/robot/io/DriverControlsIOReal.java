package frc.robot.io;

import edu.wpi.first.wpilibj.XboxController;

public class DriverControlsIOReal implements DriverControlsIO {
  private final XboxController controller;

  public DriverControlsIOReal(int port) {
    controller = new XboxController(port);
  }

  @Override
  public void updateInputs(DriverControlsIOInputs inputs) {
    inputs.leftY = -controller.getLeftY();
    inputs.leftX = controller.getLeftX();
    inputs.rightX = controller.getRightX();
    inputs.a = controller.getAButton();
    inputs.b = controller.getBButton();
  }
}