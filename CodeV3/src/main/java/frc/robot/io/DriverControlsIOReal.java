package frc.robot.io;

import edu.wpi.first.wpilibj.XboxController;

public class DriverControlsIOReal implements DriverControlsIO {
  private final XboxController controller;

  public DriverControlsIOReal(int port) {
    controller = new XboxController(port);
  }

  @Override
  public void updateInputs(DriverControlsIOInputs inputs) {
    inputs.leftBumperPressed = false;

    inputs.leftY = -controller.getLeftY();
    inputs.leftX = -controller.getLeftX();
    inputs.rightX = -controller.getRightX();

    inputs.leftTrigger = controller.getLeftTriggerAxis();
    inputs.rightTrigger = controller.getRightTriggerAxis();

    inputs.a = controller.getAButton();
    inputs.b = controller.getBButton();
    inputs.y = controller.getYButton();
    inputs.x = controller.getXButton();

    inputs.leftBumper = controller.getLeftBumper();
    inputs.leftBumperPressed = controller.getLeftBumperPressed();  // important
  }
}