package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  public enum LEDState {
    IDLE_BLUE,
    AUTO_RAINBOW,
    ALIGNING_RED_FLASH,
    READY_GREEN,
    SHOOTING_BLUE_FLASH,
    INTAKING_YELLOW,
    PASSING_YELLOW_FLASH
  }

  private final CANdle candle = new CANdle(18, CANBus.roboRIO()); // change CAN ID if needed

  // external strip start/end
  private static final int START = 0;
  private static final int END = 27; // 60 LEDs -> 8..67

  private LEDState currentState = null;

  public LEDs() {
    var cfg = new CANdleConfiguration();
    cfg.LED.StripType = StripTypeValue.GRB;
    cfg.LED.BrightnessScalar = 0.6;
    cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    candle.getConfigurator().apply(cfg);

    clearAnimations();
    setState(LEDState.IDLE_BLUE);
  }

  private void clearAnimations() {
    for (int i = 0; i < 8; i++) {
      candle.setControl(new EmptyAnimation(i));
    }
  }

  public void setState(LEDState newState) {
    if (newState == currentState) {
      return;
    }

    currentState = newState;
    clearAnimations();

    switch (newState) {
      case AUTO_RAINBOW:
        candle.setControl(new RainbowAnimation(START, END).withSlot(0));
        break;

      case ALIGNING_RED_FLASH:
        candle.setControl(
            new StrobeAnimation(START, END)
                .withSlot(0)
                .withColor(new RGBWColor(255, 0, 0, 0)));
        break;

      case READY_GREEN:
        candle.setControl(
            new SolidColor(START, END)
                .withColor(new RGBWColor(0, 255, 0, 0)));
        break;

      case SHOOTING_BLUE_FLASH:
        candle.setControl(
            new StrobeAnimation(START, END)
                .withSlot(0)
                .withColor(new RGBWColor(0, 0, 255, 0)));
        break;

      case INTAKING_YELLOW:
        candle.setControl(
            new SolidColor(START, END)
                .withColor(new RGBWColor(255, 180, 0, 0)));
        break;

      case PASSING_YELLOW_FLASH:
        candle.setControl(
            new StrobeAnimation(START, END)
                .withSlot(0)
                .withColor(new RGBWColor(255, 180, 0, 0)));
        break;

      case IDLE_BLUE:
      default:
        candle.setControl(
            new SolidColor(START, END)
                .withColor(new RGBWColor(0, 0, 255, 0)));
        break;
    }
  }
}