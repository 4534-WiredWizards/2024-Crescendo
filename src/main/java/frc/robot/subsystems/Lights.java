// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LightsConstants;

public class Lights extends SubsystemBase {

  // private final CANdle Candle = new CANdle(26, "rio");
  // private RainbowAnimation rainbow = new RainbowAnimation(1,.05,264);

  public static final CANdle Candle = new CANdle(
    LightsConstants.candlePort,
    "rio"
  );

  // DESIGN COLOR ARE NOT STORED HERE ---- SEE RGBColors BELOW
  // Team & Default Colors
  public static final Color red = new Color(255, 0, 0);
  public static final Color black = new Color(0, 0, 0);
  public static final Color darkGray = new Color(15, 15, 15);
  public static final Color lightGray = new Color(102, 102, 102);

  // Game Specific Colors
  public static final Color orange = new Color(255, 83, 0);

  // Indicator Colors
  public static final Color white = new Color(255, 230, 220);
  public static final Color green = new Color(56, 209, 0);
  public static final Color blue = new Color(8, 32, 255);
  public static final Color magenta = new Color(255, 0, 255);
  public static final Color darkOrange = new Color(255, 20, 0);
  public static final Color yellow = new Color(255, 251, 0);

  public static class Color {

    public int red;
    public int green;
    public int blue;

    public Color(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }
  }

  // Constant for first LEDs to ignore
  private final int ignoreFirstLEDS = 8;

  /** Creates a new LED. */
  public Lights() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = false;
    config.disableWhenLOS = false;
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = .4;
    config.vBatOutputMode = VBatOutputMode.Modulated;
    Candle.configAllSettings(config, 100);
    // RobotInit();

  }

  // public void RobotInit() {
  //   LEDSegment.CandleLEDs.fullClear();
  //   LEDSegment.Panel.fullClear();
  //   LEDSegment.CandleLEDs.setColor(darkOrange);
  // }

  public void teleopStart() {
    LEDSegment.CandleLEDs.setFadeAnimation(Lights.lightGray, .000001);
    LEDSegment.Panel.setFadeAnimation(Lights.lightGray, .000001);
  }

  // ------------------------- Subsytem Lights -------------------------
  // Intake Start
  public void intakeStart() {
    LEDSegment.Panel.setFadeAnimation(blue, 0.05);
  }

  // Intake Stop
  public void intakeStop() {
    LEDSegment.Panel.clearAnimation();
    teleopStart();
  }

  // Shooter Start
  public void shooterStart() {
    LEDSegment.Panel.setFadeAnimation(red, 0.7);
  }

  public void hasValidShot() {
    // Animation when the robot has a valid shot
    LEDSegment.Panel.setFadeAnimation(magenta, 1);
    // LEDSegment.Panel.setStrobeAnimation(magenta, .05);
  }

  public void hasValidShotStop() {
    LEDSegment.Panel.clearAnimation();
    teleopStart();
  }

  // Shooter Stop
  public void shooterStop() {
    LEDSegment.Panel.clearAnimation();
    teleopStart();
  }

  // Note Collected
  // public void noteCollected() {
  //   // Save the current animation
  //   // Set to 2 secound orange strobe
  //   LEDSegment.Panel.setStrobeAnimation(orange, 0.05);
  //   try{Thread.sleep(200);}catch(InterruptedException e){};
  //   LEDSegment.Panel.clearAnimation();
  //   enableRobot();
  // }
  //RobotInit
  public void robotInit() {
    LEDSegment.CandleLEDs.fullClear();
    LEDSegment.Panel.fullClear();
    drawImage(Constants.LightDesign.WIRED_WIZARDS);
  }

  // Disable Robot
  public void disabledStart() {
    LEDSegment.CandleLEDs.setFadeAnimation(Lights.orange, .000001);
    LEDSegment.Panel.fullClear();
    drawImage(Constants.LightDesign.WIRED_WIZARDS);
  }

  public void clearSegmentCommand(LEDSegment segment) {
    segment.clearAnimation();
    segment.disableLEDs();
  }

  public static enum LEDSegment {
    CandleLEDs(0, 8, 0),
    Panel(8, 256, 1);

    public final int startIndex;
    public final int segmentSize;
    public final int animationSlot;

    private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }

    public void fullClear() {
      clearAnimation();
      disableLEDs();
    }

    public void clearAnimation() {
      Candle.clearAnimation(animationSlot);
    }

    public void disableLEDs() {
      setColor(black);
    }

    public void setColor(Color color) {
      Candle.clearAnimation(animationSlot);
      Candle.setLEDs(
        color.red,
        color.green,
        color.blue,
        100,
        startIndex,
        segmentSize
      );
    }

    private void setAnimation(Animation animation) {
      Candle.animate(animation, animationSlot);
    }

    public void setFlowAnimation(Color color, double speed) {
      // Animation that gradually lights the entire LED strip one LED at a time.
      setAnimation(
        new ColorFlowAnimation(
          color.red,
          color.green,
          color.blue,
          0,
          speed,
          segmentSize,
          Direction.Forward,
          startIndex
        )
      );
    }

    public void setFadeAnimation(Color color, double speed) {
      //Animation that fades into and out of a specified color
      setAnimation(
        new SingleFadeAnimation(
          color.red,
          color.green,
          color.blue,
          0,
          speed,
          segmentSize,
          startIndex
        )
      );
    }

    public void setBandAnimation(Color color, double speed) {
      //Animation that sends a pocket of light across the LED strip.
      setAnimation(
        new LarsonAnimation(
          color.red,
          color.green,
          color.blue,
          0,
          speed,
          segmentSize,
          BounceMode.Front,
          3,
          startIndex
        )
      );
    }

    public void setStrobeAnimation(Color color, double speed) {
      //Animation that strobes the LEDs a specified color
      setAnimation(
        new StrobeAnimation(
          color.red,
          color.green,
          color.blue,
          0,
          speed,
          segmentSize,
          startIndex
        )
      );
    }

    // Random fun stuff - not practical
    public void setRainbowAnimation(double speed) {
      setAnimation(
        new RainbowAnimation(1, speed, segmentSize, false, startIndex)
      );
    }

    public void setTwinkleAnimation(Color color, double speed) {
      setAnimation(
        new TwinkleAnimation(
          color.red,
          color.green,
          color.blue,
          0,
          speed,
          segmentSize,
          TwinklePercent.Percent42
        )
      );
    }

    public void setFireAnimation(double brightness, double speed) {
      setAnimation(new FireAnimation(1, speed, segmentSize, 1, 0));
    }

    public void setColorFlowAnimation(Color color, double speed) {
      setAnimation(
        new ColorFlowAnimation(
          color.red,
          color.green,
          color.blue,
          0,
          speed,
          segmentSize,
          Direction.Forward
        )
      );
    }
  }

  // OLD CODE

  public static void setColor(Color color) {
    Candle.clearAnimation(0);
    Candle.setLEDs(color.red, color.green, color.blue);
    // Candle.setLEDs(0, 0, 255, 0, 1, 1);

  }

  public static void setColorOld(int r, int g, int b) {
    Candle.clearAnimation(0);
    Candle.setLEDs(r, g, b);
    // Candle.setLEDs(0, 0, 255, 0, 1, 1);

  }

  public static final class RGBColors {

    public static final int[] DARKGRAY = { 15, 15, 15, 15 };
    public static final int[] LIGHTGRAY = { 102, 102, 102, 100 };
    public static final int[] RED = { 255, 0, 0, 100 };
    public static final int[] BLACK = { 0, 0, 0, 0 };

    public static final class ww {

      public static final int[] DARKGRAY = RGBColors.DARKGRAY;
      public static final int[] LIGHTGRAY = RGBColors.LIGHTGRAY;
      public static final int[] RED = RGBColors.RED;
    }

    public static final class nCino {

      public static final int[] RED = { 207, 16, 19, 100 };
      public static final int[] YELLOW = { 253, 188, 1, 100 };
      public static final int[] GREEN = { 92, 182, 77, 100 };
      public static final int[] BLUE = { 23, 170, 220, 100 };
      public static final int[] TEXT = { 255, 255, 255, 20 };
    }

    public static final class Corning {

      public static final int[] TEXT = { 0, 93, 150, 20 };
    }

    public static final class CFCC {

      public static final int[] TEXT = { 255, 255, 255, 100 };
    }
  }

  public void setRowAndColumn(int row, int col, int r, int g, int b, int w) {
    // Row 0-7
    // Column 0-31
    // Calculates the position of the LED in the 8x32 grid
    // col=col-1;
    // row=row-1;
    int index = 0;
    if (col % 2 == 0) {
      index = (row) + (col * 8);
    } else {
      index = (7 - row) + (col * 8);
    }
    index = index + ignoreFirstLEDS;
    // System.out.println("Setting LED:"+ index);

    Candle.setLEDs(r, g, b, w, index, 1);
  }

  public void setArrayOfValues(int[][][] values, int offsetRow, int offsetCol) {
    for (int i = 0; i < values.length; i++) {
      int row = (values[i][0][0] + offsetRow) % 8;
      int col = (values[i][0][1] + offsetCol) % 32;
      int r = (values[i][1][0]);
      int g = (values[i][1][1]);
      int b = (values[i][1][2]);
      int w = values[i][1][3];
      // setRowAndColumn(row, col, r, g, b, w);
      Thread setPix = new Thread(
        new Runnable() {
          @Override
          public void run() {
            setRowAndColumn(row, col, r, g, b, w);
          }
        }
      );
      setPix.start();
    }
  }

  public void drawImage(int[][][] values) {
    setArrayOfValues(values, 0, 0);
  }

  // Scrolling animation
  public void scrollAnimation(int[][][] values, int msDelay, int loops) {
    for (int i = 0; i < 32 * loops; i++) {
      // final int index = i;
      // Thread addPix = new Thread(new Runnable() {

      setArrayOfValues(values, 0, i);
      try {
        Thread.sleep(msDelay);
      } catch (InterruptedException e) {}
    }
  }
}
