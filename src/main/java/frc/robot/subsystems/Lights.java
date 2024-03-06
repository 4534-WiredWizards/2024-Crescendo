// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import frc.robot.Constants.LightsConstants;

public class Lights extends SubsystemBase {
  // private final CANdle Candle = new CANdle(26, "rio");
  // private RainbowAnimation rainbow = new RainbowAnimation(1,.05,264);

  public static final CANdle Candle = new CANdle(LightsConstants.candlePort, "rio");

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
  public static final Color darkOrange =  new Color(255, 20, 0);
  public static final Color yellow = new Color(255,251,0);

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
    config.brightnessScalar = .5;
    config.vBatOutputMode = VBatOutputMode.Modulated;
    Candle.configAllSettings(config, 100);
    RobotInit();

  }

  public void RobotInit() {
    LEDSegment.CandleLEDs.fullClear();
    LEDSegment.Panel.fullClear();
    LEDSegment.CandleLEDs.setColor(darkOrange);
  }

  public void clearSegmentCommand(LEDSegment segment) {
      segment.clearAnimation();
      segment.disableLEDs();
  }

  public static enum LEDSegment {
    CandleLEDs(0,8,0),
    Panel(8,256,1);


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
      Candle.setLEDs(color.red, color.green, color.blue, 100, startIndex, segmentSize);
    }

    private void setAnimation(Animation animation) {
      Candle.animate(animation, animationSlot);
    }

    public void setFlowAnimation(Color color, double speed) {
        setAnimation(new ColorFlowAnimation(
                color.red, 
                color.green, 
                color.blue, 
                0, 
                speed, 
                segmentSize, 
                Direction.Forward, 
                startIndex
        ));
    }

    public void setFadeAnimation(Color color, double speed) {
        setAnimation(
                new SingleFadeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setBandAnimation(Color color, double speed) {
        setAnimation(new LarsonAnimation(
                color.red, color.green, color.blue, 0, speed, segmentSize, BounceMode.Front, 3, startIndex));
    }

    public void setStrobeAnimation(Color color, double speed) {
        setAnimation(new StrobeAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setRainbowAnimation(double speed) {
        setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
    }

    public void setTwinkleAnimation(Color color, double speed) {
      setAnimation(new TwinkleAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, TwinklePercent.Percent42));
    }

    public void setFireAnimation(double brightness, double speed) {
      setAnimation(new FireAnimation(1, speed, segmentSize, 1,0));
    }

    public void setColorFlowAnimation(Color color, double speed) {
      setAnimation(new ColorFlowAnimation(color.red, color.green, color.blue, 0, speed, segmentSize, Direction.Forward));
    }

  }





//  MESSING AROUND


public void scrollText(String text, int speed, Color color) {
  // Set the text color
  setColor(color);

  // Iterate through each character in the text
  for (int i = 0; i < text.length(); i++) {
      char currentChar = text.charAt(i);

      // Get the ASCII value of the character
      int asciiValue = (int) currentChar;

      // Display the character on the LED matrix
      setCharacter(asciiValue, color);

      // Wait for a short duration to create the scrolling effect
      Timer.delay(speed / 100.0);  // Convert speed to seconds

      // Clear the display to prepare for the next character
      clearDisplay();
  }
}


 // Additional helper method to set a character on the LED matrix
 private void setCharacter(int asciiValue, Color color) {
  // Adjust the ASCII value if needed to map to your specific character set
  // You may need to implement a lookup table for character mappings

  // Example: Map ASCII values 'A' to 'Z' to 0 to 25
  if (asciiValue >= 'A' && asciiValue <= 'Z') {
      asciiValue -= 'A';
  }

  // Example: Map ASCII values 'a' to 'z' to 0 to 25
  else if (asciiValue >= 'a' && asciiValue <= 'z') {
      asciiValue -= 'a';
  }

  // Example: Map ASCII values '0' to '9' to 26 to 35
  else if (asciiValue >= '0' && asciiValue <= '9') {
      asciiValue = 26 + (asciiValue - '0');
  }

  // Add more mappings as needed based on your character set

  // Display the character on the LED matrix at a specific position
  // Adjust the position and other parameters based on your matrix configuration
  setRowAndColumn(0, asciiValue * 4, color.red, color.green, color.blue, 0);
}

// Additional helper method to clear the LED matrix display
private void clearDisplay() {
  // Clear the display by setting all LEDs to black
  setColor(black);
}



  // OLD CODE

  public static void setColor(Color color) {
    Candle.clearAnimation(0);
    Candle.setLEDs(color.red, color.green, color.blue);

    // Candle.setLEDs(0, 0, 255, 0, 1, 1);

  }

  public static void setColorOld(int r, int g, int b) {
    Candle.clearAnimation(0);
    Candle.setLEDs(r,g,b);
    // Candle.setLEDs(0, 0, 255, 0, 1, 1);

  }

  public static final class RGBColors {
    public static final int[] DARKGRAY = { 15, 15, 15, 15 };
    public static final int[] LIGHTGRAY = { 102, 102, 102, 100 };
    public static final int[] RED = { 255, 0, 0, 100 };
    public static final int[] BLACK = {0,0,0,0};
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
      Thread setPix = new Thread(new Runnable() {
        @Override
        public void run() {
          setRowAndColumn(row, col, r, g, b, w);
        }
      });
      setPix.start();
    }
  }

  public void drawImage(int[][][] values) {
    setArrayOfValues(values, 0, 0);
  }

  // Scrolling animation
  public void scrollAnimation(int[][][] values, int msDelay, int loops) {


    for (int i = 0; i < 32*loops; i++) {
      // final int index = i;
      // Thread addPix = new Thread(new Runnable() {
   
      setArrayOfValues(values, 0, i);
      try {Thread.sleep(msDelay);}catch(InterruptedException e){}
    }
  }


}
