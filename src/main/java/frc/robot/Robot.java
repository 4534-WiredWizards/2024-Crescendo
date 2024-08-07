// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    UsbCamera fisheye = CameraServer.startAutomaticCapture();
    fisheye.setResolution(320, 240);
    fisheye.setPixelFormat(PixelFormat.kMJPEG);
    fisheye.setFPS(60);
    RobotContainer.leds.robotInit();
    // SmartDashboard.putString("Alliance Color", (RobotContainer.getAllianceColor()!=null) ? RobotContainer.getAllianceColor() : "None");
    SmartDashboard.putNumber("Set Shooter Speed", 0);
    DataLogManager.start();
    NetworkTableInstance
      .getDefault()
      .getTable("limelight")
      .getEntry("pipeline")
      .setNumber(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    // Limit switch on shuffle board
    SmartDashboard.putBoolean(
      "Intake LM",
      m_robotContainer.intake.getIntakeStatus()
    );
    SmartDashboard.putBoolean(
      "Arm LM FW",
      m_robotContainer.arm.getArmStatusFw()
    );
    SmartDashboard.putBoolean(
      "Arm LM RV",
      m_robotContainer.arm.getArmStatusRv()
    );
    // SmartDashboard.putBoolean(
    //   "Climb LM",
    //   m_robotContainer.climb.getClimbStatus()
    // );
    // SmartDashboard.putNumber(
    //   "Climb Encoder",
    //   m_robotContainer.climb.getPosition()
    // );
    SmartDashboard.putNumber(
      "Arm Abs Encoder",
      Units.radiansToDegrees((m_robotContainer.arm.getAbsolutePosition()))
    );
    // SmartDashboard.putNumber("Arm Abs Encoder", m_robotContainer.arm.getAbsolutePosition()); // For raw value
    SmartDashboard.putNumber("NavX", m_robotContainer.swerve.getHeading());
    // SmartDashboard.putString("Alliance", DriverStation.getAlliance().get().toString());
    // SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Limelight debug
    SmartDashboard.putBoolean(
      "TAG Has Target",
      m_robotContainer.limelight.getHasValidTarget()
    );
    SmartDashboard.putNumber(
      "TAG FrontBack Distance",
      Math.abs(m_robotContainer.limelight.targetpose.getDiagonalDistance())
    );
    SmartDashboard.putBoolean(
      "TAG In Range",
      (
        Math.abs(m_robotContainer.limelight.targetpose.getDiagonalDistance()) <=
        4
      )
    );
    SmartDashboard.putBoolean(
      "Tag Valid Shot",
      (
        (
          Math.abs(
            m_robotContainer.limelight.targetpose.getDiagonalDistance()
          ) <=
          4
        ) &&
        m_robotContainer.limelight.getHasValidTarget()
      )
    );
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotContainer.leds.disabledStart();
    {
      try {
        Thread.sleep(2000);
      } catch (InterruptedException e) {}
    }
    RobotContainer.leds.disabledStart();
  }

  @Override
  public void disabledPeriodic() {
    // While robot is disabled, run the LED code

    //RobotContainer.ledsm_led.scrollAnimation(Constants.LightDesign.WIRED_WIZARDS, 100, 2)),
    //    Lights.LEDSegment.Panel.fullClear();
    //    RobotContainer.leds.drawImage(Constants.LightDesign.WIRED_WIZARDS);
    //    {try{Thread.sleep(2000);}catch(InterruptedException e){}};
    //    Lights.LEDSegment.Panel.fullClear();
    //    RobotContainer.leds.drawImage(Constants.LightDesign.nCino);
    //    {try{Thread.sleep(2000);}catch(InterruptedException e){}};
    //    Lights.LEDSegment.Panel.fullClear();
    //    RobotContainer.leds.drawImage(Constants.LightDesign.Corning);
    //    {try{Thread.sleep(2000);}catch(InterruptedException e){}};
    //    Lights.LEDSegment.Panel.fullClear();
    //    RobotContainer.leds.drawImage(Constants.LightDesign.CFCC);

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    NetworkTableInstance
      .getDefault()
      .getTable("limelight")
      .getEntry("pipeline")
      .setNumber(0);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    NetworkTableInstance
      .getDefault()
      .getTable("limelight")
      .getEntry("pipeline")
      .setNumber(0);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // RobotContainer.leds.teleopStart(); 1   
  }

  Boolean noShot = false;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (
      (
        Math.abs(m_robotContainer.limelight.targetpose.getDiagonalDistance()) <=
        4
      ) &&
      m_robotContainer.limelight.getHasValidTarget()
    ) {
      noShot = false;
      // RobotContainer.leds.hasValidShot();
    } else if (noShot == false) {
      noShot = true;
      // System.out.println("End of april tag");
      // RobotContainer.leds.hasValidShotStop();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
