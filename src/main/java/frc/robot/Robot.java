// Copyright (c) Triple Helix Robotics

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.arm.ArmInterface;
import frc.robot.drive.SwerveDrive;

public class Robot extends TimedRobot {

  /////////////////////////////////////////////////////////////////////////////
  //                                                                         //
  //           IO, global sensors, subsystems, and global commands           //
  //                                                                         //
  /////////////////////////////////////////////////////////////////////////////

  private final PS4Controller driverController = new PS4Controller(0);
  


  private SwerveDrive swerve;
  // private ArmInterface arm;

  private Command autonomousCommand = new PrintCommand("default auto command! please override me!");
  
  /////////////////////////////////////////////////////////////////////////////
  //                                                                         //
  //                      Robot Scheduling and Stuff                         //
  //                                                                         //
  /////////////////////////////////////////////////////////////////////////////

  @Override
  public void robotInit() {
    configureButtonBindings();

    swerve.setDefaultCommand(new RunCommand(
      // TODO: change these to actually match real axis ports
      () -> swerve.joystickDrive(
        driverController.getLeftX(), 
        driverController.getLeftY(), 
        driverController.getRightY(), 
        DriveConstants.kFieldRelative
    ), swerve));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // arm.syncEncoders();
    swerve.syncEncoders();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand.schedule();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    autonomousCommand.cancel();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  /////////////////////////////////////////////////////////////////////////////
  //                                                                         //
  //                      Old Robot Container Stuffs                         //
  //                                                                         //
  /////////////////////////////////////////////////////////////////////////////

  private void configureButtonBindings() {
    Trigger leftBumper = new JoystickButton(driverController, OIConstants.kButtonLB);
    Trigger rightBumper = new JoystickButton(driverController, OIConstants.kButtonRB);

    // leftBumper.onTrue(arm.score()).onFalse(arm.stow());
    // rightBumper.onTrue(arm.intake()).onFalse(arm.stow());
  }
}

