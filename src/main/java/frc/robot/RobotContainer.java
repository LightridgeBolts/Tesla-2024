// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();
  private final SlewRateLimiter slew_left_y = new SlewRateLimiter(0.5);
  private final SlewRateLimiter slew_left_x = new SlewRateLimiter(.5);
  private final SlewRateLimiter slew_right_x = new SlewRateLimiter(.5);

  private boolean directionNegate = false;


  // The driver's controller
  PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband((directionNegate) ? -(m_driverController.getLeftY() * .7) : (m_driverController.getLeftY() * .7), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((directionNegate) ? -(m_driverController.getLeftX() * .7) : (m_driverController.getLeftX() * .7), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((directionNegate) ? -(m_driverController.getRightX() * .7) : (m_driverController.getRightX() * .7), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

            /*
             * -MathUtil.applyDeadband((directionNegate) ? -(m_driverController.getLeftY()) : (m_driverController.getLeftY()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((directionNegate) ? -(m_driverController.getLeftX()) : (m_driverController.getLeftX()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband((directionNegate) ? -(m_driverController.getRightX()) : (m_driverController.getRightX()), OIConstants.kDriveDeadband)
             */

            m_arm.setDefaultCommand(new RunCommand(() -> m_arm.runAutomatic(), m_arm));

            // set the intake to stop (0 power) when no other command is running
            m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setPower(0.0), m_intake));
        
            // configure the launcher to stop when no other command is running
            m_launcher.setDefaultCommand(new RunCommand(() -> m_launcher.stopLauncher(), m_launcher));
          

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

    

  private void configureButtonBindings() {
    new JoystickButton(m_driverController, PS4Controller.Button.kSquare.value).whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    
    // set up arm preset positions
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kScoringPosition)));
    new Trigger(() ->m_driverController.getL2Axis()> Constants.OIConstants.kTriggerButtonThreshold).onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kIntakePosition)));
    
    new POVButton(m_driverController, 0).onTrue(new InstantCommand(() -> m_arm.setTargetPosition(Constants.Arm.kUnderChainPosition)));

  //  new JoystickButton(m_driverController,  PS4Controller.Button.kCircle.value)
   //     .onTrue(new InstantCommand(() -> directionNegate = !directionNegate));
    

    // intake controls (run while button is held down, run retract command once when the button is
    // released)
    new Trigger(
            () ->
                m_driverController.getR2Axis()
                    > Constants.OIConstants.kTriggerButtonThreshold)
        .whileTrue(new RunCommand(() -> m_intake.setPower(Constants.Intake.kIntakePower), m_intake))
        .onFalse(m_intake.retract());


    new JoystickButton(m_driverController, PS4Controller.Button.kTriangle.value)
        .whileTrue(new RunCommand(() -> m_launcher.ampMode()))
        .onFalse(new RunCommand(() ->m_launcher.disableAmpMode()));
    
    
    // launcher controls (button to pre-spin the launcher and button to launch)
    new JoystickButton(m_driverController, PS4Controller.Button.kR1.value)
        .whileTrue(new RunCommand(() -> m_launcher.runLauncher(), m_launcher));

    // This runs a shot
    new JoystickButton(m_driverController,  PS4Controller.Button.kCross.value)
        .onTrue(m_intake.feedLauncherTwo(m_launcher));


    // Buttons for manually adjusting the height of the robot
    new POVButton(m_driverController, 180).onTrue(moveBack(.3));
    new POVButton(m_driverController, 270).onTrue(new RunCommand(() -> Constants.Arm.kIntakePosition = Constants.Arm.kIntakePosition + .002));
    new POVButton(m_driverController, 90).onTrue(new RunCommand(() -> Constants.Arm.kIntakePosition = Constants.Arm.kIntakePosition - .002));
    
    // new POVButton(m_driverController, 180).toggleOnTrue(getMoveBackCommand());    
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getMoveBackCommand(double poseDistanceInitial, double poseDistanceFinal) {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory moveBack =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(poseDistanceInitial, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(poseDistanceFinal, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            moveBack,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(moveBack.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  public Command getAutonShootMoveBack() {
    return new SequentialCommandGroup(moveBack(1.5));
  }

  /*
  public Command getAutonShootMoveBack() {
    Command runLaunch = new RunCommand(() -> m_launcher.runLauncher());
    return new SequentialCommandGroup(moveBack(.1), runLaunch, moveBack(.2));
  }
  */

  public Command moveBack(double time) {
    Command newCommand =
        new Command() {
          private Timer m_timer;

          @Override
          public void initialize() {
            m_timer = new Timer();
            m_timer.start();
          }

          @Override
          public void execute() {
            m_robotDrive.drive(-.5, 0, 0, true, true);            
          //  setPower(1.0);
          }

          @Override
          public boolean isFinished() {
            if (m_timer.get() < time){
              m_robotDrive.drive(-.5, 0, 0, true, true);
            } 
            return m_timer.get() > time;
          }

          @Override
          public void end(boolean interrupted) {
            m_robotDrive.drive(0, 0, 0, true, false);
          }
        };
    newCommand.addRequirements(m_robotDrive);
    return newCommand;
  }



}
