package frc.robot.subsystems.drivetrain;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.*;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class SwerveModule {

    // keeping all the variables (objects) in da class private and constant

    // SwerveModule for ONE full SwerveWheel
    // we got the drive motor for moving forward and backward
    // steer motor for turning

    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    // 2 encoders for drive and steer motor feedback
    // hella methods for shit like velocity, position, error feedback, and shi
    
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    // absolute is constant, its the starting offset of the wheels
    // we wanna keep that shit constant
    // relative offset is how far its moved from the absolute/starting point

    private final Rotation2d absoluteOffset;
    private Rotation2d relativeOffset;

    private final SparkMaxPIDController PIDDrive;
    private final SparkMaxPIDController PIDSteer;
    
    // current state of motors
    // speed and angle of motors

    private SwerveModuleState relativeState;



    public SwerveModule(int drivePort, int steerPort) {
        
        // constructor
        
        this.driveMotor = new CANSparkMax(drivePort, MotorType.kBrushless);
        this.steerMotor = new CANSparkMax(steerPort, MotorType.kBrushless);

        // fuck factory defaults
        // already configured in REV Hardware Client
        
        // 2 Idle Modes: brake or coast
        // brake will actively use electrical power to stop the motor
        // coast will stop power output to the motors 
        // brake is bootycheeks, jus coast, cus elevator go down if brake

        this.driveMotor.setIdleMode(IdleMode.kCoast);
        this.steerMotor.setIdleMode(IdleMode.kCoast);
        
        // encoders
        // the CAN motor obj give us the getEncoder objs for us

        // get the whole drum cus im generous
        // put your name on the bullets when im fillin em pussy


        this.driveEncoder = driveMotor.getEncoder();
        this.steerEncoder = steerMotor.getEncoder();


        // PID shit
        // check the doc cus im not aboutta yap about PID controllers
        // on fackin vscode 
        
        this.PIDDrive = driveMotor.getPIDController();
        this.PIDSteer = steerMotor.getPIDController();
        

        this.relativeState = new SwerveModuleState();

        // sets angle and speed of steer and drive motors to 0 degrees and 0 meters per second respectively 
        // no arguements passed cus the mf does it by default
        // right click and click 'go to definition' for more details

        absoluteOffset = Constants.SwerveConstants.kSteerEncoderOffsets[0];


    }

    @Override
    public void periodic() {


    }

}
