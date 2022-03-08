// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import org.photonvision.PhotonCamera;
//import org.photonvision.PhotonUtils;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  Compressor Compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  boolean enabled = Compressor.enabled();
  boolean pressureSwitch = Compressor.getPressureSwitchValue();
  // private final CANSparkMax m_leftDrive = new CANSparkMax(0);
  private CANSparkMax brmotor = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax blmotor = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax frmotor = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax flmotor = new CANSparkMax(1, MotorType.kBrushless);
  private VictorSPX intake = new VictorSPX(6);
  private TalonSRX intake2 = new TalonSRX(8);
  private VictorSPX thruPut = new VictorSPX(9);
  private CANSparkMax shooter = new CANSparkMax(4, MotorType.kBrushless);
  private VictorSPX intakeLift = new VictorSPX(10);

  DoubleSolenoid upSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid sideSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  private final XboxController joe = new XboxController(1);
  private final Timer m_timer = new Timer();
  private MecanumDrive m_drive = new MecanumDrive(flmotor, blmotor, frmotor, brmotor);

 // private PhotonCamera camera = new PhotonCamera("photonvision");

  double leftrig = 0;
  double righttrig = 0;

  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  // TODO: measure this
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.inchesToMeters(114);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Compressor.enableDigital();
    Compressor.enabled();
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    shooter.setIdleMode(IdleMode.kBrake);
  }

  /** This function is called periodically during teleoperated mode. */

  public double absl(double in) {
    if (in < 0.0) {
      return -in;
    } else {
      return in;
    }
  }

  @Override
  public void teleopPeriodic() {
    double forwardSpeed;
    double rotationSpeed;

    final double DeadZone = 0.05;

    double xs = joe.getLeftX() / 2;
    double ys = joe.getLeftY() / 2;
    double zr = joe.getRightX() / 2;

    // if(absl(xs)< DeadZone){
    // xs = 0.0;
    // }
    // if(absl(ys)<DeadZone){ys=0.0;}

    // if(absl(zr)<DeadZone){zr=0.0;}
    /*
    if (joe.getLeftStickButton()) {
      var result = camera.getLatestResult();

      if (result.hasTargets()) {
        // First calculate range
        double range = PhotonUtils.calculateDistanceToTargetMeters(
            CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS,
            CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(result.getBestTarget().getPitch()));

        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range
        forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

        // Also calculate angular power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
      } else {
        // If we have no targets, stay still.
        forwardSpeed = 0;
        rotationSpeed = 0;
      }
      m_drive.driveCartesian(-forwardSpeed, 0, rotationSpeed);
    } else {
      m_drive.driveCartesian(-ys, xs, zr);
    } 
    */
    m_drive.driveCartesian(-xs, ys, zr);

    // m_drive.driveCartesian(ySpeed, xSpeed, zRotation);

    leftrig = joe.getLeftTriggerAxis();
    righttrig = joe.getRightTriggerAxis();

    // thruPutLow.set(leftrig);

    // thruPutHi.set(-righttrig);
    if(joe.getYButton()){
      thruPut.set(ControlMode.PercentOutput,.5);
    }
    else{
      thruPut.set(ControlMode.PercentOutput, 0);
    }
    if (joe.getAButton())
      shooter.set(-1);
    // if (joe.getYButton())
    // shooter.set(-.80);
    if (joe.getXButton())
      shooter.set(0);
    if (joe.getBButton()) {
      intake.set(ControlMode.PercentOutput, .5);
      intake2.set(ControlMode.PercentOutput,-.5);
    } else {
      intake.set(ControlMode.PercentOutput, 0.0);
      intake2.set(ControlMode.PercentOutput,0.0);
    }
    if(joe.getLeftBumperPressed()){
       boolean upFired = false;
      if(upFired){
        upSolenoid.set(Value.kForward);
        upFired = true;
      }
      else{
        upSolenoid.set(Value.kReverse);
        upFired = false;
      }
      if(joe.getRightBumperPressed()){
        boolean sideFired = false;
       if(sideFired){
        sideSolenoid.set(Value.kForward);
         sideFired = true;
       }
       else{
         sideSolenoid.set(Value.kReverse);
         sideFired = false;
       }
      }
    }
    intakeLift.set(ControlMode.PercentOutput, joe.getLeftTriggerAxis() - joe.getRightTriggerAxis());
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_drive.driveCartesian(.5, 0, 0);
    // yeet.set(ControlMode.PercentOutput,
    // joe.getLeftTriggerAxis()-joe.getRightTriggerAxis());

  }
}
