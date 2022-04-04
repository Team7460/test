// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  private CANSparkMax blmotor = new CANSparkMax(5, MotorType.kBrushless);
  private CANSparkMax frmotor = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax flmotor = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax Launcher1 = new CANSparkMax(11, MotorType.kBrushless);
  private CANSparkMax Launcher2 = new CANSparkMax(12, MotorType.kBrushless);

  private VictorSPX intake = new VictorSPX(6);
  private TalonSRX intake2 = new TalonSRX(8);
  private VictorSPX thruPut = new VictorSPX(9);
  private CANSparkMax thruPutFlywheel = new CANSparkMax(4, MotorType.kBrushless);
  private VictorSPX intakeLift = new VictorSPX(10);

  DoubleSolenoid rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  DoubleSolenoid leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
  DoubleSolenoid sideSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  DoubleSolenoid hardStopSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

  private final XboxController joe = new XboxController(1);
  private final XboxController controller2 = new XboxController(0);
  private final Timer m_timer = new Timer();
  private MecanumDrive m_drive = new MecanumDrive(flmotor, blmotor, frmotor, brmotor);
  private boolean lastXWasPositive = false;

  double leftrig = 0;
  double righttrig = 0;

  double min_command = 0.075;
  private static final double Kp = 0.01;

  private SparkMaxPIDController launcher1PidController;

  private SparkMaxPIDController launcher2PidController;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    PortForwarder.add(5800, "photonvision.local", 5800);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);

    Compressor.enableDigital();
    Compressor.enabled();

    launcher1PidController = Launcher1.getPIDController();
    launcher2PidController = Launcher2.getPIDController();

    launcher1PidController.setP(6e-4);
    launcher1PidController.setI(0);
    launcher1PidController.setD(0);
    launcher1PidController.setIZone(0);
    launcher1PidController.setFF(0.000015);
    launcher1PidController.setOutputRange(-1, 1);

    launcher2PidController.setP(6e-4);
    launcher2PidController.setI(0);
    launcher2PidController.setD(0);
    launcher2PidController.setIZone(0);
    launcher2PidController.setFF(0.000015);
    launcher2PidController.setOutputRange(-1, 1);

    // forwardController.setTolerance(2.5);
    // turnController.setTolerance(2.5);
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
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    // read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double targets = tv.getDouble(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);

    limelightTarget(x, y, targets);

    if (Math.abs(Launcher1.get()) > 0.1) {
      hardStopSolenoid.set(Value.kReverse);
    } else {
      hardStopSolenoid.set(Value.kForward);
    }

    /*
     * 
     * // post to smart dashboard periodically
     * SmartDashboard.putNumber("LimelightX", x);
     * SmartDashboard.putNumber("LimelightY", y);
     * SmartDashboard.putNumber("LimelightArea", area);
     * 
     * double turnCalculation = turnController.calculate(x, 0);
     * double forwardCalculation = forwardController.calculate(y, 0);
     * System.out.println(targets);
     * 
     * if(targets == 0 || (turnController.atSetpoint() &&
     * forwardController.atSetpoint())){
     * forwardCalculation = 0.0;
     * turnCalculation = 0.0;
     * }
     * 
     * 
     * 
     * if(targets == 0){
     * //if(lastXWasPositive){
     * turnCalculation = 0.1;
     * //} else {
     * // turnCalculation = -0.3;
     * //}
     * }
     * 
     * if(x > 0) {
     * lastXWasPositive = true;
     * } else {
     * lastXWasPositive = false;
     * }
     * 
     * System.out.println("turncalc: " + turnCalculation);
     * System.out.println("forwardcalc: " + forwardCalculation);
     */
    /*
     * double turn = 0;
     * double err = 1;
     * System.out.println("Current X: "+x);
     * System.out.println("current Area: "+area);
     * //double forward =0;
     * //check if drivetrain should turn
     * if(x>err){
     * turn =.10;
     * }
     * else if(x<-err){
     * turn = -.10;
     * }
     * else{
     * turn = 0.0;
     * }
     * 
     * m_drive.driveCartesian(turn, 0, 0);
     */
    // m_drive.driveCartesian(-turnCalculation, forwardCalculation, 0);

    // Spin up the shooter for 3 seconds
    // if (m_timer.get() > 0 && m_timer.get() < 3) {
    // shooter.set(-0.6);
    // Run the shooter and the thruput for 7 more seconds to flush out the ball
    // } else if (m_timer.get() > 3 && m_timer.get() < 10) {
    // shooter.set(-0.6);
    // thruPut.set(ControlMode.PercentOutput, .5);
    // } else {
    // Stop the shooter and throughput
    // shooter.set(0);
    // thruPut.set(ControlMode.PercentOutput, 0);
    // }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    thruPutFlywheel.setIdleMode(IdleMode.kBrake);
    hardStopSolenoid.set(Value.kForward);

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
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    // read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double targets = tv.getDouble(0);
    // double forwardSpeed;
    // double rotationSpeed;

    // final double DeadZone = 0.05;

    double xs = controller2.getLeftX();
    double ys = controller2.getLeftY();
    double zr = controller2.getRightX();

    // if(absl(xs)< DeadZone){
    // xs = 0.0;
    // }
    // if(absl(ys)<DeadZone){ys=0.0;}

    // if(absl(zr)<DeadZone){zr=0.0;}
    /*
     * if (joe.getLeftStickButton()) {
     * var result = camera.getLatestResult();
     * 
     * if (result.hasTargets()) {
     * // First calculate range
     * double range = PhotonUtils.calculateDistanceToTargetMeters(
     * CAMERA_HEIGHT_METERS,
     * TARGET_HEIGHT_METERS,
     * CAMERA_PITCH_RADIANS,
     * Units.degreesToRadians(result.getBestTarget().getPitch()));
     * 
     * // Use this range as the measurement we give to the PID controller.
     * // -1.0 required to ensure positive PID controller effort _increases_ range
     * forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
     * 
     * // Also calculate angular power
     * // -1.0 required to ensure positive PID controller effort _increases_ yaw
     * rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(),
     * 0);
     * } else {
     * // If we have no targets, stay still.
     * forwardSpeed = 0;
     * rotationSpeed = 0;
     * }
     * m_drive.driveCartesian(-forwardSpeed, 0, rotationSpeed);
     * } else {
     * m_drive.driveCartesian(-ys, xs, zr);
     * }
     */
    if (controller2.getLeftBumper()) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      limelightTarget(x, y, targets);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      m_drive.driveCartesian(-xs, ys, -zr);
    }

    // flmotor.set(xs + ys - zr);
    // frmotor.set(xs - ys - zr);
    // brmotor.set(-xs + ys + zr);
    // blmotor.set(-xs -ys + zr);
    // m_drive.driveCartesian(ySpeed, xSpeed, zRotation);

    leftrig = controller2.getLeftTriggerAxis();
    righttrig = controller2.getRightTriggerAxis();

    // thruPutLow.set(leftrig);

    // thruPutHi.set(-righttrig);
    if (joe.getRightBumper()) {
      thruPut.set(ControlMode.PercentOutput, .5);
    } else if (joe.getBButton()) {
      thruPut.set(ControlMode.PercentOutput, -0.3);
      intake.set(ControlMode.PercentOutput, -0.3);
      intake2.set(ControlMode.PercentOutput, 0.3);
    } else {
      thruPut.set(ControlMode.PercentOutput, 0);
    }
    if (joe.getAButton()) {
      thruPutFlywheel.set(-.60);

      launcher1PidController.setReference(-4.5, CANSparkMax.ControlType.kVoltage);
      launcher2PidController.setReference(4.5, CANSparkMax.ControlType.kVoltage);
      hardStopSolenoid.set(Value.kReverse);

      // Launcher1.set(-.5);
      // Launcher2.set(.5);
    }

    if (joe.getYButton()) {
      thruPutFlywheel.set(-.2);
      Launcher1.set(-.3);
      Launcher2.set(.3);
      hardStopSolenoid.set(Value.kReverse);

    }

    if (joe.getXButton()) {
      hardStopSolenoid.set(Value.kForward);

      thruPutFlywheel.set(0);
      Launcher1.set(0);
      Launcher2.set(0);
    }

    if (joe.getLeftBumper()) {
      intake.set(ControlMode.PercentOutput, .5);
      intake2.set(ControlMode.PercentOutput, -.5);
    } else if (!joe.getBButton()) {
      intake.set(ControlMode.PercentOutput, 0);
      intake2.set(ControlMode.PercentOutput, 0);
    }

    if (controller2.getYButton()) {
      System.out.println("Going up");
      rightSolenoid.set(Value.kForward);
      leftSolenoid.set(Value.kForward);
    }
    if (controller2.getAButton()) {
      System.out.println("Going down");
      rightSolenoid.set(Value.kReverse);
      leftSolenoid.set(Value.kReverse);
    }
    if (controller2.getXButton()) {
      System.out.println("side on");
      sideSolenoid.set(Value.kForward);
    }
    if (controller2.getBButton()) {
      System.out.println("side off");
      sideSolenoid.set(Value.kReverse);
    }

    intakeLift.set(ControlMode.PercentOutput, joe.getRightTriggerAxis() - joe.getLeftTriggerAxis());

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // Launcher1.set(.5);
    // Launcher2.set(-.5);
    // brmotor.set(.1);
    // brmotor.set(.5);
    thruPutFlywheel.set(-.3);
    Launcher1.set(1);
    Launcher2.set(-1);
    // m_drive.driveCartesian(.5, 0, 0);
    // yeet.set(ControlMode.PercentOutput,
    // joe.getLeftTriggerAxis()-joe.getRightTriggerAxis());

  }

  public boolean limelightTarget(double x, double y, double targets) {
    if (targets == 0)
      return true;

    double x_error = -x;

    if (Math.abs(x_error) < 3)
      return true;

    m_drive.driveCartesian(0, 0, x_error * Kp);
    return false;
  }
}
