// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Store the last requested arcadeDrive speed so PrintDriveTrainInfo can print it alongside measured rates and positions
  private double m_speed;
  private boolean m_debug = false;
  private double m_kP;
  

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
 
    resetEncoders();
    resetGyro();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_speed = xaxisSpeed;
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void initiateDriveStraightUsingOdometry(double kP){
    m_kP = kP;
    resetEncoders();
  }
  public void driveStraightUsingOdometry(double xaxisSpeed){
    // Be sure to initiate before first call to this.
    m_speed = xaxisSpeed;
    double left = getLeftDistanceInch();
    double right = getRightDistanceInch();
    SmartDashboard.putNumber("leftDist", left);
    SmartDashboard.putNumber("rightDist", right);
    double diff = left - right; 
    double adjustment = Math.abs(diff) * m_kP;
    adjustment = Math.min(0.2, adjustment);
    adjustment = Math.signum(diff) * adjustment;
    double rightSpeedFactor = 1 + adjustment;
    double leftSpeedFactor = 1 - adjustment;
    if (rightSpeedFactor * m_speed > 1.0){
      m_speed = 1.0 / rightSpeedFactor;
    }
    if (leftSpeedFactor * m_speed > 1.0){
      m_speed = 1.0 / leftSpeedFactor; 
    }
    double leftSpeed = m_speed * leftSpeedFactor;
    double rightSpeed = m_speed * rightSpeedFactor;

    SmartDashboard.putNumber("leftSpeed",  leftSpeed);
    SmartDashboard.putNumber("rightSpeed", rightSpeed);
    SmartDashboard.putNumber("encoderDiff", diff);
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    m_diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public double getArcadeDriveSpeed(){
    return m_speed;
  }
  /**
   * @return speed of left wheel in inches per second.  XRP encoders seem to return 7.056593687694E-311
   */
  public double getLeftRate(){
    return m_leftEncoder.getRate();
  }
  /**
   * @return speed of right wheel in inches per second
   */
  public double getRightRate(){
    return m_rightEncoder.getRate();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the XRP along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the XRP along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the XRP along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the XRP around the X-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the XRP around the Y-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the XRP around the Z-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }
  public double getGyroAngleRateZ(){
    return m_gyro.getRateZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  public void setDebug(boolean debug){
    if (debug && !m_debug){
      System.out.println("%Speed,LeftDistance,RightDistance,LeftRate,RightRate");
    }
    m_debug = debug;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_debug){
      System.out.println("%" + 
        getArcadeDriveSpeed() + "," +
        getLeftDistanceInch() + "," +
        getRightDistanceInch() + "," +
        getLeftRate() + "," +  // the 'rates' are always very small numbers: 7.056593687694E-311
        getRightRate());
    }
  }
}
