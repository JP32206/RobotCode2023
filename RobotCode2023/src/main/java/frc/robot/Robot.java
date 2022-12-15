// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;

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
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // added definisions

  private final XboxController controller = new XboxController(0);

  PIDController pid = new PIDController(0.01, 0, 0);
  
  private AnalogGyro gyro = new AnalogGyro(kGyroPort);//fix

  // motor names
  private final WPI_TalonFX motor_FRang = new WPI_TalonFX(3);
  private final WPI_TalonFX motor_FRmag = new WPI_TalonFX(2);

  private final WPI_TalonFX motor_FLang = new WPI_TalonFX(8);
  private final WPI_TalonFX motor_FLmag = new WPI_TalonFX(4);

  private final WPI_TalonFX motor_RRang = new WPI_TalonFX(1);
  private final WPI_TalonFX motor_RRmag = new WPI_TalonFX(6);

  private final WPI_TalonFX motor_RLang = new WPI_TalonFX(5);
  private final WPI_TalonFX motor_RLmag = new WPI_TalonFX(7);

  CANCoder FR_coder = new CANCoder(9);
  CANCoder FL_coder = new CANCoder(11);
  CANCoder RR_coder = new CANCoder(10);
  CANCoder RL_coder = new CANCoder(12);

  // motor controll vectors

  private double FRX;
  private double FRY;

  private double FLX;
  private double FLY;

  private double RRX;
  private double RRY;

  private double RLX;
  private double RLY;
  
  private double angSpeedMax = 0.2;
  private double magSpeedMax = 0.5;
  private double xMax = 0.40;
  private double yMax = 0.40;
  private double zMax = 0.40;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

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
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    pid.enableContinuousInput(0, 360);
    FR_coder.setPositionToAbsolute();
    FR_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    FR_coder.setDirection(true);
    FR_coder.configMagentOffset(-192);
    FL_coder.setPositionToAbsolute();
    FL_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    FL_coder.setDirection(true);
    FL_coder.configMagentOffset(-90);
    RR_coder.setPositionToAbsolute();
    RR_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);    
    RR_coder.setDirection(true);
    RR_coder.configMagentOffset(-23);
    RL_coder.setPositionToAbsolute();
    RL_coder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    RL_coder.setDirection(true);
    RL_coder.configMagentOffset(-55);
    gyro.calibrate();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    drive(controller.getLeftX() * xMax, controller.getLeftY() * yMax, controller.getRightX() * zMax);

  }

  private void drive(double x, double y, double z) {
    
    controlerAng = Math.atan2(y,x) - gyro.getAngle()
    controlerMag = Math.hypot(x,y)
    x = controlerMag * Math.cos(controlerAng);
    y = controlerMag * Math.sin(controlerAng);
    
    FRX = x + (z *  0.707);
    FRY = y + (z * -0.707);

    FLX = x + (z *  0.707);
    FLY = y + (z *  0.707);

    RRX = x + (z * -0.707);
    RRY = y + (z * -0.707);

    RLX = x + (z * -0.707);
    RLY = y + (z *  0.707);
    
    // set front right motors
    motor_FRang.set(pid.calculate(-FR_coder.getAbsolutePosition()),Math.toDegrees(Math.atan2(FRY, FRX))*angSpeedMax);
    motor_FRmag.set((Math.hypot(FRY, FRX)/2.827) + (-0.36 * motor_FRang.get()));

    // set front left motors
    motor_FLang.set(pid.calculate(-FL_coder.getAbsolutePosition()),Math.toDegrees(Math.atan2(FLY, FLX))*angSpeedMax);
    motor_FLmag.set((Math.hypot(FLY, FLX)/2.827) + (-0.36 * motor_FLang.get()));

    // set rear right motors
    motor_RRang.set(pid.calculate(-RR_coder.getAbsolutePosition()),Math.toDegrees(Math.atan2(RRY, RRX))*angSpeedMax);
    motor_RRmag.set((Math.hypot(RRY, RRX)/2.827) + (-0.36 * motor_RRang.get()));

    // set rear left motors
    motor_RLang.set(pid.calculate(-RL_coder.getAbsolutePosition()),Math.toDegrees(Math.atan2(RLY, RLX))*angSpeedMax);
    motor_RLmag.set((Math.hypot(RLY, RLX)/2.827) + (-0.36 * motor_RLang.get()));
  }
  
  private void moduleDrive(WPI_TalonFX angMotor,WPI_TalonFX magMotor,double encoderAng,double x,double y){
  angle = Math.toDegrees(Math.atan2(FRY, FRX))
  magnitude = Math.hypot(x,y)/2.827
  if(distance(angle,encoderAng)>90){
    angle += 180
    magnitude = -magnitude
  }
  angMotor.set(pid.calculate(encoderAng,angle) * angSpeedMax);
  magMotor.set((magnitude * magSpeedMax) + (-0.36 * angMotor.get()));
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
