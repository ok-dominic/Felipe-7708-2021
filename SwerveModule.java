/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX; 
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;


public class SwerveModule {
  private static final double kWheelRadius = 2.0;       //was 0.0508
  private static final int kEncoderResolution = 4096;
  private static final int kEncoderResolution2 = 2048;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration
      = 2 * Math.PI; // radians per second squared

  private final WPI_TalonFX m_driveMotor;              //was speedcontroller
  private final WPI_TalonFX m_turningMotor;

/*private final Encoder m_driveEncoder = new Encoder(0, 1 ); ignore these, was from original example
  private final Encoder m_turningEncoder = new Encoder(2, 3);  */
  
  public static Encoder m_magEncoder; 
  public static WPI_TalonFX m_driveEncoder;           ///2048 resolution

  
  private final PIDController m_drivePIDController = new PIDController(15.23, 0.01, 0.087);    

  //KP initialy set to 1
  private final ProfiledPIDController m_turningPIDController
      = new ProfiledPIDController(1, 0, 0,
      new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));  //was 1,0,0

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);    //1,3
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(4, .5);     //1,.5
//                                                              first param controls speed of z , second param mystery       
  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int aChannel, int bChannel) {
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);
    
    m_magEncoder = new Encoder (aChannel, bChannel);   //these was added as potential solution
    m_driveEncoder = new WPI_TalonFX(0);

    
    //                                               enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s) 
    m_driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.25));
    m_turningMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.25));

    m_driveMotor.set(m_drivePIDController.calculate(m_driveEncoder.get(), .5));
    m_turningMotor.set(m_turningPIDController.calculate(m_magEncoder.getDistance(), .5));

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_magEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution); //was driveencoder
    m_driveEncoder.set(2 * Math.PI * kWheelRadius / kEncoderResolution2);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)                                   mallory did it
    // divided by the encoder resolution.
    m_magEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution); // was turn encoder

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }
 
  public void periodic() throws InterruptedException{
    m_magEncoder.wait(500);
  }

  public boolean getDirection(){
    return m_magEncoder.getDirection();
  }

  public double getDistance(){
    return m_magEncoder.getDistance();
  }

  public void reset(){
    m_magEncoder.reset();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    //first param was mag encoder.getrate()
    return new SwerveModuleState(m_driveEncoder.getSelectedSensorVelocity(), new Rotation2d(m_magEncoder.get())); //was drive then turn encoder
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller. 
    

    final double driveOutput = m_drivePIDController.calculate(
        m_driveEncoder.getSelectedSensorVelocity(), state.speedMetersPerSecond); // was drive encoder, as mag
                     //was get rate
    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(
        m_magEncoder.get(), state.angle.getRadians()   // was turn encoder
    );

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
