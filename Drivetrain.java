/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; 
import frc.robot.SwerveModule;

//import edu.wpi.first.hal.DIOJNI;

/**                                                                                                     I've never thought of eating people 
 * Represents a swerve drive style drivetrain.                                                                Oh my gosh our whole week is set
 */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);     
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);         
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);            
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);    

                              // parameters   driveport, turnport, a encoder port, b encoder port
                                                                                                
  private final SwerveModule m_frontLeft = new SwerveModule(3, 2, 0, 1);            //back to original now             
  private final SwerveModule m_frontRight = new SwerveModule(12, 13, 4, 5);
  private final SwerveModule m_backLeft = new SwerveModule(0, 1, 2, 3); //7 - 15   8-14   
  private final SwerveModule m_backRight = new SwerveModule(15, 14, 6, 7);

  private  AnalogGyro m_gyro = new AnalogGyro(1);
                                                                                                  
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());

  public Drivetrain() {
    m_gyro.reset();
    /*m_frontLeft.reset();  //do these go here??
    m_frontRight.reset();
    m_backLeft.reset();
    m_backRight.reset(); */

  }

  public void periodic() {
    Shuffleboard.getTab("Encoders").addBoolean("Direction",() -> m_frontLeft.getDirection());
    Shuffleboard.getTab("Encoders").addBoolean("Direction",() -> m_frontRight.getDirection());
    Shuffleboard.getTab("Encoders").addBoolean("Direction",() -> m_backLeft.getDirection());
    Shuffleboard.getTab("Encoders").addBoolean("Direction",() -> m_backRight.getDirection());


    SmartDashboard.putNumber("Front Left Distance", m_frontLeft.getDistance());
    SmartDashboard.putNumber("Front Right Distance", m_frontRight.getDistance());
    SmartDashboard.putNumber("Back Left Distance", m_backLeft.getDistance());            
    SmartDashboard.putNumber("Back Right Distance", m_backRight.getDistance());
  
    SmartDashboard.putBoolean("Front Left Direction", m_frontLeft.getDirection());
    SmartDashboard.putBoolean("Front Right Direction", m_frontRight.getDirection());
    SmartDashboard.putBoolean("Back Left Direction", m_backLeft.getDirection());
    SmartDashboard.putBoolean("Back Right Direction", m_backRight.getDirection());

  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */    
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.   ?? gyro
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_odometry.update(
        getAngle(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
    );
  }
}
