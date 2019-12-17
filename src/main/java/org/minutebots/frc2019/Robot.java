package org.minutebots.frc2019;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.minutebots.lib.controller.RamseteController;
import org.minutebots.lib.controller.SimpleMotorFeedforward;
import org.minutebots.lib.geometry.Pose2d;
import org.minutebots.lib.geometry.Rotation2d;
import org.minutebots.lib.geometry.Translation2d;
import org.minutebots.lib.kinematics.DifferentialDriveKinematics;
import org.minutebots.lib.kinematics.DifferentialDriveOdometry;
import org.minutebots.lib.trajectory.Trajectory;
import org.minutebots.lib.trajectory.TrajectoryConfig;
import org.minutebots.lib.trajectory.TrajectoryGenerator;
import org.minutebots.lib.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.Encoder;
import org.minutebots.lib.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
      
  }

  final Encoder leftEncoder = new Encoder(0, 0);
  final Encoder rightEncoder = new Encoder(0, 0);
  final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(0.0));

  private Ramsete generateRamsete(){

    final Pose2d start = new Pose2d();
    final Pose2d end = new Pose2d();
    final TrajectoryConfig  config = new TrajectoryConfig(1.0, 1.0);
    final ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
    final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0,0);
    final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0);

    final PIDController leftController = new PIDController(0,0,0);
    final PIDController rightController = new PIDController(0,0,0);


      final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
    
      final RamseteController controller = new RamseteController();
    
      return new Ramsete(trajectory, getPose, controller, feedforward, kinematics, this::wheelSpeeds, leftController, rightController, this::huh);
  }

  private Supplier<Pose2d> getPose = new Supplier<Pose2d>(){

    public Pose2d get() {
      return m_odometry.update(new Rotation2d(gyroAngle()), leftEncoder.getRate(), rightEncoder.getRate());
    }

  };

  private DifferentialDriveWheelSpeeds wheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(),rightEncoder.getRate());
  }
  private void huh(Double a, Double b){
    
  }

  private double gyroAngle(){
    return 0;
  }
  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
