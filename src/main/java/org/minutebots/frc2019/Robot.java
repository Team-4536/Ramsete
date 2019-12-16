package org.minutebots.frc2019;

import org.minutebots.lib.controller.RamseteController;
import org.minutebots.lib.trajectory.Trajectory;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
      
  }

  private Ramsete generateRamsete(){
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end,
            TrajectoryConfig config);
    
    RamseteController controller = new RamseteController();
    
      return new Ramsete(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds, leftController, rightController, outputVolts);
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
