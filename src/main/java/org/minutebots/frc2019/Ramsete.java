package org.minutebots.frc2019;

import java.util.function.BiConsumer;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Timer;
import org.minutebots.lib.controller.PIDController;
import org.minutebots.lib.controller.RamseteController;
import org.minutebots.lib.controller.SimpleMotorFeedforward;
import org.minutebots.lib.geometry.Pose2d;
import org.minutebots.lib.kinematics.ChassisSpeeds;
import org.minutebots.lib.kinematics.DifferentialDriveKinematics;
import org.minutebots.lib.kinematics.DifferentialDriveWheelSpeeds;
import org.minutebots.lib.trajectory.Trajectory;

public class Ramsete {
    private final Timer m_timer = new Timer();
    private final boolean m_usePID;
    private final Trajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final RamseteController m_follower;
    private final SimpleMotorFeedforward m_feedforward;
    private final DifferentialDriveKinematics m_kinematics;
    private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
    private final PIDController m_leftController;
    private final PIDController m_rightController;
    private final BiConsumer<Double, Double> m_output;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;
  
    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * PID control and feedforward are handled internally, and outputs are scaled -12 to 12
     * representing units of volts.
     *
     * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
     * this
     * is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory      The trajectory to follow.
     * @param pose            A function that supplies the robot pose - use one of
     *                        the odometry classes to provide this.
     * @param controller      The RAMSETE controller used to follow the trajectory.
     * @param feedforward     The feedforward to use for the drive.
     * @param kinematics      The kinematics for the robot drivetrain.
     * @param wheelSpeeds     A function that supplies the speeds of the left and
     *                        right sides of the robot drive.
     * @param leftController  The PIDController for the left side of the robot drive.
     * @param rightController The PIDController for the right side of the robot drive.
     * @param outputVolts     A function that consumes the computed left and right
     *                        outputs (in volts) for the robot drive.
     */
    public Ramsete(Trajectory trajectory,
                          Supplier<Pose2d> pose,
                          RamseteController controller,
                          SimpleMotorFeedforward feedforward,
                          DifferentialDriveKinematics kinematics,
                          Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
                          PIDController leftController,
                          PIDController rightController,
                          BiConsumer<Double, Double> outputVolts) {
      m_trajectory = trajectory;
      m_pose = pose;
      m_follower = controller;
      m_feedforward = feedforward;
      m_kinematics = kinematics;
      m_speeds = wheelSpeeds;
      m_leftController = leftController;
      m_rightController = rightController;
      m_output = outputVolts;
      m_usePID = true;
    }
  
    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds
     * from the RAMSETE controller, and will need to be converted into a usable form by the user.
     *
     * @param trajectory            The trajectory to follow.
     * @param pose                  A function that supplies the robot pose - use one of
     *                              the odometry classes to provide this.
     * @param follower              The RAMSETE follower used to follow the trajectory.
     * @param kinematics            The kinematics for the robot drivetrain.
     * @param outputMetersPerSecond A function that consumes the computed left and right
     *                              wheel speeds.
     */
    public Ramsete(Trajectory trajectory,
                          Supplier<Pose2d> pose,
                          RamseteController follower,
                          DifferentialDriveKinematics kinematics,
                          BiConsumer<Double, Double> outputMetersPerSecond) {

      m_trajectory = trajectory;
      m_pose = pose;
      m_follower = follower;
      m_kinematics = kinematics;

      m_output = outputMetersPerSecond;
  
      m_feedforward = null;
      m_speeds = null;
      m_leftController = null;
      m_rightController = null;
  
      m_usePID = false;
  
    }
  
    
    public void initialize() {
      m_prevTime = 0;
      var initialState = m_trajectory.sample(0);
      m_prevSpeeds = m_kinematics.toWheelSpeeds(
          new ChassisSpeeds(initialState.velocityMetersPerSecond,
              0,
              initialState.curvatureRadPerMeter
                  * initialState.velocityMetersPerSecond));
      m_timer.reset();
      m_timer.start();
      if (m_usePID) {
        m_leftController.reset();
        m_rightController.reset();
      }
    }
  
    
    public void execute() {
      double curTime = m_timer.get();
      double dt = curTime - m_prevTime;
  
      var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
          m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));
  
      var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
      var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;
  
      double leftOutput;
      double rightOutput;
  
      if (m_usePID) {
        double leftFeedforward =
            m_feedforward.calculate(leftSpeedSetpoint,
                (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);
  
        double rightFeedforward =
            m_feedforward.calculate(rightSpeedSetpoint,
                (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);
  
        leftOutput = leftFeedforward
            + m_leftController.calculate(m_speeds.get().leftMetersPerSecond,
            leftSpeedSetpoint);
  
        rightOutput = rightFeedforward
            + m_rightController.calculate(m_speeds.get().rightMetersPerSecond,
            rightSpeedSetpoint);
      } else {
        leftOutput = leftSpeedSetpoint;
        rightOutput = rightSpeedSetpoint;
      }
  
      m_output.accept(leftOutput, rightOutput);
  
      m_prevTime = curTime;
      m_prevSpeeds = targetWheelSpeeds;
    }
  
    
    public void end(boolean interrupted) {
      m_timer.stop();
    }
  
    
    public boolean isFinished() {
      return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
    }
  }
