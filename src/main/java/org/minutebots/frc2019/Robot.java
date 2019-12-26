package org.minutebots.frc2019;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import org.minutebots.lib.controller.PIDController;
import org.minutebots.lib.controller.RamseteController;
import org.minutebots.lib.controller.SimpleMotorFeedforward;
import org.minutebots.lib.geometry.Pose2d;
import org.minutebots.lib.geometry.Rotation2d;
import org.minutebots.lib.geometry.Translation2d;
import org.minutebots.lib.kinematics.DifferentialDriveKinematics;
import org.minutebots.lib.kinematics.DifferentialDriveOdometry;
import org.minutebots.lib.kinematics.DifferentialDriveWheelSpeeds;
import org.minutebots.lib.trajectory.Trajectory;
import org.minutebots.lib.trajectory.TrajectoryConfig;
import org.minutebots.lib.trajectory.TrajectoryGenerator;

import java.util.ArrayList;
import java.util.function.Supplier;

public class Robot extends TimedRobot {
    Encoder leftEncoder = new Encoder(0, 0);
    Encoder rightEncoder = new Encoder(0, 0);
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(0.0));
    private Supplier<Pose2d> getPose = new Supplier<Pose2d>() {

        public Pose2d get() {
            return m_odometry.update(new Rotation2d(gyroAngle()), leftEncoder.getRate(), rightEncoder.getRate());
        }

    };

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {

    }

    private Ramsete generateRamsete() {
        Pose2d start = new Pose2d();
        Pose2d end = new Pose2d();
        TrajectoryConfig config = new TrajectoryConfig(1.0, 1.0);
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);
        DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0);

        PIDController leftController = new PIDController(0, 0, 0);
        PIDController rightController = new PIDController(0, 0, 0);


        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);

        RamseteController controller = new RamseteController();

        return new Ramsete(trajectory, getPose, controller, feedforward, kinematics, this::wheelSpeeds, leftController, rightController, this::voltConsumer);
    }

    private DifferentialDriveWheelSpeeds wheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    private void voltConsumer(Double voltsLeft, Double voltsRight) {
      //We don't have a use for left and right volts as of the moment.
    }

    private double gyroAngle() {
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
