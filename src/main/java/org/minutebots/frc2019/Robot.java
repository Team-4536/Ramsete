package org.minutebots.frc2019;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import org.minutebots.lib.controller.RamseteController;
import org.minutebots.lib.geometry.Pose2d;
import org.minutebots.lib.geometry.Rotation2d;
import org.minutebots.lib.geometry.Translation2d;
import org.minutebots.lib.kinematics.DifferentialDriveKinematics;
import org.minutebots.lib.kinematics.DifferentialDriveOdometry;
import org.minutebots.lib.trajectory.TrajectoryConfig;
import org.minutebots.lib.trajectory.TrajectoryGenerator;
import java.util.ArrayList;
public class Robot extends TimedRobot {

    /**
     * This code runs on a virtual robot environment on the driver station PC, and communicates with a Roomba through NetworkTables.
     * While performance offboard may have more latency, this is the only way I know of getting NI Driverstation Packets, through the loopback DS socket.
     *
     *We'll set up NetworkTable values to the Roomba.
     **/

    static { //static blocks cursed?
        NetworkTableInstance ntInstance = NetworkTableInstance.create();
        ntInstance.startClient("frcvision.local");
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable("Roomba");
    NetworkTableEntry leftMotor = table.getEntry("leftMotor"),
            rightMotor = table.getEntry("rightMotor"),
            leftRate = table.getEntry("leftRate"),
            rightRate = table.getEntry("rightRate"),
            angle = table.getEntry("angle");

    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d(angle.getDouble(0)));
    
    /**
     * The Roomba is actually smart enough to follow paths given raw wheel speeds, so we'll use the simplified RAMSETE constructor.
     *
     * Also, I've finally decided to switch to the metric system, so all the lengths/speeds are in metres.
     **/

    //TRAJECTORY CONSTRAINTS
    TrajectoryConfig config = new TrajectoryConfig(0.5, 0.2);

    //TRAJECTORY WAYPOINTS
    Pose2d start = new Pose2d();
    Pose2d end = new Pose2d(); //TODO: Add a test trajectory
    ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();

    Ramsete ramsete = new Ramsete(
            TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config),
            () -> m_odometry.update(new Rotation2d(angle.getDouble(0)), leftRate.getDouble(0), rightRate.getDouble(0)),
            new RamseteController(),
            new DifferentialDriveKinematics(0.235), //Taken off iCreate2 Manual,
            (left, right) -> {
                //After running through calculations, this block will be executed by the Ramsete Controller, turning the wheels.
                leftMotor.setDouble(left);
                rightMotor.setDouble(right);
            });

    @Override
    public void autonomousInit() {
        ramsete.initialize();
    }

    @Override
    public void autonomousPeriodic() {
        if (!ramsete.isFinished()) ramsete.execute();
    }
}
