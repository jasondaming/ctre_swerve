package frc.robot.Util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;

public class RoboticPathing {
  // Load the path we want to pathfind to and follow
  public PathPlannerPath topPath = PathPlannerPath.fromPathFile("Top to Amp");
  public PathPlannerPath midPath = PathPlannerPath.fromPathFile("Mid to Amp");
  public PathPlannerPath botPath = PathPlannerPath.fromPathFile("Bot to Amp");

  // Key Poses to target
  public Pose2d topPose = new Pose2d(2.9, 6.8, Rotation2d.fromDegrees(-155.0));
  public Pose2d midPose = new Pose2d(2.9, 5.55, Rotation2d.fromDegrees(0));
  public Pose2d botPose = new Pose2d(2.1, 3.0, Rotation2d.fromDegrees(130));

  // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
  public PathConstraints constraints = new PathConstraints(
          TunerConstants.kSpeedAt12VoltsMps, 4.0,
          Math.PI * 1.5, Units.degreesToRadians(720));

  // Since AutoBuilder is configured, we can use it to build pathfinding commands
  public Command TopAmpRobotic = AutoBuilder.pathfindThenFollowPath(
          topPath,
          constraints,
          1.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command MidAmpRobotic = AutoBuilder.pathfindThenFollowPath(
          midPath,
          constraints,
          1.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command BotAmpRobotic = AutoBuilder.pathfindThenFollowPath(
          botPath,
          constraints,
          1.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command topRobotic = AutoBuilder.pathfindToPose(
          topPose,
          constraints,
          0.0, // Goal end velocity in meters/sec
          0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command midRobotic = AutoBuilder.pathfindToPose(
          midPose,
          constraints,
          0.0, // Goal end velocity in meters/sec
          0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );

  public Command botRobotic = AutoBuilder.pathfindToPose(
          botPose,
          constraints,
          0.0, // Goal end velocity in meters/sec
          0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
  );
}
