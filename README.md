# CTRE Swerve Example

Features:
- Improved Limelight support
- Improved PathPlanner autonomous setup
- Added SysID options for running swerve drivetrain characterization
- Different xbox joystick controller layout to try for driver comfort
- Easily scale down the max speed to help newer drivers learn
- "Turtle Mode" (left bumper) temporarily slows the drivetrain down for fine adjustments

This is an expanded version of the CTRE [SwerveWithPathPlanner](https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithPathPlanner) example using the CTRE Swerve Builder.  To use it copy the generated/TunerConstants.java file from the generated project and replace the generated/TunerConstants.java file in this project.  You will also need to set your team number.

To use the limelight ensure you have a 36h11 pipeline properly configured and then change the [enable constant](https://github.com/jasondaming/ctre_swerve/blob/master/src/main/java/frc/robot/Vision/Limelight.java#L22) to true.

Will add specific steps on characterization after testing.
