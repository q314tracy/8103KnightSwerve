Built for use by East Noble Knight Robotics, FRC team 8103 by Quentin Tracy of New Prairie Las Pumas, FRC team 2197.

Adapted Rev Robotics MAXswerve template for use.
    Project is heavily modified to use Rev hardware for motors and CANcoders for azimuth encoders.
    Project also uses hardware abstraction and polymorphism to manage the switch between simulated and real hardware.
    Real hardware implementations are untested.
    If issue with hardware IO abstraction are a problem, please let me know and we can work on moving the hardware back in the subsystem.
    Uses SparkMax PID control for drive velocity control.
        Characterization should be accurate, PID values in Configs file may need adjusted slightly to dampen large disturbances.
        Possibly can move PID control on-board to roboRIO if issues occur. Simulation results in excellent performance.
    Uses on-board roboRIO profiled PID control for azimuth.
        Characterization should be accurate, PID values again may need turned up in Constants.ModuleConstants to give more response.
    
System Identification
    To identify the direction of all componenets, including motors and encoders, identification routines must be performed.
    Check what drive ratio is installed in the modules. Adjust kDrivingMotorReduction with the correct ratio.
    Characterization should be adjusted automatically with ratio change.
    DO ALL OF THESE STEPS OFF OF THE GROUND UNLESS OTHERWISE NOTED!!!!!!!!!!!!!!!!!!!
    Follow the steps below or accept your destiny being in integration heck.
    STEP #1
        Check and DOUBLE check all CANids of all components in the subsystem
        Adjust them to your heart's content, but make ABSOLUTELY sure they are correct or you will have severe problems.
    STEP #2
        Azimuth (turn) motors MUST be rotating in the correct direction. CCW is correct direction.
        I have created a subroutine that rotates the motors in the correct theoretical positive direction.
        The subroutine can be ran as an autonomous command, simply choose the IDrun_rotatemodules and enable autonomous.
        If the direction of the motors are incorrect, invert the motors in the turning SparkMaxConfig located in the Configs file.
        Rerun the routine to confirm the direction.
    STEP #3
        Start up AdvantageScope with the robot connected to the driver's station, and import the layout contained in this project.
        Layout file name is AdvantageScope 12-10-2025.json
        Connect to the robot in AdvantageScope.
        Select the tab Module Positions.
        Select View -> Zoom to enabled range.
        Mouse over the trace and scroll out until the time scale is roughly 10 - 20 seconds.
        Perform the same command, IDrun_rotatemodules, and observe the rate of change in the trace.
        Rate of change should slope upwards to indicate encoder is increasing in value as the module rotates CCW.
        Some wrapping may occur as the value is wrapped from -pi, pi.
        Invert if needed using the kTurningEncoderInverted value in Constants.ModuleConstants.
    STEP #4
        Disable the robot and align the wheels as if you were zeroing the encoders.
        Face bevel gears to the left on all modules. Modules must be facing forward.
        Record the encoder values reported on the Module Position tab in AdvantageScope.
        Apply the inverse of the recorded value in Constants.DriveConstants for each encoder offset value.
    STEP #5
        Run the IDrun_drivemodules subroutine. and observe the direction of rotation.
        Modules should be pointing straight forward and driving forward.
        If modules are not pointing straight forward, restart from step #4. Module offset likely incorrect.
        If an individual module is driving in reverse, restart from step #1. CANids or module offset is incorrect.
        If all modules are driving in reverse, invert the drive module SparkMaxConfig in Configs.
    STEP #6
        If all IDruns are successful, put the robot on the ground and initiate the IDrun_rotatechassis routine.
        This will rotate the chassis in a CCW direction as long as your identification runs are correct.
        Observe the rate of change of the gyroscope angle. Rate of change should be positive and wrap from -180 to 180.
        Invert if incorrect using kGyroReversed in Constants.DriveConstants.

    If identification runs and gyroscope direction is correct, you are ready to run and operate the drivetrain!!!!!!