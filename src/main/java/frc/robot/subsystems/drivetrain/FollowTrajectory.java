package frc.robot.subsystems.drivetrain;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.autonomous.AutoTrajectories;
import frc.robot.subsystems.SwerveSubsystem;

public class FollowTrajectory extends SwerveControllerCommand {

    /**
     * Command to follow a given Trajectory using the SwerveControllerCommand class, which in turn uses HolonomicDriveController
     */

    private final Trajectory trajectory;
    private Timer timer = new Timer();
    private SwerveSubsystem drive;
    private boolean stopLocal = true;
 

    /*NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    NetworkTableEntry desiredXEntry;
    NetworkTableEntry desiredYEntry;
    NetworkTableEntry actualXEntry;
    NetworkTableEntry actualYEntry;*/
    


    //uses motion profiling versus standard PID for smoother heading tracking and to limit rotational speed
    private static final ProfiledPIDController rotationController =
        new ProfiledPIDController(1.5, .1
        , 0,   //.5 .1 0
             new TrapezoidProfile.Constraints(AutoConstants.kMaxRotationalVelocityMetersPerSecond,
                 AutoConstants.kMaxAccelerationMetersPerSecondSquared
            )
        );
        

    public FollowTrajectory(SwerveSubsystem drive, Trajectory trajectory, boolean stop) {

        /**
         * Super constructor for SwerveControllerCommand
         * Parameters: 
         * trajectory to be followed
         * method reference to the pose supplier
         * kinematics of the drive (wheel placements on robot)
         * x controller
         * y controller
         * rotation cont
         * roller
         * method reference to the module control method
         * requirements (drive subsystem)
         */
        super(
            trajectory, 
            drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(1.5, 0, 0), //1 0 0
            new PIDController(1.5, 0, 0), //''
            // new PIDController(0, 0, 0), //Temp 0 for testing
            // new PIDController(0, 0, 0), //Temp 0 for testing
            rotationController,
            drive::setModuleStates, 
            drive
        );
        
        stopLocal = stop;
        this.drive = drive;

        // set the rotation controller to wrap around from -PI to PI
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        this.trajectory = trajectory;
 
    
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        super.initialize();
        SmartDashboard.putNumber("Initialized", 1);
        System.out.println("Command '" + this.getName() + "' initialized at " + Timer.getFPGATimestamp());
    } 

    @Override
    public void execute() {
        // graph();
        super.execute();
    }

    public Pose2d getInitialPose() {
        
        return trajectory.getInitialPose();

    }

    // public void graph() {
    //     State desiredState = AutoTrajectories.backUp.sample(timer.get());
    //     SmartDashboard.putNumber("Desired x", Units.metersToInches(desiredState.poseMeters.getX()));
    //     SmartDashboard.putNumber("Desired y", Units.metersToInches(desiredState.poseMeters.getY()));

    //     Pose2d Currentposition = drive.getPose();
    //     SmartDashboard.putNumber("Actual x", Units.metersToInches(Currentposition.getX()));
    //     SmartDashboard.putNumber("Actual y", Units.metersToInches(Currentposition.getY()));
    // }


    @Override public void end(boolean isInterrupted){
        System.out.println("End FollowTrajectory/Interrupted: "+isInterrupted);
        System.out.println("Command '" + this.getName() + "' ended at " + Timer.getFPGATimestamp());
        if (stopLocal == true){
            drive.stopModules();
        }
    }
}