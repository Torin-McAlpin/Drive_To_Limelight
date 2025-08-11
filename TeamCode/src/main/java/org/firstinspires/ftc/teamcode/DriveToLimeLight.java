package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.FeedbackType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDElement;
@Config
@TeleOp
public class DriveToLimeLight extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;

    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private Limelight3A ll;

    private PIDElement yawPID;
    private PIDElement axialPID;
    private PIDElement lateralPID;
    int lastLength = 0;
    double axial = 0.0;
    double lateral = 0.0;
    double yaw = 0.0;

    public static PIDCoefficients yawPIDCoefficients = new PIDCoefficients(0.05,0.0,0.0);
    public static PIDCoefficients axialPIDCoefficients = new PIDCoefficients(0.9,0.0,0.0005);
    public static PIDCoefficients lateralPIDCoefficients = new PIDCoefficients(-0.3,0.0,0.00001);

    public static double axialOffset = 1.0;
    public static double lateralOffset = 0.0;
    public static double yawOffset = 0.0;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        yawPID = new PIDElement(FeedbackType.POSITION, yawPIDCoefficients);
        axialPID = new PIDElement(FeedbackType.POSITION, axialPIDCoefficients);
        lateralPID = new PIDElement(FeedbackType.POSITION, lateralPIDCoefficients);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "LF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LB");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RF");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RB");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        ll = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        telemetry.setMsTransmissionInterval(11);

        ll.pipelineSwitch(0);


        /*
         * Starts polling for data.
         */
        ll.start();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.a) {
                aprilTagDrive();
            } else {
                gamepadDrive();
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: $runtime");
            telemetry.update();
        }
    }

    public void gamepadDrive() {
        double max;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate
                //
        axial = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
        lateral = gamepad1.left_stick_x;
        yaw = gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = max(abs(leftFrontPower), abs(rightFrontPower));
        max = max(max, abs(leftBackPower));
        max = max(max, abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }

    public void aprilTagDrive() {
        List<FiducialResult> fiducialResults = ll.getLatestResult().getFiducialResults();

        if (!fiducialResults.isEmpty()) {
            FiducialResult snapshot = fiducialResults.get(0);

            //if (lastLength != fiducialResults.size()) {
                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                axial = axialPID.calculate(new KineticState(-axialOffset - snapshot.getRobotPoseTargetSpace().getPosition().z)); // Note: pushing stick forward gives negative value
                lateral = lateralPID.calculate(new KineticState(-lateralOffset + snapshot.getRobotPoseTargetSpace().getPosition().x));
                yaw = yawPID.calculate(new KineticState(-yawOffset + snapshot.getRobotPoseTargetSpace().getOrientation().getYaw() -1.5));
            /*
            } else {
                axial = 0.0;
                lateral = 0.0;
                yaw = 0.0;
            }

             */
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            telemetry.addData("Axial error:", -axialOffset - snapshot.getRobotPoseTargetSpace().getPosition().z);
            telemetry.addData("Lateral error:", -lateralOffset + snapshot.getRobotPoseTargetSpace().getPosition().x);
            telemetry.addData("Yaw error:", -yawOffset + snapshot.getRobotPoseTargetSpace().getOrientation().getYaw() - 1.5);
            telemetry.addData("Axial output:", axial);
            telemetry.addData("Lateral output:", lateral);
            telemetry.addData("Yaw output:", yaw);
            telemetry.addData("Axial:", -snapshot.getRobotPoseTargetSpace().getPosition().z);
            telemetry.addData("Lateral:", snapshot.getRobotPoseTargetSpace().getPosition().x);
            telemetry.addData("Yaw:", snapshot.getRobotPoseTargetSpace().getOrientation().getYaw());
        }
        lastLength = fiducialResults.size();
    }
}