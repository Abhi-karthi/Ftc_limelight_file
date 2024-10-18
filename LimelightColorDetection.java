package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Limelight Color Detection", group="Linear Opmode")
public class LimelightColorDetection extends MecanumTeleOpLimelight {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private final double KP = 0.05;  // Proportional constant for alignment control
    private Limelight3A limelight;

    @Override
    public void init() {
        // Initialize the motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Set motor direction
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Initialize the Limelight object
        limelight = new Limelight3A();
    }
    public static String getColor; {
        // Switch between pipelines for color detection
        int pipelineNumber = getPipeline();


        if (pipelineNumber == 0) {
            // Pipeline 0 is assumed to be set for yellow detection
            return "yellow";
  s      } else if (pipelineNumber == 1) {
            // Pipeline 1 is assumed to be set for blue detection
            return "blue";
        } else {
            // No valid color detected
            return "none";
        }
    }
}

    @Override
    public void loop() {
        // Get Limelight data
        double tx = limelight.getTx(); // Horizontal offset from crosshair to target
        double ty = limelight.getTy(); // Vertical offset from crosshair to target (used for distance)


        // Process color data
        String detectedColor = limelight.getColor(); // Assume limelight returns "yellow" or "blue"
        currentColor =


        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);

        if (hasTarget) {
            // Horizontal Alignment
            double horizontalAdjustment = KP * tx;

            // If the target is yellow or blue, align the robot
            if (detectedColor.equals("yellow") || detectedColor.equals("blue")) {
                mecanumDrive(horizontalAdjustment);
            }

            // Use ty (vertical offset) to calculate distance, if needed
            double distance = calculateDistance(ty); // You can implement this based on Limelight calibration
            telemetry.addData("Distance", distance);
        } else {
            // No valid target, stop the robot
            stopRobot();
        }

        telemetry.update();
    }

    // Helper function to drive the robot using mecanum wheels for horizontal alignment
    private void mecanumDrive(double strafePower) {
        frontLeftMotor.setPower(strafePower);
        frontRightMotor.setPower(-strafePower);
        backLeftMotor.setPower(-strafePower);
        backRightMotor.setPower(strafePower);
    }

    // Helper function to stop the robot
    private void stopRobot() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    // Simple distance calculation based on ty (can be tuned based on your field setup)
    private double calculateDistance(double ty) {
        // Example linear distance calculation, this will depend on your camera calibration
        return (20.0 - ty) * 0.1;  // Modify based on your needs
    }
}


