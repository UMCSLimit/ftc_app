package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Test", group = "")
public class Test extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    private DcMotor motorL;
    private DcMotor motorR;
    private DcMotor motorRamie;
    private DcMotor motorSznur;
    static final double     COUNTS_PER_MOTOR_REV    = 288 ;
    static final double     DRIVE_GEAR_REDUCTION    = 0.66 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54 / 2 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.5;

    public double toIn(double Cm){
        return Cm/2.54;
    }

    @Override
    public void runOpMode() {
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");
        motorRamie = hardwareMap.dcMotor.get("motorRamie");
        motorSznur = hardwareMap.dcMotor.get("motorSznur");

        waitForStart();
        if (opModeIsActive()) {
            motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                motorL.getCurrentPosition(),
                motorR.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  toIn(10),  toIn(10), 3.0);  // S1: Forward 47 Inches with 5 Sec timeout
        sleep(1000);     // pause for servos to move
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = motorL.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorR.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motorL.setTargetPosition(newLeftTarget);
            motorR.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motorL.setPower(Math.abs(speed));
            motorR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorL.isBusy() && motorR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorL.getCurrentPosition(),
                        motorR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorL.setPower(0);
            motorR.setPower(0);

            // Turn off RUN_TO_POSITION
            motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}