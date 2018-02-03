/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="cwbot: Telop IMU", group="cwbot")
public class cwbotIMUTest extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCwBot   robot           = new HardwareCwBot();              // Use a K9'shardware

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // A B

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello IMU Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.right_bumper)
            {
                robot.RunToEncoder2((int)(-2.0 * robot.ticksPerInch), this);
            }
            if (gamepad1.left_bumper)
            {
                robot.RunToEncoder2((int)(2.0 * robot.ticksPerInch), this);
            }

            float x = gamepad1.right_stick_x;
            float y = -gamepad1.right_stick_y; // Negate to get +y forward.
            float rotation = gamepad1.left_stick_x;

            // A B

            float b = y - rotation;
            float a = y + rotation;

            float biggest = Math.max(Math.abs(a),Math.abs(b));
            if (biggest < 1.0f) biggest = 1.0f;

            robot.rightRearMotor.setPower(b/biggest);
            robot.leftRearMotor.setPower(a/biggest);

            int encoderA = robot.leftRearMotor.getCurrentPosition();
            int encoderB = robot.rightRearMotor.getCurrentPosition();

            Quaternion q = robot.imu.getQuaternionOrientation();
            // The sonar only refreshes at 6.7 Hz.
            // We will average over 1 second to reduce noise.
            double vFront = robot.getFrontDistance();
            double vLeft = robot.getLeftDistance();
            telemetry.addData("Q", "%.5f %.5f %.5f %.5f",q.w,q.x,q.y,q.z);
            telemetry.addData("heading", "%.1f", robot.getHeading());
            telemetry.addData("Encoders","%d %d", encoderA,encoderB);
            telemetry.addData("ds",  "%.2f %.2f", vFront, vLeft);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            //sleep(40);
            robot.waitForTick(40);
        }
    }
}
