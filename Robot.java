package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Robot
{
    /* Public OpMode members. */
    protected DcMotor  leftDrive   = null;
    protected DcMotor  rightDrive  = null;
    protected DcMotor  leftIn      = null;
    protected DcMotor  rightIn     = null;
    protected DcMotor  leftBelt    = null;
    protected DcMotor  rightBelt   = null;
    protected DcMotor  leftLift    = null;
    protected DcMotor  rightLift   = null;
    //protected DistanceSensor distanceSensor1;
    //protected DistanceSensor distanceSensor2;
    protected DigitalChannel touchSensor;
    /* defined variables */
    protected double targetHeight;      //todo find the height of the lift
    private int encoderPerRotation = 560;
    protected int maxLiftPosition = (int) (targetHeight*Math.PI/encoderPerRotation);
    protected int minLiftPosition = 0;

    protected int currentStorage = 0;
    protected final int MAXSTORAGE = 3; // this is in all caps due to final key word conventions

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Robot()
    {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftIn     = hwMap.get(DcMotor.class, "left_in");
        rightIn    = hwMap.get(DcMotor.class, "right_in");
        leftBelt   = hwMap.get(DcMotor.class, "left_belt");
        rightBelt  = hwMap.get(DcMotor.class, "right_belt");
        leftLift   = hwMap.get(DcMotor.class, "left_lift");
        rightLift  = hwMap.get(DcMotor.class, "right_lift");

        //Define and initialize sensors
        //distanceSensor1 = hwMap.get(DistanceSensor.class, "distanceSensor1");
        //distanceSensor2 = hwMap.get(DistanceSensor.class, "distanceSensor2");
        //touchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftIn.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIn.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBelt.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBelt.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBelt.setPower(0);
        rightBelt.setPower(0);
        leftIn.setPower(0);
        rightIn.setPower(0);
        rightLift.setPower(0);
        leftLift.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBelt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBelt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftIn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
 }

