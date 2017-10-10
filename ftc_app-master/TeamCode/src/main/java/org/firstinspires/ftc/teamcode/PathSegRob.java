package org.firstinspires.ftc.teamcode;


/**
 * Define a "PathSegment" object, used for building a path for the robot to follow.
 */
public class PathSegRob {
    public DriveType Type;
    public double Speed;
    public double Turn;
    public double Strafe;
    public double Time;
    public org.firstinspires.ftc.teamcode.Destination Destination;
    public int Orientation;
    public int Heading;
    public int leftFrontEncoder;
    public int leftRearEncoder;
    public int rightFrontEncoder;
    public int rightRearEncoder;

    enum DriveMode {TANK, POV, ARCADE, HOLONOMIC}

    ;

    enum DriveType {TIME, GYRO, ENCODER, DES_ORI}

    // Constructor
    //TIME
    public PathSegRob(DriveType Type, double Speed, double Turn, double Strafe,
                      double Time) {
        this.Type = Type;
        this.Speed = Speed;
        this.Turn = Turn;
        this.Strafe = Strafe;
        this.Time = Time;
    }

    //GYRO
    public PathSegRob(DriveType Type, double Speed, double Turn, double Strafe,
                      int Heading) {

        this.Type = Type;
        this.Speed = Speed;
        this.Turn = Turn;
        this.Strafe = Strafe;
        this.Heading = Heading;
    }

    //ENCODER
    public PathSegRob(DriveType Type, double Speed, double Turn, double Strafe,
                      int leftFrontEncoder, int leftRearEncoder,
                      int rightFrontEncoder, int rightRearEncoder) {

        this.Type = Type;
        this.Speed = Speed;
        this.Turn = Turn;
        this.Strafe = Strafe;
        this.leftFrontEncoder = leftFrontEncoder;
        this.leftRearEncoder = leftRearEncoder;
        this.rightFrontEncoder = rightFrontEncoder;
        this.rightRearEncoder = rightRearEncoder;

    }

    //DES_ORI
    public PathSegRob(DriveType Type, double Speed, org.firstinspires.ftc.teamcode.Destination Destination,
                      double Turn, int Orientation) {
        this.Type = Type;
        this.Speed = Speed;
        this.Destination = Destination;
        this.Turn = Turn;
        this.Orientation = Orientation;
    }

}

class Destination {
    public double X;
    public double Y;

    public Destination(double X, double Y) {
        this.X = X;
        this.Y = Y;
    }
}
