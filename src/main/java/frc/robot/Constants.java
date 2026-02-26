//Package
package frc.robot;

//Create class
public class Constants {
    
    //ID´s constnts
    public class IDs {

        //Shooter constants
        public class Shooter {
            public static final int bottomShootingMotorId = 0;
            public static final int topShootingMotorId = 0;
            public static final int leftWristMotorId = 0;
            public static final int rightWristMotorId = 0;
        }

        //Intake constants
        public class Intake {
            public static final int intakeMotorId = 0;
            public static final int leftWristMotorId = 0;
            public static final int rightWristMotorId = 0;
        }
    }

    //Limelights constants
    public class Limelights {

        public static final String shooterLimelightName = "limelight";
        public static final String extraLimelightName = "limelight2";

        //ApriltagLimits constants
        public class AprilTagLimits {
            public static final double XError = 0.0;
            public static final double ZError = 2;
            public static final double yawError = 0.0;
            public static final double ZDeadband = 0.05; //cm
        }

        //PID constants
        public class PID {

            //Rotation constnts
            public class Rotation {
                public static final double kP = 0.1;
                public static final double kI = 0.05;
                public static final double kD = 0.0;
            }

            //X constants
            public class X {
                public static final double kP = 4.75;
                public static final double kI = 0.1;
                public static final double kD = 0.1;
            }

            //Z constants
            public class Z {
                public static final double kP = 2.5;
                public static final double kI = 0.0;
                public static final double kD = 0.1;
            }
        }
    }

    //Mechanisms constants
    public class Mechanisms {
        public static double shooterMountingAngle = 50.0; //degrees
        public static final double shooterMountingHeight = 0; //meters
        public static final double shooterWheelRadius = 0; //meters
    }
}
