package org.firstinspires.ftc.teamcode.References;

public class StorageUtils {
    public enum Position {
        /**
         * The ball on the entrance of the robot, intake.
         */
        INTAKE,
        /**
         * The ball under the shooter.
         */
        SHOOTER,
        /**
         * Ball that is not in the Shooter, but not in the Intake. Backup ball.
         */
        BACKUP,
        /**
         * No Position exactly, a.k.a. UNKNOWN. It is better to use BRAKE.
         */
        BLANK,
        /**
         * Forces the brake of the Storage, without accouting an exact position.
         */
        BRAKE
    }
}
