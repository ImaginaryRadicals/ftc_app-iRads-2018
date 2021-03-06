package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

public class Waypoints {

    // Start position specific parameters
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    boolean doPartnerMineralField = false;
    double genericRotate = 0;

    /**
     * blueCrater waypoints, as well as partner_blueDepot waypoints (for partner mineral field) are
     * first defined to be used as a template.  Then a series of rotations are used to define
     * generic waypoints depending on the start position and team color.
     *
     * These initial waypoints for blueCrater and the partner_blueDepot are generated from
     * a list of parameters to ease tweaking.
     *
     * We are allowing the 'left' and 'right' mineral to refer to the robot's perspective,
     * regardless of which side of the mineral field it is on.  This isn't consistent with a
     * global naming convention, but it doesn't matter as long as we knock the gold mineral
     * wherever we find it.
     */

    /**
     * public waypoints customized for starting location
      */
    public Navigation2D initialPosition;
    public Navigation2D unhookPosition;
    public Navigation2D dismountPosition;
    public Navigation2D scanMineral_center;
    public Navigation2D scanMineral_left;
    public Navigation2D scanMineral_right;

    public Navigation2D alignMineral_center;
    public Navigation2D alignMineral_left;
    public Navigation2D alignMineral_right;

    public Navigation2D knockMineral_center;
    public Navigation2D knockMineral_left;
    public Navigation2D knockMineral_right;

    public Navigation2D photoPosition; // team side for crater, front or back for depot
    public Navigation2D photoRotate;
    public Navigation2D flagDrop;
    public Navigation2D depot;
    public Navigation2D craterPark;

    // Optional team mineral scan
    public Navigation2D partner_scanMineral_center;
    public Navigation2D partner_scanMineral_left;
    public Navigation2D partner_scanMineral_right;

    public Navigation2D partner_alignMineral_center;
    public Navigation2D partner_alignMineral_left;
    public Navigation2D partner_alignMineral_right;

    public Navigation2D partner_knockMineral_center;
    public Navigation2D partner_knockMineral_left;
    public Navigation2D partner_knockMineral_right;

    /**
     * Template parameter constants
     */
    double cameraOffset = -90; // extra rotation needed to point the camera in given direction

    double blueCrater_start_x = 12.43;
    double blueCrater_start_y = 12.43;
    double blueCrater_start_degrees = -45 + 9;
    double unhookAngle = 10; // Angle to rotate to unhook.
    double unhookStrafe = 1;
    double blueCrater_scanXY = 24; // both X and Y for initial scan position.
    double blueCrater_radiusAngle_degrees = 45; // Direction towards blueCrater corner
    double scanRotation = 45; // How many degrees to rotate to scan each mineral.
    double alignmentOffset = 10; // How many inches to add/subtract to shift and align with side minerals.
    double blueCrater_knockXY_center = 32; // XY position for knocking center mineral
    double knockOffset = 15; // How many inches to add/subtract to knock side minerals.
    double wallOffsetPosition = 58; // Position when traveling along wall from depot to crater
    double photoRotation = 0;
    double flagDropDepth = 46;
    double depotDepth = 52;
    double craterPark_depth = 15;
    double craterPark_wall_offset = 68;

    // Crater Partner Mineral Scan (blue depot is in second quadrant)
    double partner_blueDepot_Scan_X = -48;
    double partner_blueDepot_Scan_Y = 48;
    double partner_blueDepot_Scan_angle_center = 225; // not 135, not 45
    double partner_blueDepot_Scan_angle_offset = 45;
    double partner_blueDepot_alignmentOffset = 12;
    double partner_blueDepot_knockOffXY_center = 42;
    double partner_blueDepot_knockOffset = 12;
    /**
     * Blue crater positions set and used as templates
     */
    Navigation2D blueCrater_initialPosition = new Navigation2D(blueCrater_start_x,blueCrater_start_y,degreesToRadians(blueCrater_start_degrees));
    Navigation2D blueCrater_unhookPosition = new Navigation2D(blueCrater_start_x + unhookStrafe,blueCrater_start_y - unhookStrafe,degreesToRadians(blueCrater_start_degrees + unhookAngle));
    Navigation2D blueCrater_dismountPosition = new Navigation2D(blueCrater_scanXY,blueCrater_scanXY,degreesToRadians(blueCrater_start_degrees + unhookAngle));
    Navigation2D blueCrater_scanMineral_center = new Navigation2D(blueCrater_scanXY,blueCrater_scanXY,degreesToRadians(blueCrater_start_degrees));
    Navigation2D blueCrater_scanMineral_left = new Navigation2D(blueCrater_scanXY,blueCrater_scanXY,degreesToRadians(blueCrater_start_degrees + scanRotation));
    Navigation2D blueCrater_scanMineral_right = new Navigation2D(blueCrater_scanXY,blueCrater_scanXY,degreesToRadians(blueCrater_start_degrees - scanRotation));

    Navigation2D blueCrater_alignMineral_center = new Navigation2D(blueCrater_scanXY,blueCrater_scanXY,degreesToRadians(blueCrater_start_degrees));
    Navigation2D blueCrater_alignMineral_left = new Navigation2D(blueCrater_scanXY - alignmentOffset,blueCrater_scanXY + alignmentOffset,degreesToRadians(blueCrater_start_degrees));
    Navigation2D blueCrater_alignMineral_right = new Navigation2D(blueCrater_scanXY + alignmentOffset,blueCrater_scanXY - alignmentOffset,degreesToRadians(blueCrater_start_degrees));

    Navigation2D blueCrater_knockMineral_center = new Navigation2D(blueCrater_knockXY_center,blueCrater_knockXY_center,degreesToRadians(blueCrater_start_degrees));
    Navigation2D blueCrater_knockMineral_left = new Navigation2D(blueCrater_knockXY_center - knockOffset,blueCrater_knockXY_center + knockOffset,degreesToRadians(blueCrater_start_degrees));
    Navigation2D blueCrater_knockMineral_right = new Navigation2D(blueCrater_knockXY_center + knockOffset,blueCrater_knockXY_center - knockOffset,degreesToRadians(blueCrater_start_degrees));

    // team side for crater, front or back for depot
    // Angle set to observation of marker by camera on left side.
    Navigation2D blueCrater_photoPosition = new Navigation2D(0,wallOffsetPosition,degreesToRadians(blueCrater_start_degrees));
    Navigation2D blueCrater_photoRotate = new Navigation2D(0, wallOffsetPosition, degreesToRadians(photoRotation));
    Navigation2D blueCrater_flagDrop = new Navigation2D(-flagDropDepth,wallOffsetPosition,degreesToRadians(0));
    Navigation2D blueCrater_depotDepth = new Navigation2D(-depotDepth,wallOffsetPosition,degreesToRadians(0));
    Navigation2D blueCrater_craterPark = new Navigation2D(craterPark_depth, craterPark_wall_offset,degreesToRadians(0));

    // Optional team mineral scan
    Navigation2D partner_blueDepot_scanMineral_center = new Navigation2D(partner_blueDepot_Scan_X, partner_blueDepot_Scan_Y,degreesToRadians(partner_blueDepot_Scan_angle_center));
    Navigation2D partner_blueDepot_scanMineral_left = new Navigation2D(partner_blueDepot_Scan_X, partner_blueDepot_Scan_Y,degreesToRadians(partner_blueDepot_Scan_angle_center + partner_blueDepot_Scan_angle_offset));
    Navigation2D partner_blueDepot_scanMineral_right = new Navigation2D(partner_blueDepot_Scan_X, partner_blueDepot_Scan_Y,degreesToRadians(partner_blueDepot_Scan_angle_center - partner_blueDepot_Scan_angle_offset));

    // partner is at blue depot when we are at blueCrater
    // Same as partner_blueDepot_scanMineral_center
    Navigation2D partner_blueDepot_alignMineral_center = new Navigation2D(partner_blueDepot_Scan_X, partner_blueDepot_Scan_Y, degreesToRadians(partner_blueDepot_Scan_angle_center));
    Navigation2D partner_blueDepot_alignMineral_left = new Navigation2D(partner_blueDepot_Scan_X - partner_blueDepot_alignmentOffset, partner_blueDepot_Scan_Y + partner_blueDepot_alignmentOffset, degreesToRadians(partner_blueDepot_Scan_angle_center));
    Navigation2D partner_blueDepot_alignMineral_right = new Navigation2D(partner_blueDepot_Scan_X + partner_blueDepot_alignmentOffset, partner_blueDepot_Scan_Y - partner_blueDepot_alignmentOffset, degreesToRadians(partner_blueDepot_Scan_angle_center));

    Navigation2D partner_blueDepot_knockMineral_center = new Navigation2D(-partner_blueDepot_knockOffXY_center,partner_blueDepot_knockOffXY_center, degreesToRadians(partner_blueDepot_Scan_angle_center));
    Navigation2D partner_blueDepot_knockMineral_left = new Navigation2D(-partner_blueDepot_knockOffXY_center - partner_blueDepot_knockOffset,partner_blueDepot_knockOffXY_center + partner_blueDepot_knockOffset, degreesToRadians(partner_blueDepot_Scan_angle_center));
    Navigation2D partner_blueDepot_knockMineral_right = new Navigation2D(-partner_blueDepot_knockOffXY_center + partner_blueDepot_knockOffset,partner_blueDepot_knockOffXY_center - partner_blueDepot_knockOffset, degreesToRadians(partner_blueDepot_Scan_angle_center));


    public Waypoints(Color.Ftc teamColor, RobotHardware.StartPosition startPosition, boolean doPartnerMineralField) {
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        this.doPartnerMineralField = doPartnerMineralField;
        customizeWaypoints(teamColor, startPosition, doPartnerMineralField);
    }

    void customizeWaypoints(Color.Ftc teamColor, RobotHardware.StartPosition startPosition, boolean doPartnerMineralField) {
        if(teamColor == Color.Ftc.BLUE) {
            if(startPosition == RobotHardware.StartPosition.FIELD_CRATER) {
                create_blue_crater_waypoints();

            } else if (startPosition == RobotHardware.StartPosition.FIELD_DEPOT) {
                //Blue Depot
                create_blue_depot_waypoints();

            } else {
                throw new IllegalStateException("Invalid Starting Position");
            }
        } else if (teamColor == Color.Ftc.RED) {
            if(startPosition == RobotHardware.StartPosition.FIELD_CRATER) {
                //Red Crater
                create_blue_crater_waypoints();
                rotate_waypoints_in_place(180);

            } else if (startPosition == RobotHardware.StartPosition.FIELD_DEPOT) {
                //Red Depot
                create_blue_depot_waypoints();
                rotate_waypoints_in_place(180);

            } else {
                throw new IllegalStateException("Invalid Starting Position");
            }
        } else {
            throw new IllegalStateException("Invalid Team Color");
        }
    }

    // Utility
    double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }

    void create_blue_crater_waypoints() {
        //Blue Crater
        initialPosition = blueCrater_initialPosition.copy();
        unhookPosition = blueCrater_unhookPosition.copy();
        dismountPosition = blueCrater_dismountPosition.copy();
        scanMineral_center = blueCrater_scanMineral_center.copy();
        scanMineral_left = blueCrater_scanMineral_left.copy();
        scanMineral_right = blueCrater_scanMineral_right.copy();

        alignMineral_center =  blueCrater_alignMineral_center.copy();
        alignMineral_left = blueCrater_alignMineral_left.copy();
        alignMineral_right = blueCrater_alignMineral_right.copy();

        knockMineral_center = blueCrater_knockMineral_center.copy();
        knockMineral_left = blueCrater_knockMineral_left.copy();
        knockMineral_right= blueCrater_knockMineral_right.copy();

        // team side for crater, front or back for depot
        photoPosition = blueCrater_photoPosition.copy();
        photoRotate = blueCrater_photoRotate.copy();
        flagDrop = blueCrater_flagDrop.copy();
        depot = blueCrater_depotDepth.copy();
        craterPark = blueCrater_craterPark.copy();

        // Optional team mineral scan
        partner_scanMineral_center = partner_blueDepot_scanMineral_center.copy();
        partner_scanMineral_left = partner_blueDepot_scanMineral_left.copy();
        partner_scanMineral_right = partner_blueDepot_scanMineral_right.copy();

        partner_alignMineral_center = partner_blueDepot_alignMineral_center.copy();
        partner_alignMineral_left = partner_blueDepot_alignMineral_left.copy();
        partner_alignMineral_right = partner_blueDepot_alignMineral_right.copy();

        partner_knockMineral_center = partner_blueDepot_knockMineral_center.copy();
        partner_knockMineral_left = partner_blueDepot_knockMineral_left.copy();
        partner_knockMineral_right = partner_blueDepot_knockMineral_right.copy();
    }


    void rotate_waypoints_in_place(double rotateDegrees) {
        // rotate the generic waypoints around (0,0), storing
        // back into the generic waypoints.

        initialPosition.rotate(rotateDegrees);
        unhookPosition.rotate(rotateDegrees);
        dismountPosition.rotate(rotateDegrees);
        scanMineral_center.rotate(rotateDegrees);
        scanMineral_left.rotate(rotateDegrees);
        scanMineral_right.rotate(rotateDegrees);
        alignMineral_center.rotate(rotateDegrees);
        alignMineral_left.rotate(rotateDegrees);
        alignMineral_right.rotate(rotateDegrees);
        knockMineral_center.rotate(rotateDegrees);
        knockMineral_left.rotate(rotateDegrees);
        knockMineral_right.rotate(rotateDegrees);
        photoPosition.rotate(rotateDegrees);
        photoRotate.rotate(rotateDegrees);
        depot.rotate(rotateDegrees);
        flagDrop.rotate(rotateDegrees);
        craterPark.rotate(rotateDegrees);
        partner_scanMineral_center.rotate(rotateDegrees);
        partner_scanMineral_left.rotate(rotateDegrees);
        partner_scanMineral_right.rotate(rotateDegrees);
        partner_alignMineral_center.rotate(rotateDegrees);
        partner_alignMineral_left.rotate(rotateDegrees);
        partner_alignMineral_right.rotate(rotateDegrees);
        partner_knockMineral_center.rotate(rotateDegrees);
        partner_knockMineral_left.rotate(rotateDegrees);
        partner_knockMineral_right.rotate(rotateDegrees);
    }


    void create_blue_depot_waypoints() {
        create_blue_crater_waypoints();
        // generally 90 degree rotation from blueCrater points
        genericRotate = 90;
        rotate_waypoints_in_place(genericRotate);


        flagDrop = new Navigation2D(-wallOffsetPosition, flagDropDepth, degreesToRadians(-90));
        craterPark = new Navigation2D(-craterPark_wall_offset, -craterPark_depth, degreesToRadians(-90));


        // Optional team mineral scan
        // For blueDepot, using same points as partner standard scan and knock.
        partner_scanMineral_center = blueCrater_scanMineral_center.copy();
        partner_scanMineral_left = blueCrater_scanMineral_left.copy();
        partner_scanMineral_right = blueCrater_scanMineral_right.copy();

        partner_alignMineral_center = blueCrater_alignMineral_center.copy();
        partner_alignMineral_left = blueCrater_alignMineral_left.copy();
        partner_alignMineral_right = blueCrater_alignMineral_right.copy();

        partner_knockMineral_center = blueCrater_knockMineral_center.copy();
        partner_knockMineral_left = blueCrater_knockMineral_left.copy();
        partner_knockMineral_right = blueCrater_knockMineral_right.copy();
    }
}