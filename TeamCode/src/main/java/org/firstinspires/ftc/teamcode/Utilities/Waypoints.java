package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;

public class Waypoints {

    // Start position specific parameters
    Color.Ftc teamColor;
    RobotHardware.StartPosition startPosition;
    boolean doPartnerMineralField = false;

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
    public Navigation2D flagDrop;
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
    double blueCrater_start_degrees = -45;
    double unhookAngle = 10; // Angle to rotate to unhook.
    double blueCrater_scanXY = 24; // both X and Y for scan position.
    double blueCrater_radiusAngle_degrees = 45; // Direction towards blueCrater corner
    double scanRotation = 45; // How many degrees to rotate to scan each mineral.
    double alignmentOffset = 10; // How many inches to add/subtract to shift and align with side minerals.
    double blueCrater_knockXY_center = 32; // XY position for knocking center mineral
    double knockOffset = 12; // How many inches to add/subtract to knock side minerals.
    double wallOffsetPosition = 58; // Position when traveling along wall from depot to crater
    double flagDropDepth = -60;
    double craterPark_x = 60;
    double craterPark_y = 60;
    // Crater Partner Mineral Scan
    double partner_depotScan_X = -48;
    double partner_depotScan_Y = 48;
    double partner_depotScan_angle_center = 135;
    double partner_depotScan_angle_offset = 45;
    double partner_alignmentOffset;
    double partner_knockOffXY_center;
    double partner_knockOffset;
    /**
     * Blue crater positions set and used as templates
     */
    Navigation2D blueCrater_initialPosition = new Navigation2D(blueCrater_start_x,blueCrater_start_y,degreesToRadians(blueCrater_start_degrees));
    Navigation2D blueCrater_unhookPosition = new Navigation2D(blueCrater_start_x,blueCrater_start_y,degreesToRadians(blueCrater_start_degrees + unhookAngle));
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
    Navigation2D blueCrater_photoPosition = new Navigation2D(0,wallOffsetPosition,degreesToRadians(0));
    Navigation2D blueCrater_flagDrop = new Navigation2D(flagDropDepth,wallOffsetPosition,degreesToRadians(0));
    Navigation2D blueCrater_craterPark = new Navigation2D(craterPark_x,craterPark_y,degreesToRadians(0));

    // Optional team mineral scan
    Navigation2D blueCrater_partner_scanMineral_center = new Navigation2D(partner_depotScan_X,partner_depotScan_Y,degreesToRadians(partner_depotScan_angle_center));
    Navigation2D blueCrater_partner_scanMineral_left = new Navigation2D(partner_depotScan_X,partner_depotScan_Y,degreesToRadians(partner_depotScan_angle_center + partner_depotScan_angle_offset));
    Navigation2D blueCrater_partner_scanMineral_right = new Navigation2D(partner_depotScan_X,partner_depotScan_Y,degreesToRadians(partner_depotScan_angle_center - partner_depotScan_angle_offset));

    Navigation2D blueCrater_partner_alignMineral_center;
    Navigation2D blueCrater_partner_alignMineral_left;
    Navigation2D blueCrater_partner_alignMineral_right;

    Navigation2D blueCrater_partner_knockMineral_center;
    Navigation2D blueCrater_partner_knockMineral_left;
    Navigation2D blueCrater_partner_knockMineral_right;


    public Waypoints(Color.Ftc teamColor, RobotHardware.StartPosition startPosition, boolean doPartnerMineralField) {
        this.teamColor = teamColor;
        this.startPosition = startPosition;
        this.doPartnerMineralField = doPartnerMineralField;
        customizeWaypoints(teamColor, startPosition, doPartnerMineralField);
    }

    void customizeWaypoints(Color.Ftc teamColor, RobotHardware.StartPosition startPosition, boolean doPartnerMineralField) {
        if(teamColor == Color.Ftc.BLUE) {
            if(startPosition == RobotHardware.StartPosition.FIELD_CRATER) {
                //Blue Crater

            } else if (startPosition == RobotHardware.StartPosition.FIELD_DEPOT) {
                //Blue Depot

            } else {
                throw new IllegalStateException("Invalid Starting Position");
            }
        } else if (teamColor == Color.Ftc.RED) {
            if(startPosition == RobotHardware.StartPosition.FIELD_CRATER) {
                //Red Crater

            } else if (startPosition == RobotHardware.StartPosition.FIELD_DEPOT) {
                //Red Depot

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

}