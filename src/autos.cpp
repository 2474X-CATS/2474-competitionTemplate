#include "autos.h"

/*
-------------------------------------------------------------------------------------------------------------------------------------------------
     _              _______   _____    _____
    / \    |     |     |     |     |   |
   /___\   |     |     |     |     |   -----
  /     \  |_____|     |     |_____|   ____|
 --------------------------------------------------------------------------------------------------------------------------------------------------
*/

vector<CommandInterface *> closed_side_left()
{
  return { 

      DriveToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.9, PathType::EUCLIDEAN, true),
      TurnToLocation(Zones::NAT_MID),
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM, 0, false),
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 1000), 
      
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1, TILE_SIZE_MM * 1}, true}},
          false), 
      DrivePath::getCommand({270}, true, false),
      DriveForwardForTime::getCommand(0.125, 500, true),
      IntakeCubes::getCommand(750),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      DrivePath::getCommand({TILE_SIZE_MM * 1.25}, false, true),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000)
  };
}

vector<CommandInterface *> closed_side_right()
{
  return {
      DriveToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.9, PathType::EUCLIDEAN, true),
      TurnToLocation(Zones::NAT_LOW),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM, 0, false),
      // Score
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5.1, TILE_SIZE_MM * 1}, true}},
          false),
      DrivePath::getCommand({270}, true, false),
      DriveForwardForTime::getCommand(0.125, 500, true),
      IntakeCubes::getCommand(750),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      DrivePath::getCommand({TILE_SIZE_MM * 1.25}, false, true),
      // Score
  };
}

vector<CommandInterface *> switcheroo_left()
{
  return {

  };
}

vector<CommandInterface *> renegade_right()
{
  return {
      DriveToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.9, PathType::EUCLIDEAN, true),
      DrivePath::getCommand({135}, true, false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5.1, TILE_SIZE_MM * 1}, true}},
          false),
      DrivePath::getCommand({270}, true, false),
      DriveForwardForTime::getCommand(0.125, 500, true),
      IntakeCubes::getCommand(750),
      ModifyRobotState::getCommand("is_drive_inverted", true),
      DrivePath::getCommand({TILE_SIZE_MM * 1.25}, false, true),
      DriveForwardForTime::getCommand(-0.25, 500, false),
      DrivePath::getCommand({30, TILE_SIZE_MM / 2, 90}, true, false),
      DriveForwardForTime::getCommand(0.35, 750, false)};
}

vector<CommandInterface *> renegade_left()
{
  return {
      DriveToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.9, PathType::EUCLIDEAN, true),
      DrivePath::getCommand({45}, true, false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1, TILE_SIZE_MM * 1}, true}},
          false),
      DrivePath::getCommand({270}, true, false),
      DriveForwardForTime::getCommand(0.125, 750, true),
      IntakeCubes::getCommand(750),
      ModifyRobotState::getCommand("is_drive_inverted", true),
      DrivePath::getCommand({TILE_SIZE_MM * 1.15}, false, true),
      DriveForwardForTime::getCommand(-0.25, 500, false),
      DrivePath::getCommand({30, TILE_SIZE_MM / 2 + 25, 90}, true, false),
      DriveForwardForTime::getCommand(0.35, 750, false)};
}

vector<CommandInterface *> control_rush_left()
{
  return {
      DriveToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.9, PathType::EUCLIDEAN, true),
      DrivePath::getCommand({45}, true, false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1, TILE_SIZE_MM * 1.75}, true}},
          false),
      ModifyRobotState::getCommand("is_drive_inverted", true),
      DrivePath::getCommand({90, TILE_SIZE_MM * 0.4}, true, false),
      DriveForwardForTime::getCommand(-0.25, 500, false),
      DrivePath::getCommand({30, TILE_SIZE_MM / 2 + 25, 90}, true, false),
      DriveForwardForTime::getCommand(0.35, 750, false)};
}

vector<CommandInterface *> control_rush_right()
{
  return {
      DriveToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.9, PathType::EUCLIDEAN, true), 

      DrivePath::getCommand({-TILE_SIZE_MM * 0.9, 225}, true, false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5, TILE_SIZE_MM * 1.75}, true}},
          false), 
      ModifyRobotState::getCommand("is_drive_inverted", true),
      DrivePath::getCommand({90, TILE_SIZE_MM * 0.4}, true, false),
      DriveForwardForTime::getCommand(-0.25, 500, false),
      DrivePath::getCommand({30, TILE_SIZE_MM / 2 + 25, 90}, true, false),
      DriveForwardForTime::getCommand(0.35, 750, false)};
}

vector<CommandInterface *> center_cleanup_left()
{
  return {
      DriveToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.9, PathType::EUCLIDEAN, true), 
      ModifyRobotState::getCommand("is_drive_inverted", true),
      TurnToLocation(Zones::NAT_MID),
      DrivePath::getCommand({TILE_SIZE_MM * 0.9}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 1000), 
      
      ModifyRobotState::getCommand("is_drive_inverted", false),
      DrivePath::getCommand({TILE_SIZE_MM * 0.9, 315}, false, false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 4, TILE_SIZE_MM * 2}, true}},
          false), 

      DriveForwardForTime::getCommand(0.1, 350, true), 
      IntakeCubes::getCommand(300),  

      TurnToLocation(Zones::NAT_LOW),
      DrivePath::getCommand({TILE_SIZE_MM * 0.9}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1000)

  };
}

vector<CommandInterface *> center_cleanup_right()
{
  return {
      DriveToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.9, PathType::EUCLIDEAN, true),
      TurnToLocation(Zones::NAT_LOW),
      DrivePath::getCommand({TILE_SIZE_MM * 0.9}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1000), 
      DrivePath::getCommand({-TILE_SIZE_MM * 0.9}, false, false),
      DrivePath::getCommand({225}, true, false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 2, TILE_SIZE_MM * 2}, true}},
          false), 

      DriveForwardForTime::getCommand(0.1, 350, true), 
      IntakeCubes::getCommand(300),  

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      TurnToLocation(Zones::NAT_MID),
      DrivePath::getCommand({TILE_SIZE_MM * 0.9}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 1000)
  };
}

vector<CommandInterface *> auto_skills_f__k_it_we_ball()
{ // 119 pts
  return {

      //-------------------------------------------------------------------------------
      // Mid goal control fill
      // PRETTY PLEASE WITH CHERRIES ON TOP WOR ON FIRST TRY

      ParkClear::getCommand(
          {(PCPhase){0.25, 500, true},
           (PCPhase){-0.25, 500, false},
           (PCPhase){0.05, 250, false}},
          true),

      ModifyRobotState::getCommand("is_drive_inverted", true),

      DriveForwardForTime::getCommand(0.1, 250, false),
      DriveToLocation(Zones::NAT_MID, TILE_SIZE_MM, PathType::EUCLIDEAN, false),
      TurnToLocation(Zones::NAT_MID),
      DrivePath::getCommand({TILE_SIZE_MM * 0.8}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 1000),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DrivePath::getCommand({TILE_SIZE_MM * 0.6}, false, false),

      //-------------------------------------------------------------------------------

      DriveToLocation(Zones::NAT_ML_RIGHT, TILE_SIZE_MM * 1.5, PathType::EUCLIDEAN, false),
      DrivePath::getCommand({270}, true, false),
      DeployMatchloader::getCommand(true),
      DriveForwardForTime::getCommand(0.125, 500, true),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 4.5, TILE_SIZE_MM * 2}, true}},
          false),

      FlatAlignWithY::getCommand(TILE_SIZE_MM * 4),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5, TILE_SIZE_MM * 4.75}, true}},
          false),

      DrivePath::getCommand({270}, true, false),
      DriveForwardForTime::getCommand(0.125, 500, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1200),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DriveForwardForTime::getCommand(0.25, 1000, true),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      DriveForwardForTime::getCommand(0.25, 1000, false),
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, 0, 100),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1200),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 4, TILE_SIZE_MM * 5.75}, true}},
          false),

      //------------------------------------------------------------------
      // Low Goal control fill
      // IM LITERALLY ON MY KNEES BEGGING THAT THIS WORKS

      ParkClear::getCommand(
          {(PCPhase){0.25, 750, true},
           (PCPhase){-0.05, 250, false}},
          false),

      DriveToLocation(Zones::FAR_LOW, TILE_SIZE_MM * 2, PathType::EUCLIDEAN, false),
      TurnToLocation(Zones::FAR_LOW),
      DrivePath::geCommand({TILE_SIZE_MM * 0.75}, false, true),
      IntakeCubes::getCommand(100),
      DrivePath::getCommand({TILE_SIZE_MM}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1000),

      //-------------------------------------------------------------------

      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1, TILE_SIZE_MM * 5}, true}},
          false),

      DrivePath::getCommand({90}, true, false),
      DriveForwardForTime::getCommand(0.25, 1000, true),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1.5, TILE_SIZE_MM * 4}, true}},
          false),
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 2),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1, TILE_SIZE_MM * 1.5}, true}},
          false),
      DrivePath::getCommand({90}, true, false),
      DriveForwardForTime::getCommand(0.125, 500, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DriveForwardForTime::getCommand(0.25, 1000, false),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      DriveForwardForTime::getCommand(0.25, 1000, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0, 100),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 2, TILE_SIZE_MM * 0.75}, true}},
          false),
      DriveForwardForTime::getCommand(0.2, 1000, true)};
}

vector<CommandInterface *> auto_skills_safety_net()
{
  return {
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 5),
      DrivePath::getCommand({270}, true, false),
      DriveForwardForTime::getCommand(0.1, 500, true),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 4.5, TILE_SIZE_MM * 2}, true}},
          false),

      FlatAlignWithY::getCommand(TILE_SIZE_MM * 4),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5, TILE_SIZE_MM * 4.75}, true}},
          false),

      DrivePath::getCommand({270}, true, false),
      DriveForwardForTime::getCommand(0.125, 500, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1200),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DriveForwardForTime::getCommand(0.25, 1000, true),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      DriveForwardForTime::getCommand(0.25, 1000, false),
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, 0, 100),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1200),

      DriveForwardForTime::getCommand(-0.1, 300, false),
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 1),
      DrivePath::getCommand({90}, true, false),

      DriveForwardForTime::getCommand(0.25, 1000, true),
      IntakeCubes::getCommand(1000),
      
      ModifyRobotState::getCommand("is_drive_inverted", true), 
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1.5, TILE_SIZE_MM * 4}, true}},
          false),
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 2),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1, TILE_SIZE_MM * 1.5}, true}},
          false),
      DrivePath::getCommand({90}, true, false),
      DriveForwardForTime::getCommand(0.125, 500, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DriveForwardForTime::getCommand(0.25, 1000, false),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      DriveForwardForTime::getCommand(0.25, 1000, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0, 100),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 2, TILE_SIZE_MM * 0.75}, true}},
          false),
      DriveForwardForTime::getCommand(0.2, 1000, true)
  };
}

vector<CommandInterface *> auto_skills()
{
  return auto_skills_f__k_it_we_ball();
}

vector<CommandInterface *> empty()
{
  return {};
}

vector<CommandInterface *> do_nothing()
{
  return {
      IntakeCubes::getCommand(500)};
}

vector<CommandInterface *> drive_forward()
{
  return {
      DriveForwardForTime::getCommand(0.2, 500, false)};
}
