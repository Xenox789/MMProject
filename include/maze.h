// Maze representation and flood-fill algorithm
#pragma once
#include <Arduino.h>
#include "config.h"
#include "sensing/wall_view.h"   // provides ::WallInfo (compat alias)

// Wall bit flags for each cell
constexpr uint8_t WALL_NORTH = 0x01;
constexpr uint8_t WALL_EAST  = 0x02;
constexpr uint8_t WALL_SOUTH = 0x04;
constexpr uint8_t WALL_WEST  = 0x08;

// Cardinal directions
enum Direction : uint8_t {
  NORTH = 0,
  EAST  = 1,
  SOUTH = 2,
  WEST  = 3
};

// Relative directions 
enum RelativeDir : uint8_t {
  REL_FORWARD = 0,
  REL_RIGHT   = 1,
  REL_BACK    = 2,
  REL_LEFT    = 3
};

namespace Maze {

void init();

// Wall management
void setWall(int x, int y, Direction dir);
bool hasWall(int x, int y, Direction dir);
void updateWalls(int x, int y, Direction heading, const WallInfo& walls);

// Flood-fill
void floodFill(int goalX1, int goalY1, int goalX2, int goalY2);
void floodFillToStart();
uint16_t getFloodValue(int x, int y);

// Navigation
Direction bestDirection(int x, int y, Direction currentHeading);
RelativeDir getRelativeTurn(Direction currentHeading, Direction targetHeading);

// Direction helpers
Direction turnRightDir(Direction d);
Direction turnLeftDir(Direction d);
Direction oppositeDir(Direction d);
void getNextCell(int x, int y, Direction dir, int& nx, int& ny);

// Debug
void printMaze();
void printFlood();

// Check if at goal
bool isGoal(int x, int y);

} // namespace Maze
