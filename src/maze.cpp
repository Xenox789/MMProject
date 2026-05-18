#include "maze.h"

namespace Maze {

static const int N = CONFIG::MAZE_SIZE;

// Wall data: each cell stores its known walls as bit flags
static uint8_t walls[N][N];

// Flood values: distance to goal for each cell
static uint16_t flood[N][N];

// Infinity value for flood fill
static const uint16_t INF = 0xFFFF;

void init() {
  // Clear all internal walls
  for (int x = 0; x < N; x++) {
    for (int y = 0; y < N; y++) {
      walls[x][y] = 0;
      flood[x][y] = INF;
    }
  }
  
  // Set boundary walls (the outer walls of the maze are always present)
  for (int i = 0; i < N; i++) {
    walls[i][0]     |= WALL_SOUTH;  // Bottom row
    walls[i][N-1]   |= WALL_NORTH;  // Top row
    walls[0][i]     |= WALL_WEST;   // Left column
    walls[N-1][i]   |= WALL_EAST;   // Right column
  }
  
  // Start cell (0,0) always has a wall to the east in standard mazes
  // (This ensures the mouse starts facing north)
}

// --- Wall Management ---

void setWall(int x, int y, Direction dir) {
  if (x < 0 || x >= N || y < 0 || y >= N) return;
  
  switch (dir) {
    case NORTH:
      walls[x][y] |= WALL_NORTH;
      if (y + 1 < N) walls[x][y+1] |= WALL_SOUTH;  // Neighbor's wall too
      break;
    case EAST:
      walls[x][y] |= WALL_EAST;
      if (x + 1 < N) walls[x+1][y] |= WALL_WEST;
      break;
    case SOUTH:
      walls[x][y] |= WALL_SOUTH;
      if (y - 1 >= 0) walls[x][y-1] |= WALL_NORTH;
      break;
    case WEST:
      walls[x][y] |= WALL_WEST;
      if (x - 1 >= 0) walls[x-1][y] |= WALL_EAST;
      break;
  }
}

bool hasWall(int x, int y, Direction dir) {
  if (x < 0 || x >= N || y < 0 || y >= N) return true;  // Out of bounds = wall
  
  switch (dir) {
    case NORTH: return walls[x][y] & WALL_NORTH;
    case EAST:  return walls[x][y] & WALL_EAST;
    case SOUTH: return walls[x][y] & WALL_SOUTH;
    case WEST:  return walls[x][y] & WALL_WEST;
  }
  return true;
}

// Update maze with walls detected by sensors at current position + heading
void updateWalls(int x, int y, Direction heading, const WallInfo& wi) {
  // The "front" sensor detects a wall in the direction we're heading
  if (wi.front) setWall(x, y, heading);
  
  // "left" sensor detects a wall to the left of our heading
  if (wi.left) setWall(x, y, turnLeftDir(heading));
  
  // "right" sensor detects a wall to the right of our heading
  if (wi.right) setWall(x, y, turnRightDir(heading));
}

// --- Flood Fill Algorithm ---
// Uses BFS to calculate shortest path from every cell to the goal

void floodFill(int goalX1, int goalY1, int goalX2, int goalY2) {
  // Reset all values to infinity
  for (int x = 0; x < N; x++)
    for (int y = 0; y < N; y++)
      flood[x][y] = INF;
  
  // BFS queue (simple ring buffer)
  static int queueX[N * N];
  static int queueY[N * N];
  int head = 0, tail = 0;
  
  // Seed the goal cells with distance 0
  for (int gx = goalX1; gx <= goalX2; gx++) {
    for (int gy = goalY1; gy <= goalY2; gy++) {
      flood[gx][gy] = 0;
      queueX[tail] = gx;
      queueY[tail] = gy;
      tail++;
    }
  }
  
  // BFS expansion
  while (head != tail) {
    int cx = queueX[head];
    int cy = queueY[head];
    head++;
    
    uint16_t newDist = flood[cx][cy] + 1;
    
    // Try all 4 directions
    for (int d = 0; d < 4; d++) {
      Direction dir = (Direction)d;
      
      // Skip if there's a wall blocking this direction
      if (hasWall(cx, cy, dir)) continue;
      
      int nx, ny;
      getNextCell(cx, cy, dir, nx, ny);
      
      // Skip if out of bounds
      if (nx < 0 || nx >= N || ny < 0 || ny >= N) continue;
      
      // Update if we found a shorter path
      if (newDist < flood[nx][ny]) {
        flood[nx][ny] = newDist;
        queueX[tail] = nx;
        queueY[tail] = ny;
        tail++;
      }
    }
  }
}

void floodFillToStart() {
  floodFill(0, 0, 0, 0);
}

uint16_t getFloodValue(int x, int y) {
  if (x < 0 || x >= N || y < 0 || y >= N) return INF;
  return flood[x][y];
}

// --- Navigation ---

// Find the best direction to move from (x,y) based on flood values
// Prefers the direction that doesn't require turning
Direction bestDirection(int x, int y, Direction currentHeading) {
  uint16_t bestVal = INF;
  Direction bestDir = currentHeading;
  
  // Check all 4 directions, prioritize forward to minimize turns
  Direction checkOrder[4];
  checkOrder[0] = currentHeading;                  // Forward first
  checkOrder[1] = turnRightDir(currentHeading);    // Then right
  checkOrder[2] = turnLeftDir(currentHeading);     // Then left
  checkOrder[3] = oppositeDir(currentHeading);     // U-turn last
  
  for (int i = 0; i < 4; i++) {
    Direction dir = checkOrder[i];
    
    if (hasWall(x, y, dir)) continue;
    
    int nx, ny;
    getNextCell(x, y, dir, nx, ny);
    
    if (nx < 0 || nx >= N || ny < 0 || ny >= N) continue;
    
    uint16_t val = flood[nx][ny];
    if (val < bestVal) {
      bestVal = val;
      bestDir = dir;
    }
  }
  
  return bestDir;
}

RelativeDir getRelativeTurn(Direction currentHeading, Direction targetHeading) {
  int diff = ((int)targetHeading - (int)currentHeading + 4) % 4;
  return (RelativeDir)diff;
}

// --- Direction Helpers ---

Direction turnRightDir(Direction d) {
  return (Direction)(((int)d + 1) % 4);
}

Direction turnLeftDir(Direction d) {
  return (Direction)(((int)d + 3) % 4);
}

Direction oppositeDir(Direction d) {
  return (Direction)(((int)d + 2) % 4);
}

void getNextCell(int x, int y, Direction dir, int& nx, int& ny) {
  nx = x;
  ny = y;
  switch (dir) {
    case NORTH: ny = y + 1; break;
    case EAST:  nx = x + 1; break;
    case SOUTH: ny = y - 1; break;
    case WEST:  nx = x - 1; break;
  }
}

bool isGoal(int x, int y) {
  return x >= CONFIG::GOAL_X1 && x <= CONFIG::GOAL_X2 &&
         y >= CONFIG::GOAL_Y1 && y <= CONFIG::GOAL_Y2;
}

// --- Debug ---

void printMaze() {
  Serial.println("=== MAZE WALLS ===");
  for (int y = N - 1; y >= 0; y--) {
    // Print north walls
    for (int x = 0; x < N; x++) {
      Serial.print("+");
      Serial.print(hasWall(x, y, NORTH) ? "---" : "   ");
    }
    Serial.println("+");
    
    // Print west walls and cell
    for (int x = 0; x < N; x++) {
      Serial.print(hasWall(x, y, WEST) ? "|" : " ");
      if (flood[x][y] == INF) {
        Serial.print(" X ");
      } else {
        char buf[4];
        snprintf(buf, 4, "%3d", flood[x][y]);
        Serial.print(buf);
      }
    }
    Serial.println("|");
  }
  // Bottom border
  for (int x = 0; x < N; x++) {
    Serial.print("+---");
  }
  Serial.println("+");
}

void printFlood() {
  Serial.println("=== FLOOD VALUES ===");
  for (int y = N - 1; y >= 0; y--) {
    for (int x = 0; x < N; x++) {
      if (flood[x][y] == INF) {
        Serial.print("  X");
      } else {
        char buf[4];
        snprintf(buf, 4, "%3d", flood[x][y]);
        Serial.print(buf);
      }
      Serial.print(" ");
    }
    Serial.println();
  }
}

} // namespace Maze
