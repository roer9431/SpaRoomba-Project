#include <Sparki.h>

//odometry
float pos[3], w_speed, w_l, w_r, w_pivot;
float dist_d, theta_d, facing;
//constants
float pi;
//mapping
int grid_size, env_map[4][4], goal[2], map_pos[2];
//control flow
int loop_time, display_counter;
bool moving, scouting, mapping, thinking, planning, done;
//sensory
int gaze, object, object_placement[3][4]; //object_placement[i][j]: object i is at j=0 distance, centered at j=1, starts at j=2, and ends at j=3
bool object_sighted;
//pathing
float waypoint[3];
int index_map[16], map_adj[16][4], path[10];
int master_plan[16];

void setup() {
  true_setup();
  make_map();
}

void loop() {
  int t_start = millis();
  if (moving) {move_arc();}
  //if (scouting) {scout();}
  if (thinking) {think();}
  if (planning) {plan();}
  display();
  get_pos();
  check_status();
  int t_end = millis();
  loop_time = (t_end - t_start);
}

void check_status() {
  get_relative_pos();
  if (abs(dist_d) > .5) {moving = true;}
  if (moving == true && abs(dist_d) < grid_size/4) {planning = true;}
  if (map_pos[0] == goal[0] && map_pos[1] == goal[1]) {done = true;}
  if (done) {
    moving = false;
    sparki.moveStop();
    sparki.beep(random(1, 4)*100);
  }
}

void display(){
  int timer = 45;
  sparki.clearLCD();
  if (display_counter < timer) {
    sparki.println("X, Y Waypoint");
    sparki.print(waypoint[0]);
    sparki.print(", ");
    sparki.println(waypoint[1]);
    sparki.println("X, Y, Theta Pos");
    sparki.print(pos[0]);
    sparki.print(",");
    sparki.print(pos[1]);
    sparki.print(",");
    sparki.println(pos[2]);
    sparki.print("");
  }
  else if (display_counter < timer*2) {
    int stow = env_map[map_pos[0]][map_pos[1]];
    env_map[map_pos[0]][map_pos[1]] = 2;
    sparki.print(env_map[0][3]); sparki.print(" ");
    sparki.print(env_map[1][3]); sparki.print(" ");
    sparki.print(env_map[2][3]); sparki.print(" ");
    sparki.print(env_map[3][3]);
    if (moving) {sparki.println("     Moving");} else {sparki.println("");}
    sparki.print(env_map[0][2]); sparki.print(" ");
    sparki.print(env_map[1][2]); sparki.print(" ");
    sparki.print(env_map[2][2]); sparki.print(" ");
    sparki.print(env_map[3][2]);
    if (planning) {sparki.println("     Planning");} else {sparki.println("");}
    sparki.print(env_map[0][1]); sparki.print(" ");
    sparki.print(env_map[1][1]); sparki.print(" ");
    sparki.print(env_map[2][1]); sparki.print(" ");
    sparki.println(env_map[3][1]);
    sparki.print(env_map[0][0]); sparki.print(" ");
    sparki.print(env_map[1][0]); sparki.print(" ");
    sparki.print(env_map[2][0]); sparki.print(" ");
    sparki.println(env_map[3][0]);
    sparki.println();
    sparki.println("Map Pos X, Y");
    sparki.print(map_pos[0]);
    sparki.print("; ");
    sparki.println(map_pos[1]);
    env_map[map_pos[0]][map_pos[1]] = stow;
    if (path[9] != -1) {sparki.print(path[9]); sparki.print("-");}
    if (path[8] != -1) {sparki.print(path[8]); sparki.print("-");}
    if (path[7] != -1) {sparki.print(path[7]); sparki.print("-");}
    if (path[6] != -1) {sparki.print(path[6]); sparki.print("-");}
    if (path[5] != -1) {sparki.print(path[5]); sparki.print("-");}
    if (path[4] != -1) {sparki.print(path[4]); sparki.print("-");}
    if (path[3] != -1) {sparki.print(path[3]); sparki.print("-");}
    if (path[2] != -1) {sparki.print(path[2]); sparki.print("-");}
    if (path[1] != -1) {sparki.print(path[1]); sparki.print("-");}
    if (path[0] != -1) {sparki.print(path[0]);}
  }
  sparki.updateLCD();
  display_counter += 1;
  if (display_counter == 2*timer) {display_counter = 0;}
}

void get_pos(){
  float avg_speed = w_speed * (w_l + w_r) / 200;
  pos[0] += cos(pos[2]) * avg_speed * loop_time;
  pos[1] += sin(pos[2]) * avg_speed * loop_time;
  
  pos[2] += (w_r - w_l) * w_speed * loop_time / (200 * w_pivot);
  if (pos[2] >= pi) {pos[2] -= 2*pi;}
  if (pos[2] < -1*pi) {pos[2] += 2*pi;}

  facing = pos[2] - pi*gaze/180;

  map_pos[0] = (int)(pos[0]/grid_size);
  map_pos[1] = (int)(pos[1]/grid_size);
}

void get_relative_pos() {
  float x_d = waypoint[0] - pos[0];
  float y_d = waypoint[1] - pos[1];
  dist_d = sqrt(pow(x_d, 2) + pow(y_d, 2));
  
  theta_d = atan2(y_d, x_d) - pos[2];
  if (theta_d >= pi) {theta_d -= 2*pi;}
  if (theta_d < pi*-1) {theta_d += 2*pi;}
}

void make_map() {
  for (int i = 0; i < 16; i++){
    if (i%4 != 3) {map_adj[i][0] = i + 1;}
    else {map_adj[i][0] = -1;}
    if (i + 4 < 16) {map_adj[i][1] = i + 4;}
    else {map_adj[i][1] = -1;}
    if (i%4 != 0) {map_adj[i][2] = i - 1;}
    else {map_adj[i][2] = -1;}
    if (i - 4 >= 0) {map_adj[i][3] = i - 4;}
    else {map_adj[i][3] = -1;}
  }
}

void move_arc(){
  int w_r_dir = DIR_CW;
  int w_l_dir = DIR_CCW;
  w_r = dist_d + 3*w_pivot*theta_d;
  w_l = dist_d - 3*w_pivot*theta_d;
  float mod;
  if (w_r > w_l) {
    mod = 100/w_r;
    w_r = 100;
    w_l = w_l*mod;
  }
  else {
    mod = 100/w_l;
    w_l = 100;
    w_r = w_r*mod;
  }
  if (dist_d < .5) {w_r = 0; w_l = 0;}
  if (w_r < 0) {w_r_dir = DIR_CCW;}   //check for negative wheel speed inputs
  if (w_l < 0) {w_l_dir = DIR_CW;}
  sparki.motorRotate(MOTOR_LEFT, w_l_dir, abs(w_l));
  sparki.motorRotate(MOTOR_RIGHT, w_r_dir, abs(w_r));
}

void move_direct() {
  int w_r_dir = DIR_CW;
  int w_l_dir = DIR_CCW;
  w_l = 100;
  w_r = 100;
  if (abs(theta_d) >= pi/128 && abs(dist_d) > .5) {
    if (theta_d < 0) {w_r_dir = DIR_CCW; w_r = -100;}   //choose direction of rotation
    else {w_l_dir = DIR_CW; w_l = -100;}
    sparki.motorRotate(MOTOR_LEFT, w_l_dir, 100);
    sparki.motorRotate(MOTOR_RIGHT, w_r_dir, 100);
  }
  else if (abs(dist_d) > .5) {
      sparki.motorRotate(MOTOR_LEFT, w_l_dir, 100);
      sparki.motorRotate(MOTOR_RIGHT, w_r_dir, 100);
  }
  if (abs(dist_d) < .5) {
    sparki.moveStop();
    moving = false;
    w_l = 0;
    w_r = 0;
  }
}

void plan() {
  int seeker = 9;
  int next_waypoint[2];
  while (seeker > 0 && path[seeker] != (map_pos[0] + 4*map_pos[1])) {seeker--;}
  path[seeker] = -1;
  seeker--;
  if (seeker >= 0) {
    next_waypoint[0] = path[seeker] % 4;
    next_waypoint[1] = path[seeker] / 4;
    waypoint[0] = grid_size*next_waypoint[0] + grid_size/2;
    waypoint[1] = grid_size*next_waypoint[1] + grid_size/2;
  }
  planning = false;
}

void scout() {
  sparki.servo(gaze++);
  if(gaze>30){ 
    if (object_sighted){
      object_sighted = false;
    }
    gaze=-30;
    Serial.println(); 
  }
  int cm = sparki.ping();
  Serial.print(cm); 
  Serial.print(" "); 
  if (!object_sighted && 1 < cm && cm <= 50) {
    object += 1;
    if (object > 2) {object = 0;}
    object_placement[object][0] = cm;
    object_placement[object][2] = (int)(facing*180/pi);
  }
  else if (object_sighted && 1 < cm && cm <= 50){
    object_placement[object][3] = (int)(facing*180/pi);
  }
  else if (object_sighted && 50 < cm) {
    object_sighted = false;
    object_placement[object][1] = (object_placement[object][2] + object_placement[object][3]) / 2;
  }
}

void think() {
  int solved[2][16];  //0 = distance, 1 = parent node
  int unsolved[2][16];
  int current;
  int goal_index = goal[0] + 4*goal[1];
  int current_index = map_pos[0] + 4*map_pos[1];

  for (int i = 0; i < 16; i++) {
    solved[0][i] = 99;
    solved[1][i] = -1;
    unsolved[0][i] = 99;
    unsolved[1][i] = -1;
  }
  for (int i = 0; i < 10; i++) {
    path[i] = -1;
  }
  current = current_index;
  unsolved[0][current] = 0;
  unsolved[1][current] = -2;
  solved[0][current] = 0;
  solved[1][current] = -1;
  while (current != goal_index) {
    for (int i = 0; i < 4; i++) {
      int probe = map_adj[current][i]; //probe is index of adjacent node
      if (probe != -1 && unsolved[1][probe] != -2) { //-2 means explored
        int new_distance = solved[0][current] + get_distance(current, probe);
        if (unsolved[0][probe] > new_distance) {
          unsolved[0][probe] = new_distance;
          unsolved[1][probe] = current;
        }
      }
    }

    int ephemeral;
    int searcher;
    int minimum = 100;
    for (searcher = 0; searcher < 16; searcher++) {
      if (unsolved[0][searcher] < minimum && unsolved[1][searcher] != -2) {
        minimum = unsolved[0][searcher];
        ephemeral = searcher;
      }
    }
    
    solved[0][ephemeral] = unsolved[0][ephemeral];
    solved[1][ephemeral] = unsolved[1][ephemeral];
    unsolved[1][ephemeral] = -2;  //-2 to signify is exlored
    current = ephemeral;
  }
  path[0] = goal_index;
  int tracer = goal_index;
  int aligner = 1;
  while (tracer != current_index) {
    path[aligner] = solved[1][tracer];
    tracer = path[aligner];
    aligner++;
  }
  thinking = false;
}

void true_setup() {
  sparki.servo(0);
  delay(500);
  loop_time = 200;
  pi = 3.14159;
  w_speed = .0028;
  w_pivot = 4.2;
  w_l = 0;
  w_r = 0;

  gaze = 0;
  object_sighted = false;

  grid_size = 14;   //distance across one grid square in cm
  env_map[0][0] = 0;
  env_map[0][1] = 1;
  env_map[0][2] = 0;
  env_map[0][3] = 1;
  env_map[1][0] = 1;
  env_map[1][1] = 1;
  env_map[1][2] = 0;
  env_map[1][3] = 1;
  env_map[2][0] = 0;
  env_map[2][1] = 1;
  env_map[2][2] = 1;
  env_map[2][3] = 1;
  env_map[3][0] = 1;
  env_map[3][1] = 1;
  env_map[3][2] = 0;
  env_map[3][3] = 0;

  pos[0] = (grid_size/2);
  pos[1] = (grid_size/2) + (grid_size*3);
  pos[2] = 0;
  map_pos[0] = 0;
  map_pos[1] = 3;
  waypoint[0] = pos[0];
  waypoint[1] = pos[1];
  waypoint[2] = pos[2];
  goal[0] = 1;
  goal[1] = 0;

  path[0] = -1;
  path[1] = -1;
  path[2] = -1;
  path[3] = -1;
  path[4] = -1;
  path[5] = -1;
  path[6] = -1;
  path[7] = -1;
  path[8] = -1;
  path[9] = -1;

  display_counter = 0;
  
  moving = false;
  scouting = true;
  mapping = false;
  thinking = true;   //set to true for full code
  planning = true;   //set to true for full code, also uncomment the moving part of check_status

  done = false;
}

//----------------------------------helper functions

int get_distance(int goal_index, int current_index) {
  int vessel = 99;
  bool horizontal = abs(goal_index - current_index) == 1 && 
                    abs(goal_index%4 - current_index%4) != 3;
  bool vertical = abs(goal_index - current_index) == 4;
  if (vertical || horizontal) {
    int x1 = goal_index%4;
    int y1 = goal_index/4;
    int x2 = current_index%4;
    int y2 = current_index/4;
    if (env_map[x1][y1] == 1 && env_map[x2][y2] == 1) {vessel = 1;}
  }
  return vessel;
}
