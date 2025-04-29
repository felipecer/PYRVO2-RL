#ifndef RL_EXTENSIONS_STRUCTS
#define RL_EXTENSIONS_STRUCTS

namespace RL_EXTENSIONS
{
  struct BatchAgentData
  {
    int id;
    float px, py;
    float vx, vy;
    float pvx, pvy;
    float dist_goal;
  };

  struct NeighborDataObsCart
  {
    float pos_x;
    float pos_y;
    float vel_x;
    float vel_y;
    float pref_vel_x;
    float pref_vel_y;
  };

  struct NeighborDataObsPolar
  {
    float pos_x;
    float pos_y;
    float vel_mag;
    float vel_angle;
    float pref_vel_mag;
    float pref_vel_angle;
  };
} // namespace RL_EXTENSIONS
#endif