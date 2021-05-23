#ifndef DifferentialDrive_h
#define DifferentialDrive_h

struct wheel_properties{
  int dir_pin;
  int step_pin;
  bool dir;
  float interval;
  int compare_register_value;
  bool step_state;
  float velocity;
  long steps;
};

struct control_command{
  float linear;
  float angular;
};

typedef struct wheel_properties Wheel;
typedef struct control_command Control;

class DifferentialDrive{
  private:
    int en_pin;
    
    float body_radius;
    float wheel_radius;

    Control c;
    Wheel right_wheel;
    Wheel left_wheel;

    float current_x = 0;
    float current_y = 0;
    float current_theta = 0;

    void update_control_velocities(float linear, float angular);
    void update_wheel_velocities();
    void udpate_intervals();
    void update_compare_register_value();
    void update_registers();

  public:
    DifferentialDrive(float b, float w);
    
    void init_ISR();
    void control(float linear, float angular);
    void move_angle(float theta);
    void move_distance(float theta);
    void move_it(float x, float y, float theta);
    void left_step();
    void right_step();
};

extern DifferentialDrive robot;

#endif
