extern volatile float vertical_position;
extern volatile float vertical_velocity;

void altFinit(float);
void altFupdate(float balt, float accz, float tdelta);
