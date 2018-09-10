


struct dronerace_ransac_struct
{
  // Settings
  float dt_max;

  // States
  int buf_index_of_last;
  int buf_size;
};

extern struct dronerace_ransac_struct dr_ransac;



extern void ransac_reset(void);
extern void ransac_push(float time, float x, float y, float mx, float my);


