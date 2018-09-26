


struct dronerace_fp_struct
{
  // Current Gate Position
  int gate_nr;
  float gate_x;
  float gate_y;
  float gate_alt;
  float gate_psi;

  // Current Navigation Target
  float x_set;
  float y_set;
  float psi_set;
  float alt_set;
};

// Variables
extern struct dronerace_fp_struct dr_fp;


// Functions
extern void flightplan_reset(void);
extern void flightplan_run(void);
