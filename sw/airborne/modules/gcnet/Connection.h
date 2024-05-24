#pragma once

// Struct that defines a connection between two layers of neurons
typedef struct Connection {
  // Connection shape: (post, pre)
  int post, pre;
  // Connection weights
  // Use a 1D array, since structs don't support variable-sized arrays
  // TODO: is this the best way to go? Would like to be able to do w[i][j]
  // Check this with Erik
  float *w;
} Connection;

// Struct that holds the configuration (weights) of a connection
// To be used when loading parameters from a header file
typedef struct ConnectionConf {
  // Connection shape: (post, pre)
  int const post, pre;
  // Connection weights (1D array)
  // TODO: or actual weight array? Might be easier to specify in conf header..
  float const *w;
} ConnectionConf;

// Build connection
Connection build_connection(int const post, int const pre);

// Init connection
void init_connection(Connection *c);

// Reset connection
// Doesn't actually do anything, just for consistency
void reset_connection(Connection *c);

// Load parameters (weights) for connection from header file
// (using the ConnectionConf struct)
void load_connection_from_header(Connection *c, ConnectionConf const *conf);

// Free allocated memory for connection
void free_connection(Connection *c);

// Forward
// Spikes as floats to deal with real-valued inputs
void forward_connection_float(Connection *c, float out[], float const in[]);

void forward_connection_int(Connection *c, float out[], int const in[]);
