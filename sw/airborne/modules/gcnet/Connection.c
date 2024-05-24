#include "Connection.h"

#include <stdio.h>
#include <stdlib.h>

// Build connection
Connection build_connection(int const post, int const pre) {
  // Connection struct
  Connection c;

  // Set shape
  c.post = post;
  c.pre = pre;

  // Allocate memory for weight array
  c.w = calloc(post * pre, sizeof(*c.w));

  return c;
}

// Init connection
void init_connection(Connection *c) {
  // Loop over weights
  for (int i = 0; i < c->post; i++) {
    for (int j = 0; j < c->pre; j++) {
      c->w[i * c->pre + j] = rand() / (float)RAND_MAX;
    }
  }
}

// Reset connection
// Doesn't actually do anything, just for consistency
void reset_connection(Connection *c) {}

// Load parameters for connection (weights) from a header file
// (using the ConnectionConf struct)
void load_connection_from_header(Connection *c, ConnectionConf const *conf) {
  // Check if same shape
  if ((c->pre != conf->pre) || (c->post != conf->post)) {
    printf("Connection has a different shape than specified in the "
           "ConnectionConf!\n");
    exit(1);
  }
  // Loop over weights
  // TODO: could also be done by just exchanging pointers to arrays?
  for (int i=0; i<c->post*c->pre; i++){
    c->w[i] = conf->w[i];
  }
}

// Free allocated memory for connection
void free_connection(Connection *c) {
  // calloc() was used for weight array
  // Only one call, so only one free (as opposed to other methods for 2D arrays)
  free(c->w);
}

// Forward
// Spikes as floats to deal with real-valued inputs
void forward_connection_float(Connection *c, float out[], float const in[]) {
  // Loop over weights and multiply with spikes
  for (int i = 0; i < c->post; i++) {
    for (int j = 0; j < c->pre; j++) {
      out[i] += c->w[i*c->pre + j] * in[j];
    }
  }
}

// Forward
// Spikes as floats to deal with real-valued inputs
void forward_connection_int(Connection *c, float out[], int const in[]) {
  // Loop over weights and multiply with spikes
  for (int i = 0; i < c->post; i++) {
    for (int j = 0; j < c->pre; j++) {
      out[i] += c->w[i*c->pre + j] * in[j];
    }
  }
}
