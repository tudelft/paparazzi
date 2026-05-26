/*
 * Paparazzi terrain module to read AP compatible .DAT files and provide terrain height information
 */

#ifndef TERRAIN_H
#define TERRAIN_H

#include "std.h"
#include "math/pprz_geodetic_int.h"

// Initialize the terrain module (can setup cache and state)
extern void terrain_init(void);

/**
 * Get terrain height AMSL (Above Mean Sea Level)
 * @param loc struct LlaCoor_i pointer with lat and lon populated (degrees * 1e7)
 * @param height Pointer to float where the terrain height in meters will be stored
 * @return true if terrain height could be determined, false otherwise
 *
 * Example usage:
 * \code
 * struct LlaCoor_i my_loc;
 * my_loc.lat = 407127760;  // 40.712776 N
 * my_loc.lon = -740059740; // 74.005974 W
 * float my_height;
 * if (terrain_get_height_amsl(&my_loc, &my_height)) {
 *     // my_height now contains the terrain height in meters
 * }
 * \endcode
 */
extern bool terrain_get_height_amsl(const struct LlaCoor_i *loc, float *height);

#endif /* TERRAIN_H */
