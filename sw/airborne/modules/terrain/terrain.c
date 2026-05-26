
/*
 * Paparazzi terrain module
 * 
 * Why: This module parses AP compatible .DAT terrain files stored on an SD card or other internal storage
 * to evaluate the altitude Above Mean Sea Level (AMSL) of any given coordinate.
 * It caches 2K data blocks (LRU cache) to avoid heavy block reads to the storage medium
 */

#include "modules/terrain/terrain.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "mcu_periph/sys_time.h"
#include "math/pprz_geodetic_float.h"
#include <stddef.h>

// Provide the path mapping to where the .DAT files are stored
#ifndef TERRAIN_DATA_DIR
#define TERRAIN_DATA_DIR "/TERRAIN"
#endif

/*
 * MAVLink terrain grids are 4x4, SD cards store an 8x7 matrix of them.
 * Why: This ensures efficient 2048-byte SD block alignments and easy
 * synchronization with MAVLink chunks.
 */
#define TERRAIN_GRID_MAVLINK_SIZE 4
#define TERRAIN_GRID_BLOCK_MUL_X 7
#define TERRAIN_GRID_BLOCK_MUL_Y 8
#define TERRAIN_GRID_BLOCK_SIZE_X (TERRAIN_GRID_MAVLINK_SIZE * TERRAIN_GRID_BLOCK_MUL_X)
#define TERRAIN_GRID_BLOCK_SIZE_Y (TERRAIN_GRID_MAVLINK_SIZE * TERRAIN_GRID_BLOCK_MUL_Y)
#define TERRAIN_GRID_BLOCK_SPACING_X ((TERRAIN_GRID_BLOCK_MUL_X - 1) * TERRAIN_GRID_MAVLINK_SIZE)
#define TERRAIN_GRID_BLOCK_SPACING_Y ((TERRAIN_GRID_BLOCK_MUL_Y - 1) * TERRAIN_GRID_MAVLINK_SIZE)

// Number of 2-kilobyte cache blocks to hold in memory
#define TERRAIN_GRID_BLOCK_CACHE_SIZE 12
#define TERRAIN_GRID_FORMAT_VERSION 1

/*
 * The data block format stored directly on disk.
 * Why packed: The exact layout must correspond byte-by-byte to ArduPilot's disk format
 * so that we can load the 2048 bytes blindly and cast.
 */
struct grid_block {
    // Bitmap indicating which 4x4 MAVLink sub-grids are populated (56 bits used)
    uint64_t bitmap;
    // South-West corner of the grid block in degrees * 1e7
    int32_t lat;
    int32_t lon;
    // CRC for data integrity checking
    uint16_t crc;
    // Schema version to catch legacy/incompatible files
    uint16_t version;
    // Ground distance (in meters) between individual grid points
    uint16_t spacing;
    // Grid altitude values in meters above sea level
    int16_t height[TERRAIN_GRID_BLOCK_SIZE_X][TERRAIN_GRID_BLOCK_SIZE_Y];
    // Coordinate indices for addressing the exact 32x28 blocks
    uint16_t grid_idx_x;
    uint16_t grid_idx_y;
    // South-West reference of the bounding rounded degree block
    int16_t lon_degrees;
    int8_t lat_degrees;
    // Appended retroactively in AP, thus excluded from the CRC calc so old files don't fail
    uint8_t version_minor;
} __attribute__((packed));

union grid_io_block {
    struct grid_block block;
    uint8_t buffer[2048]; //TODO: Optimize
};

enum GridCacheState {
    GRID_CACHE_INVALID = 0,
    GRID_CACHE_VALID = 2,
};

// Represents a memory-resident block along with eviction metadata
struct grid_cache {
    struct grid_block grid;
    enum GridCacheState state;
    uint32_t last_access_ms; // Why: Enables a simple standard LRU (Least Recently Used) caching policy
};

// Structural decomposition of a queried coordinate against the grid
struct grid_info {
    int8_t lat_degrees;
    int16_t lon_degrees;
    int32_t grid_lat;
    int32_t grid_lon;
    uint16_t grid_idx_x;
    uint16_t grid_idx_y;
    uint8_t idx_x;
    uint8_t idx_y;
    float frac_x;
    float frac_y;
};

static struct grid_cache cache[TERRAIN_GRID_BLOCK_CACHE_SIZE];
// Default spacing is 100 meters per point in standard setups
static uint16_t grid_spacing = 100;//30 is also common for higher resolution
static bool initialized = false;

/*
 * Basic CRC-16 CCITT
 * Why: Original AP DAT terrain blocks are signed with CCITT16 to prevent reading
 * corrupted map sectors, avoiding e.g. collisions into fake mountains.
 */
static uint16_t crc16_ccitt(const uint8_t *data, size_t length, uint16_t crc) {
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

static uint16_t get_block_crc(struct grid_block *block) {
    uint16_t saved_crc = block->crc;
    block->crc = 0;
    // Exclude the version_minor at the end from checksum to remain compatible
    // with earliest deployed databases
    uint16_t ret = crc16_ccitt((const uint8_t *)block, offsetof(struct grid_block, version_minor), 0);
    block->crc = saved_crc;
    return ret;
}

/*
 * Spherical distance algorithm, specifically duplicating Ardupilot's logic.
 * Why: By duplicating 'Location::get_distance_NE', we ensure integer index
 * snapping remains mathematically identical to exactly which block MAVProxy wrote.
 */
static void location_get_dist_NE(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2, float *dist_n, float *dist_e) {
    float lat_rad = ((float)lat1) * 1.0e-7f * (float)M_PI / 180.0f;
    float cos_lat = cosf(lat_rad);
    *dist_n = (lat2 - lat1) * 0.01113195f;
    *dist_e = (lon2 - lon1) * 0.01113195f * cos_lat;
}

// Spherical location extrusion algorithm complementing AP
static void location_offset(int32_t *lat, int32_t *lon, float dist_n, float dist_e) {
    float lat_rad = ((float)*lat) * 1.0e-7f * (float)M_PI / 180.0f;
    float cos_lat = cosf(lat_rad);
    // Limit convergence near the poles
    if (cos_lat < 0.05f) cos_lat = 0.05f;
    *lat += (int32_t)(dist_n / 0.01113195f);
    *lon += (int32_t)(dist_e / (0.01113195f * cos_lat));
}

/*
 * Determine how many disk blocks exist across a longitudinal band of coordinates.
 * Why: Required to compute the 1D absolute file offset within the square file chunk.
 */
static uint32_t east_blocks(struct grid_block *block) {
    int32_t loc1_lat = block->lat_degrees * 10000000;
    int32_t loc1_lon = block->lon_degrees * 10000000;
    int32_t loc2_lat = loc1_lat;
    int32_t loc2_lon = (block->lon_degrees + 1) * 10000000;
    
    location_offset(&loc2_lat, &loc2_lon, 0, 2.0f * grid_spacing * TERRAIN_GRID_BLOCK_SIZE_Y);
    
    float dn = 0, de = 0;
    location_get_dist_NE(loc1_lat, loc1_lon, loc2_lat, loc2_lon, &dn, &de);
    return (uint32_t)(de / (grid_spacing * TERRAIN_GRID_BLOCK_SPACING_Y));
}

static void calculate_grid_info(const struct LlaCoor_i *loc, struct grid_info *info) {
    // Find absolute rounded degree chunk
    // Why: File naming strictly divides the world into 1-degree tiles
    info->lat_degrees = (loc->lat < 0 ? (loc->lat - 9999999) : loc->lat) / 10000000;
    info->lon_degrees = (loc->lon < 0 ? (loc->lon - 9999999) : loc->lon) / 10000000;

    int32_t ref_lat = info->lat_degrees * 10000000;
    int32_t ref_lon = info->lon_degrees * 10000000;

    float offset_n, offset_e;
    location_get_dist_NE(ref_lat, ref_lon, loc->lat, loc->lon, &offset_n, &offset_e);

    uint32_t idx_x = (uint32_t)(offset_n / grid_spacing);
    uint32_t idx_y = (uint32_t)(offset_e / grid_spacing);

    // Divisors yield what SD block it's inside
    info->grid_idx_x = idx_x / TERRAIN_GRID_BLOCK_SPACING_X;
    info->grid_idx_y = idx_y / TERRAIN_GRID_BLOCK_SPACING_Y;

    // Modulos extract exact coordinates inside the tiny 32x28 window
    info->idx_x = idx_x % TERRAIN_GRID_BLOCK_SPACING_X;
    info->idx_y = idx_y % TERRAIN_GRID_BLOCK_SPACING_Y;

    // Fracs preserve the intermediate unrounded floating spacing
    // Why: To allow 4-point bilinear interpolation across adjacent sampled edges
    info->frac_x = (offset_n - idx_x * grid_spacing) / grid_spacing;
    info->frac_y = (offset_e - idx_y * grid_spacing) / grid_spacing;

    // Resolve exactly the coordinate the AP expects this subgrid's SW edge to sit at
    int32_t grid_sw_lat = ref_lat;
    int32_t grid_sw_lon = ref_lon;
    location_offset(&grid_sw_lat, &grid_sw_lon, 
                    info->grid_idx_x * TERRAIN_GRID_BLOCK_SPACING_X * grid_spacing,
                    info->grid_idx_y * TERRAIN_GRID_BLOCK_SPACING_Y * grid_spacing);
    info->grid_lat = grid_sw_lat;
    info->grid_lon = grid_sw_lon;
}

// Validates the respective 4x4 subdivision was actually populated by analyzing the bitmap bit flags
static bool check_bitmap(struct grid_block *grid, uint8_t idx_x, uint8_t idx_y) {
    uint8_t bit_idx_x = idx_x / TERRAIN_GRID_MAVLINK_SIZE;
    uint8_t bit_idx_y = idx_y / TERRAIN_GRID_MAVLINK_SIZE;
    uint8_t bit_num = bit_idx_y * TERRAIN_GRID_BLOCK_MUL_X + bit_idx_x;
    return (grid->bitmap & (1ULL << bit_num)) != 0;
}

// Executes standard C IO reading to parse block out of DAT file
static bool read_block(struct grid_block *block) {
    char file_path[128];
    uint32_t lat_tmp = abs((int32_t)block->lat_degrees);
    if (lat_tmp > 99u) lat_tmp = 99u;
    uint32_t lon_tmp = abs((int32_t)block->lon_degrees);
    if (lon_tmp > 999u) lon_tmp = 999u;

    // Why: Format strictly uses absolute paths like /TERRAIN/N22E011.DAT layout.
    snprintf(file_path, sizeof(file_path), "%s/%c%02u%c%03u.DAT",
             TERRAIN_DATA_DIR,
             block->lat_degrees < 0 ? 'S' : 'N', lat_tmp,
             block->lon_degrees < 0 ? 'W' : 'E', lon_tmp);

    FILE *f = fopen(file_path, "rb");
    if (!f) return false;

    uint32_t num_east = east_blocks(block);
    uint32_t blocknum = num_east * block->grid_idx_x + block->grid_idx_y;
    uint32_t offset = blocknum * sizeof(union grid_io_block);

    fseek(f, offset, SEEK_SET);

    union grid_io_block io_block;
    size_t ret = fread(&io_block, 1, sizeof(io_block), f);
    fclose(f);

    if (ret != sizeof(io_block)) return false;
    
    if (io_block.block.bitmap == 0 ||
        io_block.block.spacing != grid_spacing ||
        io_block.block.version != TERRAIN_GRID_FORMAT_VERSION) {
        return false;
    }

    if (io_block.block.crc != get_block_crc(&io_block.block)) {
        return false;
    }

    memcpy(block, &io_block.block, sizeof(struct grid_block));
    return true;
}

/*
 * Inquires the ring cache buffers for an already-mapped piece of terrain memory.
 * If mapping doesn't exist, it evicts strictly the oldest (least recently used) buffer index.
 */
static struct grid_cache* find_grid_cache(struct grid_info *info) {
    uint16_t oldest_i = 0;
    uint32_t now_ms = get_sys_time_msec();

    for (uint16_t i = 0; i < TERRAIN_GRID_BLOCK_CACHE_SIZE; i++) {
        if (cache[i].state == GRID_CACHE_VALID &&
            cache[i].grid.grid_idx_x == info->grid_idx_x &&
            cache[i].grid.grid_idx_y == info->grid_idx_y &&
            cache[i].grid.lat_degrees == info->lat_degrees &&
            cache[i].grid.lon_degrees == info->lon_degrees &&
            cache[i].grid.spacing == grid_spacing) { // Cache Hit
            cache[i].last_access_ms = now_ms;
            return &cache[i];
        }
        if (cache[i].last_access_ms < cache[oldest_i].last_access_ms) {
            oldest_i = i;
        }
    }

    // Cache Miss; execute eviction protocol
    struct grid_cache *grid = &cache[oldest_i];
    memset(grid, 0, sizeof(*grid));
    
    grid->grid.lat = info->grid_lat;
    grid->grid.lon = info->grid_lon;
    grid->grid.spacing = grid_spacing;
    grid->grid.grid_idx_x = info->grid_idx_x;
    grid->grid.grid_idx_y = info->grid_idx_y;
    grid->grid.lat_degrees = info->lat_degrees;
    grid->grid.lon_degrees = info->lon_degrees;
    grid->last_access_ms = now_ms;

    // Reads blocking IO thread natively
    if (read_block(&grid->grid)) {
        grid->state = GRID_CACHE_VALID;
    }

    return grid;
}

void terrain_init(void) {
    if (initialized) return;
    memset(cache, 0, sizeof(cache));
    initialized = true;
}

bool terrain_get_height_amsl(const struct LlaCoor_i *loc, float *height) {
    if (!initialized) terrain_init();

    struct grid_info info;
    calculate_grid_info(loc, &info);

    struct grid_cache *grid_c = find_grid_cache(&info);
    if (grid_c->state != GRID_CACHE_VALID) {
        return false;
    }

    // Out of bounce guard clamping; 32x28
    if (info.idx_x > TERRAIN_GRID_BLOCK_SIZE_X - 2) info.idx_x = TERRAIN_GRID_BLOCK_SIZE_X - 2;
    if (info.idx_y > TERRAIN_GRID_BLOCK_SIZE_Y - 2) info.idx_y = TERRAIN_GRID_BLOCK_SIZE_Y - 2;

    struct grid_block *grid = &grid_c->grid;

    // Fail quickly if target bounds within 4x4 submatrices were never synchronized
    if (!check_bitmap(grid, info.idx_x,   info.idx_y) ||
        !check_bitmap(grid, info.idx_x,   info.idx_y+1) ||
        !check_bitmap(grid, info.idx_x+1, info.idx_y) ||
        !check_bitmap(grid, info.idx_x+1, info.idx_y+1)) {
        return false;
    }

    float h00 = grid->height[info.idx_x+0][info.idx_y+0];
    float h01 = grid->height[info.idx_x+0][info.idx_y+1];
    float h10 = grid->height[info.idx_x+1][info.idx_y+0];
    float h11 = grid->height[info.idx_x+1][info.idx_y+1];

    /*
     * Simple dual linear interpolation across altitude boundary planes.
     * Why: 100m is quite spaced out, using direct nearest neighbor 
     * results in "stair-casing" heights on ramps, causing flight issues.
     */
    float avg1 = (1.0f - info.frac_x) * h00 + info.frac_x * h10;
    float avg2 = (1.0f - info.frac_x) * h01 + info.frac_x * h11;
    float avg  = (1.0f - info.frac_y) * avg1 + info.frac_y * avg2;

    *height = avg;
    return true;
}
