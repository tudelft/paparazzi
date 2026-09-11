#include "../protocol.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

_Static_assert(sizeof(union catia_pose_sample_union) == CATIA_POSE_SAMPLE_MSG_SIZE, "Pose wire size");
_Static_assert(offsetof(union catia_pose_sample_union, data.shot) == 12, "Pose shot offset");
_Static_assert(offsetof(union catia_pose_sample_union, data.flags) == 96, "Pose flags offset");
_Static_assert(sizeof(union dc_shot_mask_union) == CATIA_SHOOT_MASK_MSG_SIZE, "Mask wire size");
_Static_assert(offsetof(union dc_shot_mask_union, data.camera_mask) == CATIA_SHOOT_MSG_SIZE, "Mask offset");
_Static_assert(CATIA_SHOOT_MASK != CATIA_SHOOT_TARGETED, "Mask and ID commands must differ");

static struct catia_transport transmit(const union catia_pose_sample_union *sample, unsigned int length)
{
  struct catia_transport transport = {0};
  uint8_t checksum_a = (uint8_t)(length + 5);
  uint8_t checksum_b = checksum_a;
  parse_catia(&transport, STX);
  parse_catia(&transport, checksum_a);
  parse_catia(&transport, CATIA_POSE_SAMPLE);
  checksum_a += CATIA_POSE_SAMPLE;
  checksum_b += checksum_a;
  for (unsigned int index = 0; index < length; ++index) {
    parse_catia(&transport, sample->bin[index]);
    checksum_a += sample->bin[index];
    checksum_b += checksum_a;
  }
  parse_catia(&transport, checksum_a);
  parse_catia(&transport, checksum_b);
  return transport;
}

int main(void)
{
  union catia_pose_sample_union sample = {0};
  sample.data.sequence = 0x01020304;
  sample.data.sample_begin_us = UINT32_MAX - 10;
  sample.data.sample_end_us = 12;
  sample.data.shot.data.lat = -488100000;
  sample.data.velocity_north_bfp = -1234;
  sample.data.flags = CATIA_POSE_SAMPLE_GPS_PRESENT;
  assert(sample.bin[0] == 4 && sample.bin[3] == 1);
  assert((uint32_t)(sample.data.sample_end_us - sample.data.sample_begin_us) == 23);
  struct catia_transport transport = transmit(&sample, CATIA_POSE_SAMPLE_MSG_SIZE);
  assert(transport.msg_received && transport.msg_id == CATIA_POSE_SAMPLE);
  assert(transport.payload_len == CATIA_POSE_SAMPLE_MSG_SIZE);
  for (size_t index = 0; index < sizeof(sample.bin); ++index) {
    assert(transport.payload[index] == sample.bin[index]);
  }
  transport = transmit(&sample, CATIA_POSE_SAMPLE_MSG_SIZE - 1);
  assert(transport.msg_received && transport.payload_len != CATIA_POSE_SAMPLE_MSG_SIZE);
  puts("Pose sample wire layout and transport tests passed");
}