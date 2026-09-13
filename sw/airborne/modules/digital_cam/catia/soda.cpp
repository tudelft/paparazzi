/** @file soda.cpp @brief Small validated command-line dispatch point for post-capture analysis handlers. */
#include <fstream>
#include <iostream>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <vector>
#include "version.h"

namespace {
void print_usage(const char *program)
{
  std::cout << SODA_BUILD_VERSION << '\n'
            << "Usage: " << program << " [OPTIONS] IMAGE [10 shot integers]\n"
            << "  --local     Development-PC mode; no hardware or remote connection.\n"
            << "  --aicam     Select the AI camera handler.\n"
            << "  --chdkcam   Select the CHDK camera handler.\n"
            << "  --lwircam   Select the thermal camera handler.\n"
            << "  --earcam    Select the acoustic camera handler.\n"
            << "  --version   Show application name, version, and build Git revision.\n"
            << "  --help      Show this help.\n"
            << "  --          End option parsing.\n"
            << "Select at most one camera; otherwise use the generic handler.\n"
            << "IMAGE must be readable and nonempty. Optional int32 shot values:\n"
            << "number latitude longitude altitude roll pitch yaw speed course groundalt\n";
}

void process_aicam()
{
  std::cout << "Now I can do nifty stuff for aicam" << std::endl;
}

void process_chdkcam()
{
  std::cout << "Now I can do nifty stuff for chdkcam" << std::endl;
}

void process_lwircam()
{
  std::cout << "Now I can do nifty stuff for lwircam" << std::endl;
}

void process_earcam()
{
  std::cout << "Now I can do nifty stuff for earcam" << std::endl;
}

struct CameraHandler {
  const char *option;
  void (*process)();
};

const CameraHandler handlers[] = {
  {"--aicam", process_aicam},
  {"--chdkcam", process_chdkcam},
  {"--lwircam", process_lwircam},
  {"--earcam", process_earcam}
};

bool is_shot_value(const char *text)
{
  const char *digit = text;
  if (*digit == '-' || *digit == '+') ++digit;
  if (*digit == '\0') return false;
  for (; *digit != '\0'; ++digit) {
    if (*digit < '0' || *digit > '9') return false;
  }
  errno = 0;
  char *end = nullptr;
  const long long value = std::strtoll(text, &end, 10);
  return errno != ERANGE && *end == '\0'
         && value >= std::numeric_limits<int32_t>::min()
         && value <= std::numeric_limits<int32_t>::max();
}
}

int main(int argc, char *argv[])
{
  const CameraHandler *selected = nullptr;
  std::vector<const char *> positional;
  bool options = true;
  bool local = false;
  bool help = false;
  bool version = false;
  for (int argument = 1; argument < argc; ++argument) {
    const char *value = argv[argument];
    if (options && std::strcmp(value, "--") == 0) {
      options = false;
      continue;
    }
    if (options && std::strcmp(value, "--local") == 0) {
      if (local) {
        std::cerr << "SODA: duplicate --local option" << std::endl;
        return 2;
      }
      local = true;
      continue;
    }
    if (options && std::strcmp(value, "--help") == 0) {
      help = true;
      continue;
    }
    if (options && std::strcmp(value, "--version") == 0) {
      version = true;
      continue;
    }
    const CameraHandler *match = nullptr;
    if (options) {
      for (const CameraHandler &handler : handlers) {
        if (std::strcmp(value, handler.option) == 0) match = &handler;
      }
    }
    if (match != nullptr) {
      if (selected != nullptr) {
        std::cerr << "SODA: specify only one camera selector" << std::endl;
        return 2;
      }
      selected = match;
    } else if (options && value[0] == '-' && !is_shot_value(value)) {
      std::cerr << "SODA: unknown option " << value << std::endl;
      return 2;
    } else {
      positional.push_back(value);
    }
  }
  if (help) {
    print_usage(argv[0]);
    return 0;
  }
  if (version) {
    std::cout << SODA_BUILD_VERSION << std::endl;
    return 0;
  }
  if (positional.empty()) {
    std::cerr << "SODA: no image was provided" << std::endl;
    return 1;
  }
  if (positional.size() != 1 && positional.size() != 11) {
    std::cerr << "SODA: expected IMAGE optionally followed by 10 shot integers" << std::endl;
    return 2;
  }
  for (size_t index = 1; index < positional.size(); ++index) {
    if (!is_shot_value(positional[index])) {
      std::cerr << "SODA: invalid shot integer " << positional[index] << std::endl;
      return 2;
    }
  }

  std::ifstream image(positional[0], std::ios::binary | std::ios::ate);
  if (!image || image.tellg() <= 0) {
    std::cerr << "SODA: could not inspect image " << positional[0] << std::endl;
    return 1;
  }

  if (local) std::cerr << "SODA: development-PC mode (--local)" << std::endl;
  if (selected != nullptr) {
    selected->process();
  } else {
    std::cout << "SODA: I looked at the image. Cool, right?" << std::endl;
  }
  return 0;
}