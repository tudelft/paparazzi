#include <fstream>
#include <iostream>

int main(int argc, char *argv[])
{
  if (argc < 2) {
    std::cerr << "SODA: no image was provided" << std::endl;
    return 1;
  }

  std::ifstream image(argv[1], std::ios::binary | std::ios::ate);
  if (!image || image.tellg() <= 0) {
    std::cerr << "SODA: could not inspect image " << argv[1] << std::endl;
    return 1;
  }

  std::cout << "SODA: I looked at the image. Cool, right?" << std::endl;
  return 0;
}