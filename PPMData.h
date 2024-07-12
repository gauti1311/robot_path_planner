#pragma once

#include <fstream>
#include <ios>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace ppm {
/// PPMObject which stores the RGB Data and the Header Information
struct PPMObject
{
  std::string magicNum{};
  std::string magicNum2{};
  int width{};
  int height{};
  int maxColVal{};
  std::vector<char> rgb_data{};
};

inline std::istream& operator>>(std::istream& inputStream, PPMObject& other)
{
  std::getline(inputStream, other.magicNum);
  std::getline(inputStream, other.magicNum2);
  inputStream >> other.width >> other.height >> other.maxColVal;
  inputStream.get(); // skip the trailing white space
  std::size_t size = other.width * other.height * 3U;
  other.rgb_data.resize(size);
  inputStream.read(other.rgb_data.data(), size);
  return inputStream;
}

inline std::ostream& operator<<(std::ostream& outputStream, const PPMObject& other)
{
  outputStream << "P6"
               << "\n"
               << "# Created by GIMP version 2.10.30 PNM plug-in"
               << "\n"
               << other.width << " " << other.height << "\n"
               << other.maxColVal << "\n";
  std::size_t size = other.width * other.height * 3U;
  outputStream.write(other.rgb_data.data(), size);
  return outputStream;
}

/// Loads a file into a PPMObject
/// @returns the PPMObject, empty optional if file could not be opened
inline std::optional<PPMObject> load_file(const std::string filename)
{
  std::fstream file(filename);
  if (file.is_open())
  {
    PPMObject obj{};
    file >> obj;
    file.close();
    return {obj};
  }
  return std::nullopt;
}

inline void write_file(const std::string filename, const PPMObject& ppm)
{
  std::ofstream file(filename, std::ios::binary);
  if (file)
  {
    file << ppm;
    file.close();
  }
}
} // namespace ppm