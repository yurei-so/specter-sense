#include "specter_sense/environment.hpp"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>

namespace specter {
namespace {

std::string trim(std::string value) {
  while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front()))) value.erase(value.begin());
  while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) value.pop_back();
  return value;
}

bool valid_key(const std::string& key) {
  if (key.empty() || !(std::isalpha(static_cast<unsigned char>(key[0])) || key[0] == '_')) return false;
  for (const char character : key)
    if (!(std::isalnum(static_cast<unsigned char>(character)) || character == '_')) return false;
  return true;
}

}  // namespace

void load_dotenv_if_present(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    if (std::filesystem::exists(path)) throw std::runtime_error("cannot read environment file: " + path.string());
    return;
  }
  std::string line;
  std::size_t number{};
  while (std::getline(input, line)) {
    ++number;
    line = trim(line);
    if (line.empty() || line[0] == '#') continue;
    if (line.starts_with("export ")) line = trim(line.substr(7));
    const auto separator = line.find('=');
    if (separator == std::string::npos) throw std::runtime_error(path.string() + ":" + std::to_string(number) + ": expected KEY=VALUE");
    const auto key = trim(line.substr(0, separator));
    auto value = trim(line.substr(separator + 1));
    if (!valid_key(key)) throw std::runtime_error(path.string() + ":" + std::to_string(number) + ": invalid variable name");
    if (value.size() >= 2 && ((value.front() == '"' && value.back() == '"') ||
                            (value.front() == '\'' && value.back() == '\'')))
      value = value.substr(1, value.size() - 2);
    if (!std::getenv(key.c_str()) && ::setenv(key.c_str(), value.c_str(), 0) != 0)
      throw std::runtime_error("failed to set environment variable: " + key);
  }
}

}  // namespace specter
