#pragma once

#include "specter_sense/types.hpp"

#include <filesystem>
#include <string>

namespace specter {

AppConfig load_config(const std::filesystem::path& path);
void validate_config(const AppConfig& config);
std::string serialize_config(const AppConfig& config);
void save_config_atomic(const std::filesystem::path& path, const AppConfig& config);

}  // namespace specter
