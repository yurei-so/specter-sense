#pragma once

#include "specter_sense/types.hpp"

#include <filesystem>

namespace specter {

AppConfig load_config(const std::filesystem::path& path);
void validate_config(const AppConfig& config);

}  // namespace specter
