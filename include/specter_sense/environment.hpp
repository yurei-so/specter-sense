#pragma once

#include <filesystem>

namespace specter {

// Loads KEY=VALUE entries without replacing variables already exported by the caller.
// A missing file is allowed so packaged command-line use remains self-contained.
void load_dotenv_if_present(const std::filesystem::path& path);

}  // namespace specter
