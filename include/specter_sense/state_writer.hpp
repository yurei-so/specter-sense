#pragma once

#include "specter_sense/types.hpp"

#include <boost/json/value.hpp>
#include <filesystem>

namespace specter {

boost::json::value snapshot_to_json(const Snapshot& snapshot);
void write_snapshot_atomic(const std::filesystem::path& path, const Snapshot& snapshot);

}  // namespace specter
