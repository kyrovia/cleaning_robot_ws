#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace traj_cover
{

inline std::filesystem::path package_share_relative_path(
  const std::string & package,
  const std::string & relative_path)
{
  const std::filesystem::path path(relative_path);
  if (path.empty() || path.is_absolute()) {
    throw std::invalid_argument("package path must be non-empty and relative: " + relative_path);
  }
  const auto share = std::filesystem::path(
    ament_index_cpp::get_package_share_directory(package));
  const auto resolved = (share / path).lexically_normal();
  const auto relative = resolved.lexically_relative(share);
  if (relative.empty() || relative.string().rfind("..", 0) == 0) {
    throw std::invalid_argument("package path escapes its share directory: " + relative_path);
  }
  return resolved;
}

inline std::filesystem::path source_package_relative_path(
  const std::string & package,
  const std::string & relative_path)
{
  const std::filesystem::path path(relative_path);
  if (path.empty() || path.is_absolute()) {
    throw std::invalid_argument("package path must be non-empty and relative: " + relative_path);
  }
  // colcon 独立安装前缀是 <workspace>/install/<package>，源码在 <workspace>/src/<package>。
  const auto prefix = std::filesystem::path(ament_index_cpp::get_package_prefix(package));
  const auto source = (prefix / ".." / ".." / "src" / package).lexically_normal();
  if (!std::filesystem::is_directory(source)) {
    throw std::runtime_error("source package directory not found: " + source.string());
  }
  const auto resolved = (source / path).lexically_normal();
  const auto relative = resolved.lexically_relative(source);
  if (relative.empty() || relative.string().rfind("..", 0) == 0) {
    throw std::invalid_argument("package path escapes its source directory: " + relative_path);
  }
  return resolved;
}

}  // namespace traj_cover
