#include "third_party/optuna.h"

namespace optuna
{

std::string run_command(const std::string& command)
{
  auto fp = popen(command.c_str(), "r");
  std::string ret;
  constexpr int PATH_MAX_OPTUNA = 65536;
  char path[PATH_MAX_OPTUNA];
  while (fgets(path, PATH_MAX_OPTUNA, fp) != NULL)
  {
    ret += path;
  }
  pclose(fp);
  return ret;
}
};  // namespace optuna