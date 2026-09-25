#pragma once

#include <functional>
#include <vector>

namespace util {

void parallel(std::vector<std::function<void()>> tasks);

}
