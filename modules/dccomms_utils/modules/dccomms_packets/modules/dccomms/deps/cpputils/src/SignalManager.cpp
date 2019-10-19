#include <cpputils/SignalManager.h>

namespace cpputils {

std::list<std::function<void(int)>> SignalManager::_sigint_callbacks;
std::function<void(int)> SignalManager::_sigint_last_callback = [](int signal) {
};

void SignalManager::AddCallback(int signal, std::function<void(int)> handler) {
  std::signal(signal, &SignalManager::_topHandler);
  switch (signal) {
  case SIGINT: {
    _sigint_callbacks.push_back(handler);
    break;
  }
  }
}

void SignalManager::SetLastCallback(int signal,
                                    std::function<void(int)> handler) {
  std::signal(signal, &SignalManager::_topHandler);
  switch (signal) {
  case SIGINT: {
    _sigint_last_callback = handler;
    break;
  }
  }
}

void SignalManager::_topHandler(int signal) {
  switch (signal) {
  case SIGINT: {
    for (auto handler : _sigint_callbacks) {
      handler(signal);
    }
    _sigint_last_callback(signal);
    break;
  }
  }
}
}
