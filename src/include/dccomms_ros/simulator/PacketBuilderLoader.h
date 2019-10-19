#ifndef DCCOMMSROS_PACKETBUILDERFACTORYLOADER_H
#define DCCOMMSROS_PACKETBUILDERFACTORYLOADER_H

#include <class_loader/multi_library_class_loader.h>
#include <dccomms/IPacketBuilder.h>
#include <dccomms_ros/simulator/PacketBuilderLoader.h>
#include <string>
#include <exception>

namespace dccomms_ros {

#define PB_LOADER_EXCEPTION_NOCLASSFOUND 0

class PacketBuilderLoaderException : public std::exception {
public:
  PacketBuilderLoaderException(std::string msg, int cod);
  virtual const char *what() const throw() { return message.c_str(); }
  int code;

private:
  std::string message;
};

class PacketBuilderLoader {
public:
  static dccomms::PacketBuilderPtr
  LoadPacketBuilder(const std::string &libName, const std::string className);

private:
  // static class_loader::MultiLibraryClassLoader _loader;
};
}
#endif
