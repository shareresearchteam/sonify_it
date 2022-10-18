#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sstream>
#include <string>

namespace sonify_it {
class FUDIDecoder {
 public:
  uint8_t kStreamPrecision = 3;

  FUDIDecoder(const int port);
  bool activate();
  bool deactivate();
  bool is_activated();

  bool connect();
  bool disconnect();
  bool is_connected();

  bool has_error();
  std::string decode();
  bool has_message();

  void set_precision();

 private:
  struct sockaddr_in server_address_;
  std::string next_msg_;
  std::stringstream stream_;
  uint8_t stream_precision_;
};
}  // namespace sonify_it