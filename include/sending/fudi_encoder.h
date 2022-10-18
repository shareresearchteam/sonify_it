#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <sstream>
#include <string>

namespace sonify_it {
class FUDIEncoder {
 public:
  uint8_t kStreamPrecision = 3;

  FUDIEncoder(const int port, const char* ip);
  bool connect();
  bool disconnect();
  bool is_connected();
  bool has_error();
  bool encode();

  void set_precision();

 private:
  struct sockaddr_in puredata_address_;
  std::stringstream stream_;
  uint8_t stream_precision_;
};
}  // namespace sonify_it