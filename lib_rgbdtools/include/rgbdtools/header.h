#ifndef RGBD_HEADER_H
#define RGBD_HEADER_H

namespace rgbdtools {

struct Time {
  int32_t sec;
  int32_t nsec;
};

struct Header {
  uint32_t seq;
  Time stamp;
  std::string frame_id;
};

} // namespace rgbdtools

#endif // RGBDTOOLS_HEADER_H
