#ifndef CCNY_RGBD_HEADER_H
#define CCNY_RGBD_HEADER_H

namespace ccny_rgbd {

struct Time {
  int32_t sec;
  int32_t nsec;
};

struct Header {
  uint32_t seq;
  Time stamp;
  std::string frame_id;
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_HEADER_H
