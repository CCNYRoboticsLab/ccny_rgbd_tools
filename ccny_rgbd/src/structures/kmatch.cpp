/*
 * kmatch.cpp
 *
 *  Created on: Dec 13, 2012
 *      Author: carlos
 */

#include <ccny_rgbd/structures/kmatch.h>

namespace ccny_rgbd {

KMatch::KMatch(int ref_idx, float dist)
{
  ref_id_ = ref_idx;
  dist_ = dist;
}

KMatch::KMatch(const KMatch& m)
{
  ref_id_ = m.ref_id_;
  dist_ = m.dist_;
}

KMatch& KMatch::operator=(const KMatch &m)
{
  ref_id_ = m.ref_id_;
  dist_ = m.dist_;

  return *this;
}


bool KMatch::operator<(const KMatch& m) const
{
   return (dist_ < m.dist_);
}

} //namespace ccny_rgbd




