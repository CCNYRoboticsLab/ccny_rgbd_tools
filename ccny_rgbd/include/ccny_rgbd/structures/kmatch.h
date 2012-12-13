/*
 * kmatch.h
 *
 *  Created on: Dec 13, 2012
 *      Author: carlos
 */

#ifndef KMATCH_H_
#define KMATCH_H_

#include <set>

namespace ccny_rgbd
{

class KMatch
{
public:
  typedef std::set<KMatch> kMatchSet;
  typedef std::set<KMatch>::iterator kMatchSetIterator;
  typedef std::set<KMatch>::const_iterator ckMatchSetIterator;


  KMatch(int ref_idx, float dist);
  KMatch(const KMatch& m);
  ~KMatch()
  {
  }

  KMatch &
  operator=(const KMatch& m);

  bool
  operator<(const KMatch& m) const;

private:
  int ref_id_;
  float dist_;
};

} //namespace ccny_rgbd



#endif /* KMATCH_H_ */
