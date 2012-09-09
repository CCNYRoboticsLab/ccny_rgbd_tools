
template <typename PointT>
ccny_rgbd::FeatureHistory<PointT>::FeatureHistory():
  index_(0),
  capacity_(5)
{


}

template <typename PointT>
ccny_rgbd::FeatureHistory<PointT>::~FeatureHistory()
{


}

template <typename PointT>
void ccny_rgbd::FeatureHistory<PointT>::add(const PointCloudT& cloud)
{
  if (history_.size() < capacity_)
  {
   history_.push_back(cloud);

   ++index_;
   if (index_ >= capacity_) index_ = 0;
  }
  else if (capacity_ > 0)
  {
   history_[index_] = cloud;

   ++index_;
   if (index_ >= capacity_) index_ = 0;
  }
}

template <typename PointT>
void ccny_rgbd::FeatureHistory<PointT>::getAll(PointCloudT& cloud)
{
  for (unsigned int i=0; i < history_.size(); ++i)
    cloud += history_[i];
}

template <typename PointT>
void ccny_rgbd::FeatureHistory<PointT>::reset()
{
  history_.clear();
  index_ = 0;
}
