#ifndef GAZEBO_OMNI_DRIVE_HPP_
#define GAZEBO_OMNI_DRIVE_HPP_

#include <gazebo_omni_drive_private.hpp>
#include <gazebo/common/Plugin.hh>
#include <memory>

namespace gazebo_omni_drive_plugins
{
class GazeboOmniRosDrive : public gazebo::ModelPlugin
{
  public:
    /// Constructor
    GazeboOmniRosDrive();
  
    /// Destructor
    ~GazeboOmniRosDrive();
  
  protected:
    // Documentation inherited
    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    // Documentation inherited
    void Reset() override;
  private:
  /// Private data pointer
    std::unique_ptr<GazeboOmniRosDrivePrivate> impl_;
};
}  // namespace gazebo_plugins



#endif