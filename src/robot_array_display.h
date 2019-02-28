#ifndef RVIZ_ROBOT_ARRAY_DISPLAY_H_
#define RVIZ_ROBOT_ARRAY_DISPLAY_H_

#include <geometry_msgs/PoseArray.h>
#include <urdf/model.h>

#include <rviz/display.h>
#include <memory>

namespace Ogre
{
class ManualObject;
};

namespace rviz
{
class ColorProperty;
class FloatProperty;
class EnumProperty;
class Axes;
class Robot;
class StringProperty;
class RosTopicProperty;
class IntProperty;
}  // namespace rviz

namespace robot_array_plugin
{
/** @brief Displays a geometry_msgs/PoseArray message as a bunch of line-drawn arrows. */
class RobotArrayDisplay : public rviz::Display
{
    Q_OBJECT
  public:
    RobotArrayDisplay();
    virtual ~RobotArrayDisplay();

    // Overrides from Display
    virtual void onInitialize();
    virtual void update(float wall_dt, float ros_dt);
    virtual void fixedFrameChanged();
    virtual void reset();

    void clear();

  private Q_SLOTS:
    void updateVisualVisible();
    void updateCollisionVisible();
    void updateTfPrefix();
    void updateAlpha();
    void updateRobotDescription();
    void changedTopic();

  protected:
    /** @brief Loads a URDF from the ros-param named by our
     * "Robot Description" property, iterates through the links, and
     * loads any necessary models. */
    virtual void load();

    // overrides from Display
    virtual void onEnable();
    virtual void onDisable();

    ros::Subscriber pose_array_sub_;
    geometry_msgs::PoseArray::ConstPtr last_received_msg_;
    urdf::Model urdf_description_;
    std::vector<std::shared_ptr<rviz::Robot>> robots_;  ///< Handles actually drawing the robot

    bool has_new_transforms_;  ///< Callback sets this to tell our update function it needs to update the transforms

    float time_since_last_transform_;

    std::string robot_description_;

    rviz::Property* visual_enabled_property_;
    rviz::Property* collision_enabled_property_;
    rviz::FloatProperty* update_rate_property_;
    rviz::StringProperty* robot_description_property_;
    rviz::RosTopicProperty* pose_array_topic_property_;
    rviz::FloatProperty* alpha_property_;
    rviz::StringProperty* tf_prefix_property_;
    rviz::IntProperty* num_robots_to_display_property_;
    void processMessage(const geometry_msgs::PoseArray::ConstPtr& msg);
    void updateNumRobotsToDisplay(int n);
};

}  // namespace robot_array_plugin

#endif /* RVIZ_ROBOT_ARRAY_DISPLAY_H_ */
