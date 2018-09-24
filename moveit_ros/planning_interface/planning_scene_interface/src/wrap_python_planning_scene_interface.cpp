/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/py_bindings_tools/roscpp_initializer.h>
#include <moveit/py_bindings_tools/py_conversions.h>
#include <moveit/py_bindings_tools/serialize_msg.h>

#include <boost/function.hpp>
#include <boost/python.hpp>
#include <Python.h>

/** @cond IGNORE */

namespace bp = boost::python;

namespace moveit
{
namespace planning_interface
{
class PlanningSceneInterfaceWrapper : protected py_bindings_tools::ROScppInitializer, public PlanningSceneInterface
{
public:
  // ROSInitializer is constructed first, and ensures ros::init() was called, if needed
  PlanningSceneInterfaceWrapper(const std::string& ns = "")
    : py_bindings_tools::ROScppInitializer(), PlanningSceneInterface(ns)
  {
  }

  bp::list getKnownObjectNamesPython(bool with_type = false)
  {
    return py_bindings_tools::listFromString(getKnownObjectNames(with_type));
  }

  bp::list getKnownObjectNamesInROIPython(double minx, double miny, double minz, double maxx, double maxy, double maxz,
                                          bool with_type = false)
  {
    return py_bindings_tools::listFromString(getKnownObjectNamesInROI(minx, miny, minz, maxx, maxy, maxz, with_type));
  }

  // std::map<std::string, geometry_msgs::Pose> getObjectPoses(const std::vector<std::string>& object_ids);
  bp::dict getObjectPosesPython(bp::list object_ids)
  {
    std::map<std::string, geometry_msgs::Pose> ops = getObjectPoses(py_bindings_tools::stringFromList(object_ids));
    std::map<std::string, std::string> ser_ops;
    for (std::map<std::string, geometry_msgs::Pose>::const_iterator it = ops.begin(); it != ops.end(); ++it)
      ser_ops[it->first] = py_bindings_tools::serializeMsg(it->second);

    return py_bindings_tools::dictFromType(ser_ops);
  }

  // std::map<std::string, moveit_msgs::CollisionObject> getObjects(const std::vector<std::string>& object_ids = std::vector<std::string>());
  bp::dict getObjectsPython(bp::list object_ids)
  {
    std::map<std::string, moveit_msgs::CollisionObject> objs =
        getObjects(py_bindings_tools::stringFromList(object_ids));
    std::map<std::string, std::string> ser_objs;
    for (std::map<std::string, moveit_msgs::CollisionObject>::const_iterator it = objs.begin(); it != objs.end(); ++it)
      ser_objs[it->first] = py_bindings_tools::serializeMsg(it->second);

    return py_bindings_tools::dictFromType(ser_objs);
  }

  // std::map<std::string, moveit_msgs::AttachedCollisionObject> getAttachedObjects(const std::vector<std::string>& object_ids = std::vector<std::string>());
  bp::dict getAttachedObjectsPython(const bp::list& object_ids)
  {
    std::map<std::string, moveit_msgs::AttachedCollisionObject> aobjs =
        getAttachedObjects(py_bindings_tools::stringFromList(object_ids));
    std::map<std::string, std::string> ser_aobjs;
    for (std::map<std::string, moveit_msgs::AttachedCollisionObject>::const_iterator it = aobjs.begin();
         it != aobjs.end(); ++it)
      ser_aobjs[it->first] = py_bindings_tools::serializeMsg(it->second);

    return py_bindings_tools::dictFromType(ser_aobjs);
  }

  // bool applyPlanningScene(const moveit_msgs::PlanningScene& ps);
  bool applyPlanningScenePython(const std::string& ps_str)
  {
    moveit_msgs::PlanningScene ps_msg;
    py_bindings_tools::deserializeMsg(ps_str, ps_msg);
    return applyPlanningScene(ps_msg);
  }

  // ==== old until here

  // bool applyCollisionObject(const moveit_msgs::CollisionObject& collision_object);
  bool applyCollisionObjectPython1(const std::string& co_str)
  {
    moveit_msgs::CollisionObject co_msg;
    py_bindings_tools::deserializeMsg(co_str, co_msg);
    return applyCollisionObject(co_msg);
  }

  // bool applyCollisionObject(const moveit_msgs::CollisionObject& collision_object,
  //                           const std_msgs::ColorRGBA& object_color);
  bool applyCollisionObjectPython2(const std::string& co_str, const std::string& color_str)
  {
    moveit_msgs::CollisionObject co_msg;
    py_bindings_tools::deserializeMsg(co_str, co_msg);
    std_msgs::ColorRGBA color_msg;
    py_bindings_tools::deserializeMsg(color_str, color_msg);
    return applyCollisionObject(co_msg, color_msg);
  }

  // bool applyCollisionObjects(
  //     const std::vector<moveit_msgs::CollisionObject>& collision_objects,
  //     const std::vector<moveit_msgs::ObjectColor>& object_colors = std::vector<moveit_msgs::ObjectColor>());
  bool applyCollisionObjectsPython(
      const bp::list& co_string_list,
      const bp::list& color_string_list)
  {
    int l = bp::len(co_string_list);
    std::vector<moveit_msgs::CollisionObject> co_msgs(l);
    std::vector<moveit_msgs::ObjectColor> color_msgs(l);
    // TODO felixvd: Check for empty color vector?
    for (int i = 0; i < l; ++i)
    {
      py_bindings_tools::deserializeMsg(bp::extract<std::string>(co_string_list[i]), co_msgs[i]);
      py_bindings_tools::deserializeMsg(bp::extract<std::string>(color_string_list[i]), color_msgs[i]);
    }
    return applyCollisionObjects(co_msgs, color_msgs);
  }

  // void addCollisionObjects(
  //     const std::vector<moveit_msgs::CollisionObject>& collision_objects,
  //     const std::vector<moveit_msgs::ObjectColor>& object_colors = std::vector<moveit_msgs::ObjectColor>()) const
  void addCollisionObjectsPython(
      const bp::list& collision_objects,
      const bp::list& object_colors) const
  {
    int l = bp::len(collision_objects);
    std::vector<moveit_msgs::CollisionObject> co_msgs(l);
    std::vector<moveit_msgs::ObjectColor> color_msgs(l);
    // TODO felixvd: Check for empty color vector?
    for (int i = 0; i < l; ++i)
    {
      py_bindings_tools::deserializeMsg(bp::extract<std::string>(collision_objects[i]), co_msgs[i]);
      py_bindings_tools::deserializeMsg(bp::extract<std::string>(object_colors[i]), color_msgs[i]);
    }
    addCollisionObjects(co_msgs, color_msgs);
  }

  // void removeCollisionObjects(const std::vector<std::string>& object_ids) const;
  void removeCollisionObjectsPython(const bp::list& object_ids) const
  {
    removeCollisionObjects(py_bindings_tools::stringFromList(object_ids));
  }

  // bool applyAttachedCollisionObject(const moveit_msgs::AttachedCollisionObject& attached_collision_object);
  bool applyAttachedCollisionObjectPython(const std::string& attached_collision_object)
  {
    moveit_msgs::AttachedCollisionObject aco_msg;
    py_bindings_tools::deserializeMsg(attached_collision_object, aco_msg);
    return applyAttachedCollisionObject(aco_msg);
  }

  // bool
  // applyAttachedCollisionObjects(const std::vector<moveit_msgs::AttachedCollisionObject>& attached_collision_objects);
  bool
  applyAttachedCollisionObjectsPython(const bp::list& attached_collision_objects)
  {
    int l = bp::len(attached_collision_objects);
    std::vector<moveit_msgs::AttachedCollisionObject> aco_msgs(l);
    for (int i = 0; i < l; ++i)
    {
      py_bindings_tools::deserializeMsg(bp::extract<std::string>(attached_collision_objects[i]), aco_msgs[i]);
    }
    return applyAttachedCollisionObjects(aco_msgs);
  }

  // bool allowCollisions(const std::string& link_name_1, const std::string& link_name_2 = "");
  bool allowCollisionsPython1(const std::string& link_name_1, const std::string& link_name_2 = "")
  {
    return allowCollisions(link_name_1, link_name_2);
  }

  // bool allowCollisions(const std::vector<std::string>& link_group_1, const std::string& link_name_2 = "");
  bool allowCollisionsPython2(const bp::list& link_group_1, const std::string& link_name_2 = "")
  {
    return allowCollisions(py_bindings_tools::stringFromList(link_group_1), link_name_2);
  }

  // bool disallowCollisions(const std::string& link_name_1, const std::string& link_name_2 = "");
  bool disallowCollisionsPython1(const std::string& link_name_1, const std::string& link_name_2 = "")
  {
    return disallowCollisions(link_name_1, link_name_2);
  }
  // bool disallowCollisions(const std::vector<std::string>& link_group_1, const std::string& link_name_2 = "");
  bool disallowCollisionsPython2(const bp::list& link_group_1, const std::string& link_name_2 = "")
  {
    return disallowCollisions(py_bindings_tools::stringFromList(link_group_1), link_name_2);
  }

  bool setCollisionsPython1(bool set_to_allow, bp::list& link_group_1, 
                                        bp::list& link_group_2)
  {
    return setCollisions(set_to_allow, py_bindings_tools::stringFromList(link_group_1), py_bindings_tools::stringFromList(link_group_2));
  }

  // bool setCollisions(bool set_to_allow, const std::string& link_name_1, 
  //                                       const std::vector<std::string>& link_group_2 = std::vector<std::string>());
  bool setCollisionsPython2(bool set_to_allow, const std::string& link_name_1, 
                                        bp::list& link_group_2)
  {
    return setCollisions(set_to_allow, link_name_1, py_bindings_tools::stringFromList(link_group_2));
  }                                        
};

static void wrap_planning_scene_interface()
{
  bp::class_<PlanningSceneInterfaceWrapper> PlanningSceneClass("PlanningSceneInterface",
                                                               bp::init<bp::optional<std::string>>());

  PlanningSceneClass.def("get_known_object_names", &PlanningSceneInterfaceWrapper::getKnownObjectNamesPython);
  PlanningSceneClass.def("get_known_object_names_in_roi",
                         &PlanningSceneInterfaceWrapper::getKnownObjectNamesInROIPython);
  PlanningSceneClass.def("get_object_poses", &PlanningSceneInterfaceWrapper::getObjectPosesPython);
  PlanningSceneClass.def("get_objects", &PlanningSceneInterfaceWrapper::getObjectsPython);
  PlanningSceneClass.def("get_attached_objects", &PlanningSceneInterfaceWrapper::getAttachedObjectsPython);
  PlanningSceneClass.def("apply_planning_scene", &PlanningSceneInterfaceWrapper::applyPlanningScenePython);
  PlanningSceneClass.def("apply_collision_object", &PlanningSceneInterfaceWrapper::applyCollisionObjectPython1);
  PlanningSceneClass.def("apply_collision_object", &PlanningSceneInterfaceWrapper::applyCollisionObjectPython2);
  PlanningSceneClass.def("apply_collision_objects", &PlanningSceneInterfaceWrapper::applyCollisionObjectsPython);
  PlanningSceneClass.def("add_collision_objects", &PlanningSceneInterfaceWrapper::addCollisionObjectsPython);
  PlanningSceneClass.def("remove_collision_object", &PlanningSceneInterfaceWrapper::removeCollisionObjectsPython);
  PlanningSceneClass.def("apply_attached_collision_object", &PlanningSceneInterfaceWrapper::applyAttachedCollisionObjectPython);
  PlanningSceneClass.def("apply_attached_collision_objects", &PlanningSceneInterfaceWrapper::applyAttachedCollisionObjectsPython);
  PlanningSceneClass.def("allow_collisions", &PlanningSceneInterfaceWrapper::allowCollisionsPython1);
  PlanningSceneClass.def("allow_collisions", &PlanningSceneInterfaceWrapper::allowCollisionsPython2);
  PlanningSceneClass.def("disallow_collisions", &PlanningSceneInterfaceWrapper::disallowCollisionsPython1);
  PlanningSceneClass.def("disallow_collisions", &PlanningSceneInterfaceWrapper::disallowCollisionsPython2);
  PlanningSceneClass.def("set_collisions", &PlanningSceneInterfaceWrapper::setCollisionsPython1);
  PlanningSceneClass.def("set_collisions", &PlanningSceneInterfaceWrapper::setCollisionsPython2);
}
}
}

BOOST_PYTHON_MODULE(_moveit_planning_scene_interface)
{
  using namespace moveit::planning_interface;
  wrap_planning_scene_interface();
}

/** @endcond */