/*
Copyright 2016 Luke Fraser

This file is part of UNR_Object_Manipulation.

UNR_Object_Manipulation is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

UNR_Object_Manipulation is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with UNR_Object_Manipulation.  If not, see <http://www.gnu.org/licenses/>. 
*/
#include <boost/python.hpp>
#include "grasp_server_library/grasp_server.h"
#include "python_bindings_tools/roscpp_initializer.h"
#include <Python.h>
#include <string>

// Convert Python Bopost Scope
namespace bp = boost::python;

namespace py_bindings_tools {

template<typename T>
std::vector<T> typeFromList(const boost::python::list &values) {
  int l = boost::python::len(values);
  std::vector<T> v(l);
  for (int i = 0; i < l ; ++i)
    v[i] = boost::python::extract<T>(values[i]);
  return v;
}

template<typename T>
boost::python::list listFromType(const std::vector<T>& v) {
  boost::python::list l;
  for (std::size_t i = 0 ; i < v.size() ; ++i)
    l.append(v[i]);
  return l;
}

std::vector<double> doubleFromList(const boost::python::list &values) {
  return typeFromList<double>(values);
}

std::vector<std::string> stringFromList(const boost::python::list &values) {
  return typeFromList<std::string>(values);
}

boost::python::list listFromDouble(const std::vector<double>& v) {
  return listFromType<double>(v);
}

boost::python::list listFromString(const std::vector<std::string>& v) {
  return listFromType<std::string>(v);
}
}  // namespace py_bindings_tools

namespace grasplib {
class PoseWrapper : public Pose {
 public:
  PoseWrapper() : Pose() {}

  void convertListToPose(const bp::list &l, geometry_msgs::Pose &p) const {
    std::vector<double> v = py_bindings_tools::doubleFromList(l);
    p.position.x =  v[0];
    p.position.y = v[1];
    p.position.z = v[2];
    p.orientation.x = v[3];
    p.orientation.y = v[4];
    p.orientation.z = v[5];
    p.orientation.w = v[6];
  }
  bp::list convertPoseToList(const geometry_msgs::Pose &pose) const {
    std::vector<double> v(7);
    v[0] = pose.position.x;
    v[1] = pose.position.y;
    v[2] = pose.position.z;
    v[3] = pose.orientation.x;
    v[4] = pose.orientation.y;
    v[5] = pose.orientation.z;
    v[6] = pose.orientation.w;
    return py_bindings_tools::listFromDouble(v);
  }
  bp::list ConvertVector4fToList(const Eigen::Vector4f vec) const {
    std::vector<double> v(4);
    v[0] = vec.x();
    v[1] = vec.y();
    v[2] = vec.z();
    v[3] = vec.w();
    return py_bindings_tools::listFromDouble(v);
  }
  bool SetPosePython(bp::list pose) {
    geometry_msgs::Pose msg;
    convertListToPose(pose, msg);
    SetPose(msg);
    return true;
  }

  bp::list GetPosePython() const {
    return convertPoseToList(GetPose());
  }
  bp::list GetPoseNormalPythonPose(bp::list pose) const {
    geometry_msgs::Pose msg;
    convertListToPose(pose, msg);
    return ConvertVector4fToList(GetPoseNormal(msg));
  }
  bp::list GetPoseNormalPython() const {
    return ConvertVector4fToList(GetPoseNormal());
  }
};
////////////////////////////////////////////////////////////////////////////////
// INTERFACE
////////////////////////////////////////////////////////////////////////////////
static bp::class_<PoseWrapper> WrapPoseInterface() {
  bp::class_<PoseWrapper> PoseClass("Pose");

  PoseClass.def("SetPose", &PoseWrapper::SetPosePython);
  PoseClass.def("GetPose", &PoseWrapper::GetPosePython);
  PoseClass.def("GetPoseNormal", &PoseWrapper::GetPoseNormalPythonPose);
  PoseClass.def("GetPoseNormal", &PoseWrapper::GetPoseNormalPython);
}
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Grasp
////////////////////////////////////////////////////////////////////////////////
class GraspWrapper : public Grasp {
 public:
  GraspWrapper() : Grasp() {}
};

////////////////////////////////////////////////////////////////////////////////
// INTERFACE
////////////////////////////////////////////////////////////////////////////////
static void WrapGraspInterface() {
  bp::class_<GraspWrapper> GraspClass("Grasp");
}
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// ObjectPickPlace
////////////////////////////////////////////////////////////////////////////////
class ObjectPickPlaceWrapper : public ObjectPickPlace {
 public:
  ObjectPickPlaceWrapper() : ObjectPickPlace() {}

  bool GenerateGraspFromPosePython() {
    return GenerateGraspFromPose();
  }

  bool GetPickPython() {
    return GetPick();
  }

  bool GetPlacePython() {
    return GetPlace();
  }

  bool SetPickPython() {
    return GetPick();
  }

  bool SetPlacePython() {
    return SetPlace();
  }

  bool SetTypePython() {
    return SetType();
  }

  bool SetNamePython() {
    return SetName();
  }
};
////////////////////////////////////////////////////////////////////////////////
// INTERFACE
////////////////////////////////////////////////////////////////////////////////
static void WrapObjectPickPlaceInterface() {
  bp::class_<ObjectPickPlaceWrapper> ObjectPickPlaceClass("ObjectPickPlace");

  ObjectPickPlaceClass.def("GenerateGraspFromPose",
    &ObjectPickPlaceWrapper::GenerateGraspFromPosePython);

  ObjectPickPlaceClass.def("GetPick",
    &ObjectPickPlaceWrapper::GetPickPython);

  ObjectPickPlaceClass.def("GetPlace",
    &ObjectPickPlaceWrapper::GetPlacePython);

  ObjectPickPlaceClass.def("SetPick",
    &ObjectPickPlaceWrapper::SetPickPython);

  ObjectPickPlaceClass.def("SetPlace",
    &ObjectPickPlaceWrapper::SetPlacePython);

  ObjectPickPlaceClass.def("SetType",
    &ObjectPickPlaceWrapper::SetTypePython);

  ObjectPickPlaceClass.def("SetName",
    &ObjectPickPlaceWrapper::SetNamePython);
}
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Grasp Server Python Wrapper class
////////////////////////////////////////////////////////////////////////////////
class GraspServerWrapper : protected py_bindings_tools::ROScppInitializer,
                           public GraspServer {
 public:
  GraspServerWrapper(std::string arm) :
      GraspServer(arm),
      py_bindings_tools::ROScppInitializer() {}

  bp::str CheckServerStatePython() {
    std::string status = "Hello";
    // CheckServerState(&status);
    return bp::str(status.c_str());
  }

  bool AddObjectPython(bp::str object) {
    return AddObject(bp::extract<std::string>(object));
  }

  bool RemoveObjectPython(bp::str object) {
    return RemoveObject(bp::extract<std::string>(object));
  }

  bool LoadObjectsPythonFile(bp::str filename) {
    return LoadObjects(bp::extract<std::string>(filename));
  }

  static std::vector<ObjectPickPlace> ObjectPickPlaceWraptoBase(
      std::vector<ObjectPickPlaceWrapper> objects) {
    std::vector<ObjectPickPlace> list(objects.size());

    for (int i = 0; i != objects.size(); ++i) {
      list[i] = *reinterpret_cast<ObjectPickPlace*>(&objects[i]);
    }
    return list;
  }

  bool LoadObjectsPython(bp::list objects) {
    std::vector<ObjectPickPlaceWrapper> wrapper_list =
      py_bindings_tools::typeFromList<ObjectPickPlaceWrapper>(objects);

    // Convert to base class type;
    std::vector<ObjectPickPlace> list = ObjectPickPlaceWraptoBase(wrapper_list);
    return LoadObjects(list);
  }

  bool SaveObjectsPythonFile(bp::str filename) {
    return SaveObjects(bp::extract<std::string>(filename));
  }

  bool SaveObjectsPython(bp::str filename, bp::list objects) {
    std::vector<ObjectPickPlaceWrapper> wrapper_list =
      py_bindings_tools::typeFromList<ObjectPickPlaceWrapper>(objects);

    // Convert to base class type;
    std::vector<ObjectPickPlace> list = ObjectPickPlaceWraptoBase(wrapper_list);
    return SaveObjects(bp::extract<std::string>(filename), list);
  }
  bool MergeObjectsPythonFile(bp::str filename) {
    return MergeObjects(bp::extract<std::string>(filename));
  }
  bool MergeObjectsPython(bp::str filename, bp::list objects) {
    std::vector<ObjectPickPlaceWrapper> wrapper_list =
      py_bindings_tools::typeFromList<ObjectPickPlaceWrapper>(objects);

    std::vector<ObjectPickPlace> list = ObjectPickPlaceWraptoBase(wrapper_list);

    return MergeObjects(bp::extract<std::string>(filename), list);
  }

  PoseWrapper GetArmPosePython(bp::str arm) {
    PoseWrapper pose;
    Pose& pose_ref = pose;
    pose_ref = GetArmPose(bp::extract<std::string>(arm));
    return pose;
  }
};

static void WrapGraspServerInterface() {
  bp::class_<GraspServerWrapper> GraspServerClass(
    "GraspServer", bp::init<std::string>());
  GraspServerClass
    .def("CheckServerState", &GraspServerWrapper::CheckServerStatePython)
    .def("AddObject",        &GraspServerWrapper::AddObjectPython)
    .def("RemoveObject",     &GraspServerWrapper::RemoveObjectPython)
    .def("LoadObjects",      &GraspServerWrapper::LoadObjectsPythonFile)
    .def("LoadObjects",      &GraspServerWrapper::LoadObjectsPython)
    .def("SaveObjects",      &GraspServerWrapper::SaveObjectsPythonFile)
    .def("SaveObjects",      &GraspServerWrapper::SaveObjectsPython)
    .def("MergeObjects",     &GraspServerWrapper::MergeObjectsPythonFile)
    .def("MergeObjects",     &GraspServerWrapper::MergeObjectsPython)
    .def("GetArmPose",       &GraspServerWrapper::GetArmPosePython);
}
}  // namespace grasplib

BOOST_PYTHON_MODULE(grasp_server) {
  grasplib::WrapPoseInterface();
  grasplib::WrapGraspInterface();
  grasplib::WrapObjectPickPlaceInterface();
  grasplib::WrapGraspServerInterface();
}
