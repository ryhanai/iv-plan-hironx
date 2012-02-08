#define BOOST_PYTHON_STATIC_LIB
#include <string>
#include <iostream>
#include <vector>
#include <new>
#include <algorithm>
#include <boost/python.hpp>

#include <boost/foreach.hpp>
#include <boost/range/value_type.hpp>

#include "PQP.h"

#define PQP_SEQSIZE_ERROR (-6) // A value smaller than any other error codes

typedef std::vector<PQP_REAL> real_vector;
enum contact_type { all = PQP_ALL_CONTACTS, first = PQP_FIRST_CONTACT };


class PQP_Model2 : public PQP_Model {
public:
  int AddTri(const real_vector p1, const real_vector p2, const real_vector p3, int id) {
    if (p1.size() == 3 && p2.size() == 3 && p3.size() == 3) {
      PQP_REAL q1[3];
      q1[0] = p1[0]; q1[1] = p1[1]; q1[2] = p1[2];
      PQP_REAL q2[3];
      q2[0] = p2[0]; q2[1] = p2[1]; q2[2] = p2[2];
      PQP_REAL q3[3];
      q3[0] = p3[0]; q3[1] = p3[1]; q3[2] = p3[2];
      PQP_Model::AddTri(q1, q2, q3, id);
    } else {
      std::cerr << "the number of floats is not three" << std::endl;
      return PQP_SEQSIZE_ERROR; 
    }
    
    return PQP_OK;
  }

  real_vector const& get_list() const {
    return v_;
  }

  void add(real_vector const& that) {
    std::copy(that.begin(), that.end(), std::back_inserter(v_));
  }

private:
  real_vector v_;

};

template<typename T_>
class vector_to_pylist_converter {
public:
  typedef T_ native_type;

  static PyObject* convert(native_type const& v) {
    namespace py = boost::python;
    py::list retval;
    BOOST_FOREACH(typename boost::range_value<native_type>::type i, v)
      {
	retval.append(py::object(i));
      }
    return py::incref(retval.ptr());
  }
};

template<typename T_>
class pylist_to_vector_converter {
public:
  typedef T_ native_type;

  static void* convertible(PyObject* pyo) {
    if (!PySequence_Check(pyo))
      return 0;

    return pyo;
  }

  static void construct(PyObject* pyo, boost::python::converter::rvalue_from_python_stage1_data* data) {
    namespace py = boost::python;
    native_type* storage = 
      new(reinterpret_cast<py::converter::rvalue_from_python_storage<native_type>*>(data)->storage.bytes) native_type();
    for (py::ssize_t i = 0, l = PySequence_Size(pyo); i < l; ++i) {
      storage->push_back(
			 py::extract<typename boost::range_value<native_type>::type>(
										     PySequence_GetItem(pyo, i)));
    }
    data->convertible = storage;
  }
};

int PQP_Collide2(PQP_CollideResult *result,
		 real_vector R1, real_vector T1, PQP_Model2 *o1,
		 real_vector R2, real_vector T2, PQP_Model2 *o2, enum contact_type ctype) {
  if (R1.size() != 9) return PQP_SEQSIZE_ERROR;
  if (T1.size() != 3) return PQP_SEQSIZE_ERROR;
  if (R2.size() != 9) return PQP_SEQSIZE_ERROR;
  if (T2.size() != 3) return PQP_SEQSIZE_ERROR;

  PQP_REAL R1_[3][3];
  PQP_REAL T1_[3];
  PQP_REAL R2_[3][3];
  PQP_REAL T2_[3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R1_[i][j] = R1[i*3+j];
    }
  }
  for (int i = 0; i < 3; i++) { T1_[i] = T1[i]; }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R2_[i][j] = R2[i*3+j];
    }
  }
  for (int i = 0; i < 3; i++) { T2_[i] = T2[i]; }

  return PQP_Collide(result, R1_, T1_, o1, R2_, T2_, o2, ctype);
}

int PQP_Distance2(PQP_DistanceResult *result,
		  real_vector R1, real_vector T1, PQP_Model2 *o1,
		  real_vector R2, real_vector T2, PQP_Model2 *o2,
		  PQP_REAL rel_err, PQP_REAL abs_err) {
  if (R1.size() != 9) return PQP_SEQSIZE_ERROR;
  if (T1.size() != 3) return PQP_SEQSIZE_ERROR;
  if (R2.size() != 9) return PQP_SEQSIZE_ERROR;
  if (T2.size() != 3) return PQP_SEQSIZE_ERROR;

  PQP_REAL R1_[3][3];
  PQP_REAL T1_[3];
  PQP_REAL R2_[3][3];
  PQP_REAL T2_[3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R1_[i][j] = R1[i*3+j];
    }
  }
  for (int i = 0; i < 3; i++) { T1_[i] = T1[i]; }
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R2_[i][j] = R2[i*3+j];
    }
  }
  for (int i = 0; i < 3; i++) { T2_[i] = T2[i]; }

  return PQP_Distance(result, R1_, T1_, o1, R2_, T2_, o2, rel_err, abs_err);
}


BOOST_PYTHON_MODULE( libcpqp )
{
  using namespace boost::python;

  to_python_converter<real_vector, vector_to_pylist_converter<real_vector> >();
  converter::registry::push_back(
				 &pylist_to_vector_converter<real_vector>::convertible,
				 &pylist_to_vector_converter<real_vector>::construct,
				 boost::python::type_id<real_vector>());

  class_<PQP_Model2>("PQP_Model")
    .def("BeginModel", &PQP_Model2::BeginModel)
    .def("AddTri", &PQP_Model2::AddTri)
    .def("EndModel", &PQP_Model2::EndModel)
    .def("MemUsage", &PQP_Model2::MemUsage)
    .def("get_list", &PQP_Model2::get_list, return_value_policy<copy_const_reference>())
    .def("add", (void(PQP_Model2::*)(real_vector const&))&PQP_Model2::add)
    ;

  class_<PQP_CollideResult>("PQP_CollideResult")
    .def("NumBVTests", &PQP_CollideResult::NumBVTests)
    .def("NumTriTests", &PQP_CollideResult::NumTriTests)
    .def("NumPairs", &PQP_CollideResult::NumPairs)
    .def("Id1", &PQP_CollideResult::Id1)
    .def("Id2", &PQP_CollideResult::Id2)
    ;

  class_<PQP_DistanceResult>("PQP_DistanceResult")
    .def("NumBVTests", &PQP_DistanceResult::NumBVTests)
    .def("NumTriTests", &PQP_DistanceResult::NumTriTests)
    .def("Distance", &PQP_DistanceResult::Distance)
    ;

  def("PQP_Collide", &PQP_Collide2);
  def("PQP_Distance", &PQP_Distance2);

  enum_<contact_type>("contact_type")
    .value("all", all)
    .value("first", first)
    ;
}
