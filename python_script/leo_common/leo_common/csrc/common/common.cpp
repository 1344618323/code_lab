#include "common.h"
#include <pybind11/numpy.h>
#include <opencv4/opencv2/opencv.hpp>

namespace py = pybind11;

int plus(int a, int b) {
    return a + b;
}

void create_img() {
    cv::Mat m(100, 100, CV_8U, cv::Scalar(100));
    cv::imshow("img", m);
    cv::waitKey(0);
}

void show_img(py::array_t<uint8_t>& img) {
    auto rows = img.shape(0);
    auto cols = img.shape(1);
    auto type = CV_8UC3;
    cv::Mat m(rows, cols, type, (unsigned char*)img.data());
    cv::imshow("img", m);
    cv::waitKey(0);
}

struct Pet {
    Pet(const std::string& name) : name(name) {}
    void setName(const std::string& name_) { name = name_; }
    const std::string& getName() const { return name; }
    std::string name;
};

PYBIND11_MODULE(common, m) {
    m.doc() = "pybind11 basic operation plugin";  // optional module docstring
    m.def("plus", &plus, "A function that plus two numbers");
    m.def("create_img", &create_img, "A function that create a img");
    m.def("show_img", &show_img, "A function that show a img");
    py::class_<Pet>(m, "Pet")
            .def(py::init<const std::string&>(),
                 py::arg("name") =
                         "cat")  // In pybind 11, writing the default parameters at the function
                                 // declaration is not useful and must be written here
            .def("setName", &Pet::setName)
            .def("getName", &Pet::getName);
}
