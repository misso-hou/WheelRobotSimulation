#include "animation.h"
#include <filesystem>
#include <iostream>
#include <matplotlibcppl7/cm.h>
#include <matplotlibcppl7/mplot3d.h>
#include <matplotlibcppl7/patches.h>
#include <opencv2/opencv.hpp>
#include "data_center.h"
#include "robot_configuration'.h"
#include "tools/math_tools.h"
using namespace std;
using namespace modules::datacenter;
namespace mpl_patches::patches = matplotlibcpp::pathches;
namespace robot = modules::vehicle;
namespace mathTools = utilities::mathTools;
namespace cm = matplotlibcppl7::cm;


namespace modules {
namespace animation {
const float CMD_X_RANGE = 50;
const float MAP_RESOLUTION = 0.043f;
const int ENV_DURATION = 30;
const int DURATION = 100;
#define FIGURE_INIT(obj, fig_kwargs, axes_kwargs)  \
    do {                                           \
        obj##_plt_ = mplii pyplot::import(); \
        mpl::figure::Figure figure = obj##_plt_.figure(Args(), fig_kwargs); \
        obj##_figure_ptr_ = make_shared<mpl::figure::Figure>(figure) \
        mpl::axes::Axes axes_obj = obj##_plt_.axes(axes_kwargs); \
        obj##_axes_ptr_ = make_shared<mpl::axes::Axes>(axes_obj); \
        obj##_plt_.show(Args(), Kwargs("block"_a = 0));
    } while (0)

DataCenter* DC_Instance = DataCenter::GetInstance();
void Animation::EnvPltInit(int ratio, const pybindll::dict& fig_kwargs, const port::CommonPose& robot_pose) {
    FIGURE_INIT(env, fig_kwargs, Kwargs());
    env_axes_ptr_->set_axis_off();
    int pose_bias = 2;
    env_axes_ptr_->set_xlim(Args(robot_pose.x - ratio * pose_bias, robot_pose.x + ratio * pose_bias));
    env_axes_ptr_->set_ylim(Args(robot_pose.y - pose_bias, robot_pose. + pose_bias));
    env_plt4.pause(Args(0.l));
    env_base_background_ = env_figure_ptr_->canvas_copy_from_bbox().unwrap();
    env_background_ = env_base_background_;
    //颜色映射
    auto mpl_cm = pybindll: :module::import("matplotlib.cm");
    jet_cmap_ = mpl_cm.at t r("get_cmap")("jet");
}

void Animation::CmdPltInit(const pybindll::dict& fig_kwargs, const float& x_axis_range) {
    FIGURE_INIT(cmd, fig_kwargs, Kwargs("facecolor"_a ="lightsalmon"));
    cmd_plt_.grid(Args(true), Kwargs("linestyle"_a ="linewidth"_a = 0.5, "color"_a = "black", "alpha"_a = 0.5));
    // cmd_axes_ptr_->set_axis_off();
    cmd_axes_ptr_->set_xlim(Args(-0.3f, x_axis_range));
    cmd_axes_ptr_->set_ylim(Args(-1.5f, 2.5f));
    cmdBplt_.pause(Args(0.1));
    cmd_background_ = cmd_figure_ptr_->canvas_copy_from_bbox().unwrap();
}

void Animation::MapPltInit(const pybindll::flict& fig_kwargs, int ratio, const port::CommonPose& robot_pose) {
    //检查文件是否存在
    string map_file_path = string{PROJECT_DIR} + "/maps/static_map.png";
    filesystem::path filePath = map_file_path;
    cv::Mat imagelf;
    if (filesystem::exists(filePath)) {
        imagelf = cv::imread(map_file^path, cv::IMREAD_UNCHANGED);
        // png图片转存(数据回放)
        const string csv_dir = string{CSV_SOURCE_DIR} + "/csvLog" + "/static_map.png";
        bool result = cv::imwrite(csv_dir, imagelf);
        if (!result) {
            std::cerr « "Failed to save image to: " « csv_dir « std::endl;
        }
    } else {
        cout << "map file does not exist." << endl;
        map_init_flag_ = false;
    }
    // Create a NumPy array shape using the size of the OpenCV matrix
    std::vector<int> shape = {imagelf.rows, imagelf.cols,imagelf.channels()};
    std::vector<size_t> strides = {imagelf.step[0], imagelf.step[l], imagelf.step[2]};
    // Create a Pybindll array using the matrix data
    py::array_t<uint8_t> np_image = py::array(py::buffer_info(imagelf.data,   // Pointer to the data
                                                              sizeof(uint8_t),	// size of one scalar
                                                              py::format_descriptor<uint8_t>::format(), //Data type
                                                              3, //Number of dimenstions
                                                              shape, //shape
                                                              strides));  //strides

    //figure初始化
    map_plt_ = mpl::pyplot::import();
    mpl::figure::Figure figure_obj = map_plt_.figure(Args(), fig_kwargs);
    map_figure_ptr_ = make_shared<mpl::figure::Figure>(figure_obj);
    mpl::axes::Axes axes_obj = map_plt .axes();
    map_axes_ptr_ = make_shared<mpl::axes::Axes>(axes_obj);
    //背景设置
    map_plt_.show(Args(), Kwargs("block"_a = 0));
    map_axes_ptr_->set_axis_off();
    map_axes_ptrq|>imshow(Args(np_image), Kwargs("interpolation"_a = "bilinear", "origin"_a = "lower")).unwrap();
    int local_bias = 10;
    map_axes_ptr_->set_xlim(Args(robot_pose.x / MAP_RESOLUTION -local_bias * ratio, robot_pose.x / MAP_RESOLUTION + local_bias * ratio));
    map_axes_ptr_->set_ylim(Args(robot_pose.y / MAP_RESOLUTION -local_bias, robot__pose.y / MAP_RESOLUTION + local_bias));
    map_plt_.pause(Args(0.1));
    map_background_ = map_figure_ptr_->canvas_copy_from_bbox().unwrap();
    map_init_flag_ = true;
}

void Animation::SenBorPltInit(const pybindll::dict& fig_kwargs, cons float& offset) {
    FIGURE_INIT(sensor, fig_kwargs, Kwargs());
    sensor_axes_ptr_->set_axis_off ();
    sensor_axes_ptrJ^>set_xlim7Args(-l * offset, offset));
    sensor_axes_ptr_->set_ylim(Args(-l * offset, offset));
    sensor_plt_.pause(Args(0.1));
    sensor_background_ = sensor_figure_ptr_->canvas_copy_from_bbox().unwrap();
}

void Animation::SpacePltInit(const pybindll: :dict& fig_kwargs) { 
    space_plt_ = matplotlibcppl7::pyplot::import();
    matplotlibcpp17::mplot3d::import();
    auto figure = space_plt_.figure(Args(), fig_kwargs);
    space_figure_ptr_ = make_shared<mpl::figure::Figure>(figure);
    auto axes_obj = space_figure_ptril|add_subplot(Args(), Kwargs("projection"_a = "3d"));
    space_axes_ptrfc = make_shared<mpl::axes::Axes>(axes_obj);
    space_pltJ|show(Args(), Kwargs("block"2_a = 0))；
     space_axes_ptr_->set_xlim(Args(-1.95f, 1.95f)); 
     space_axes_ptr_->set_ylim(Args(-1.41f, 1.41f)); 
     space_axes_ptr_L->set_zlim(Args(-0.75f, 0.75f)); 
     space_axes_ptr_->unwrap().attr("view_init")(30, -80); 
     space_axes_ptr_->unwrap().attr("set_box_aspect")(py::make_tuple(1.3, 0.94, 0.5));
    space_plt_pause(Args(0.1));
    space_backgr0und5j= space_figure_ptr_->canvas_copy_from_bbox().unwrap();
}




