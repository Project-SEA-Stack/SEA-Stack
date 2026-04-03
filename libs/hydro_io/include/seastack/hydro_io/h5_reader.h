/*********************************************************************
 * @file  h5_reader.h
 * @brief HDF5 reader for BEMIO-formatted hydrodynamic data files.
 *
 * H5FileInfo reads an HDF5 file and produces a HydroData object.
 * HydroData itself is defined in seastack/hydro/hydro_data.h
 * (part of SEAStack::Hydro, no HDF5 dependency).
 *********************************************************************/
#ifndef SEASTACK_HYDRO_IO_H5_READER_H
#define SEASTACK_HYDRO_IO_H5_READER_H

#include <string>
#include <seastack/hydro/hydro_data.h>

namespace H5 {
class H5File;
}

#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

namespace seastack::hydro_io {

class H5FileInfo {
  public:
    H5FileInfo(std::string file, int num_bod = 1);
    H5FileInfo() = delete;

    H5FileInfo(const H5FileInfo& old) = default;
    H5FileInfo& operator=(const H5FileInfo& rhs) = default;

    H5FileInfo(H5FileInfo&&) = default;
    H5FileInfo& operator=(H5FileInfo&& rhs) = default;

    ~H5FileInfo();

    seastack::hydro::HydroData ReadH5Data();

  private:
    std::string h5_file_name_;
    int num_bodies_;

    void InitScalar(H5::H5File& file, std::string data_name, double& var);
    void Init1D(H5::H5File& file, std::string data_name, Eigen::VectorXd& var);
    void Init2D(H5::H5File& file, std::string data_name, Eigen::MatrixXd& var);
    void Init3D(H5::H5File& file, std::string data_name, Eigen::Tensor<double, 3>& var);
    Eigen::MatrixXd SqueezeMid(Eigen::Tensor<double, 3>& to_be_squeezed);
};

}  // namespace seastack::hydro_io

#endif  // SEASTACK_HYDRO_IO_H5_READER_H
