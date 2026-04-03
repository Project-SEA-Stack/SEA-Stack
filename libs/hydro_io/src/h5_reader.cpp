/*********************************************************************
 * @file  h5fileinfo.cpp
 *
 * @brief implementation file of HydroData main class and helper class \
 * H5FileInfo.
 *********************************************************************/
#include <H5Cpp.h>
#include <hdf5.h>

#include <seastack/hydro_io/h5_reader.h>
#include <filesystem>  // std::filesystem::absolute
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>
#include <seastack/infra/logging.h>
#include <seastack/core/math_constants.h>  // M_PI (portable)

namespace seastack::hydro_io {

namespace {

void TrimTrailingNullsAndAsciiSpace(std::string* s) {
    while (!s->empty()) {
        const unsigned char c = static_cast<unsigned char>(s->back());
        if (c == '\0' || c == ' ' || c == '\t') {
            s->pop_back();
        } else {
            break;
        }
    }
    while (!s->empty()) {
        const unsigned char c = static_cast<unsigned char>(s->front());
        if (c == ' ' || c == '\t') {
            s->erase(s->begin());
        } else {
            break;
        }
    }
}

bool EqualsIgnoreCaseAscii(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::tolower(static_cast<unsigned char>(a[i])) !=
            std::tolower(static_cast<unsigned char>(b[i]))) {
            return false;
        }
    }
    return true;
}

void ValidateSimulationParameters(const seastack::hydro::HydroData& data, const std::string& h5_path) {
    const auto& sim = data.GetSimulationInfo();
    if (!(sim.rho > 0.0) || std::isnan(sim.rho) || std::isinf(sim.rho)) {
        throw std::runtime_error(
            "HDF5 simulation_parameters: rho is missing or invalid (must be finite and > 0). "
            "Check dataset type and path in '" +
            h5_path + "'.");
    }
    if (!(sim.g > 0.0) || std::isnan(sim.g) || std::isinf(sim.g)) {
        throw std::runtime_error(
            "HDF5 simulation_parameters: g is missing or invalid (must be finite and > 0). "
            "Check dataset type and path in '" +
            h5_path + "'.");
    }
    const double wd = sim.water_depth;
    if (std::isnan(wd) || wd < 0.0) {
        throw std::runtime_error(
            "HDF5 simulation_parameters: water_depth is invalid (NaN or negative). "
            "Check dataset in '" +
            h5_path + "'.");
    }
}

/// Read numeric (float or integer) dataset elements into a double buffer using HDF5 type conversion.
void ReadNumericDatasetToDoubleBuffer(H5::DataSet& dataset,
                                      const std::string& data_name,
                                      int rank,
                                      const hsize_t* dims,
                                      hsize_t n_elem,
                                      std::vector<double>& buf) {
    H5::DataType datatype   = dataset.getDataType();
    const H5T_class_t tclass = H5Tget_class(datatype.getId());
    if (tclass != H5T_FLOAT && tclass != H5T_INTEGER) {
        throw std::runtime_error(
            "HDF5 dataset '" + data_name +
            "': expected floating-point or integer elements (H5T_FLOAT / H5T_INTEGER), got another type.");
    }
    H5::DataSpace filespace = dataset.getSpace();
    H5::DataSpace mspace(rank, dims);
    buf.resize(n_elem);
    dataset.read(buf.data(), H5::PredType::NATIVE_DOUBLE, mspace, filespace);
}

void ParseStringScalarToDouble(const std::string& raw, double& var, const std::string& data_name) {
    std::string str = raw;
    TrimTrailingNullsAndAsciiSpace(&str);
    if (EqualsIgnoreCaseAscii(str, "infinite")) {
        var = std::numeric_limits<double>::infinity();
        return;
    }
    try {
        size_t idx = 0;
        const double v = std::stod(str, &idx);
        while (idx < str.size() &&
               std::isspace(static_cast<unsigned char>(str[idx]))) {
            ++idx;
        }
        if (idx != str.size()) {
            LOG_WARNING("InitScalar: trailing junk after numeric value in dataset " << data_name);
            return;
        }
        var = v;
    } catch (const std::exception&) {
        LOG_WARNING("InitScalar: could not parse string value '" << str << "' in dataset " << data_name);
    }
}

}  // namespace

H5FileInfo::H5FileInfo(std::string file, int num_bod) {
    h5_file_name_ = file;
    num_bodies_   = num_bod;
    seastack::infra::debug::LogDebug(std::string("Searching for file: ") + file);
    if (std::filesystem::exists(file)) {
        seastack::infra::debug::LogDebug(std::string("Found file at: ") + std::filesystem::absolute(file).string());
    } else {
        seastack::infra::cli::LogWarning(std::string("H5 file does not exist, absolute file location: ") + std::filesystem::absolute(file).string());
    }
}

seastack::hydro::HydroData H5FileInfo::ReadH5Data() {
    try {
        H5::H5File userH5File(h5_file_name_, H5F_ACC_RDONLY);
        seastack::hydro::HydroData data_to_init;
        data_to_init.resize(num_bodies_);

    // simparams first
    InitScalar(userH5File, "simulation_parameters/rho", data_to_init.sim_data_.rho);
    InitScalar(userH5File, "simulation_parameters/g", data_to_init.sim_data_.g);
    InitScalar(userH5File, "simulation_parameters/water_depth", data_to_init.sim_data_.water_depth);
    double rho = data_to_init.sim_data_.rho;
    double g   = data_to_init.sim_data_.g;

    // Wave heading angles (optional -- not all H5 files have multi-heading data).
    // BEMIO convention stores directions in degrees; convert to radians for
    // internal use (wave components use radians throughout).
    // Use H5Lexists before openDataSet so missing wave_dir does not trigger HDF5-DIAG on stderr.
    constexpr char kWaveDirDataset[] = "simulation_parameters/wave_dir";
    const htri_t wave_dir_link       = H5Lexists(userH5File.getId(), kWaveDirDataset, H5P_DEFAULT);
    if (wave_dir_link > 0) {
        try {
            Init1D(userH5File, kWaveDirDataset, data_to_init.sim_data_.wave_directions);
            data_to_init.sim_data_.wave_directions *= M_PI / 180.0;
        } catch (...) {
            // Wrong type, read failure, or H5::* not derived from std::exception across libs.
            data_to_init.sim_data_.wave_directions.resize(0);
        }
    } else {
        if (wave_dir_link < 0) {
            LOG_WARNING("HDF5 H5Lexists failed for " << kWaveDirDataset << " in '" << h5_file_name_
                                                     << "'; treating wave_dir as absent.");
            H5Eclear2(H5E_DEFAULT);
        }
        data_to_init.sim_data_.wave_directions.resize(0);
    }

    ValidateSimulationParameters(data_to_init, h5_file_name_);

    // for each body things
    for (int i = 0; i < num_bodies_; i++) {
        data_to_init.body_data_[i].body_name = "body" + std::to_string(i + 1);
        std::string bodyName                 = data_to_init.body_data_[i].body_name;
        data_to_init.body_data_[i].body_num  = i;

        InitScalar(userH5File, bodyName + "/properties/disp_vol", data_to_init.body_data_[i].disp_vol);
        Init1D(userH5File, bodyName + "/hydro_coeffs/radiation_damping/impulse_response_fun/t",
               data_to_init.body_data_[i].rirf_time_vector);
        data_to_init.body_data_[i].rirf_timestep =
            data_to_init.body_data_[i].rirf_time_vector[1] - data_to_init.body_data_[i].rirf_time_vector[0];

        Init1D(userH5File, bodyName + "/properties/cg", data_to_init.body_data_[i].cg);
        Init1D(userH5File, bodyName + "/properties/cb", data_to_init.body_data_[i].cb);
        Init2D(userH5File, bodyName + "/hydro_coeffs/linear_restoring_stiffness",
               data_to_init.body_data_[i].lin_matrix);
        Init2D(userH5File, bodyName + "/hydro_coeffs/added_mass/inf_freq", data_to_init.body_data_[i].inf_added_mass);
        data_to_init.body_data_[i].inf_added_mass *= rho;
        Init3D(userH5File, bodyName + "/hydro_coeffs/radiation_damping/impulse_response_fun/K",
               data_to_init.body_data_[i].rirf_matrix);

        Init1D(userH5File, "simulation_parameters/w", data_to_init.reg_wave_data_[i].freq_list);
        Init3D(userH5File, bodyName + "/hydro_coeffs/excitation/mag",
               data_to_init.reg_wave_data_[i].excitation_mag_matrix);
        data_to_init.reg_wave_data_[i].excitation_mag_matrix =
            data_to_init.reg_wave_data_[i].excitation_mag_matrix *
            data_to_init.reg_wave_data_[i].excitation_mag_matrix.constant(rho * g);
        Init3D(userH5File, bodyName + "/hydro_coeffs/excitation/phase",
               data_to_init.reg_wave_data_[i].excitation_phase_matrix);

        Init1D(userH5File, bodyName + "/hydro_coeffs/excitation/impulse_response_fun/t",
               data_to_init.irreg_wave_data_[i].excitation_irf_time);
        Eigen::Tensor<double, 3> temp;
        Init3D(userH5File, bodyName + "/hydro_coeffs/excitation/impulse_response_fun/f", temp);
        data_to_init.irreg_wave_data_[i].excitation_irf_matrix = SqueezeMid(temp);
        data_to_init.irreg_wave_data_[i].excitation_irf_matrix *= rho * g;
    }

        userH5File.close();

        return data_to_init;
    } catch (const H5::Exception& e) {
        std::ostringstream oss;
        oss << "Unable to open/read HDF5 hydro data file: " << h5_file_name_ << "\n";
        oss << "HDF5 error: " << e.getDetailMsg() << "\n";
#ifdef _WIN32
        oss << "This often indicates the file is locked by another application (e.g., HDFView) on Windows.\n";
        oss << "Close any viewers or set HDF5_USE_FILE_LOCKING to FALSE (or BEST_EFFORT) and retry.\n";
        oss << "PowerShell example: $env:HDF5_USE_FILE_LOCKING = \"FALSE\"\n";
#else
        oss << "Ensure the file is not in use by another process and is readable.\n";
#endif
        throw std::runtime_error(oss.str());
    } catch (...) {
        throw std::runtime_error(
            "Unable to open/read HDF5 hydro data file: " + h5_file_name_ +
            "\nUnexpected exception during HDF5 I/O (not H5::Exception; possible library mismatch).\n");
    }
}

// squeezes the middle dimension of 1 out
Eigen::MatrixXd H5FileInfo::SqueezeMid(Eigen::Tensor<double, 3>& to_be_squeezed) {
    if (to_be_squeezed.dimension(1) == 0) {
        throw std::runtime_error(
            "SqueezeMid: middle dimension of tensor is zero; expected >= 1");
    }
    int dof  = to_be_squeezed.dimension(0);
    int size = to_be_squeezed.dimension(2);
    Eigen::MatrixXd squeezed(dof, size);
    for (int i = 0; i < dof; i++) {
        for (int j = 0; j < size; j++) {
            // squeeze 6x1x1000 or whatever into 6x1000 matrix
            squeezed(i, j) = to_be_squeezed(i, 0, j);
        }
    }
    return squeezed;
}

void H5FileInfo::InitScalar(H5::H5File& file, std::string data_name, double& var) {
    H5::DataSet dataset   = file.openDataSet(data_name);
    H5::DataType datatype = dataset.getDataType();
    const hid_t tid       = datatype.getId();
    const H5T_class_t tclass = H5Tget_class(tid);

    if (tclass == H5T_FLOAT) {
        H5::DataSpace filespace = dataset.getSpace();
        hsize_t dims[2]         = {0, 0};
        int rank                = filespace.getSimpleExtentDims(dims);
        H5::DataSpace mspace1   = H5::DataSpace(rank, dims);
        dataset.read(&var, H5::PredType::NATIVE_DOUBLE, mspace1, filespace);
    } else if (tclass == H5T_INTEGER) {
        H5::DataSpace filespace = dataset.getSpace();
        hsize_t dims[2]         = {0, 0};
        int rank                = filespace.getSimpleExtentDims(dims);
        hsize_t n_elem          = 1;
        for (int d = 0; d < rank; ++d) {
            n_elem *= dims[d];
        }
        if (n_elem != 1) {
            LOG_WARNING("InitScalar: integer dataset " << data_name << " has " << n_elem
                                                       << " elements; expected scalar (1)");
        } else {
            H5::DataSpace mspace1 = H5::DataSpace(rank, dims);
            dataset.read(&var, H5::PredType::NATIVE_DOUBLE, mspace1, filespace);
        }
    } else if (tclass == H5T_STRING) {
        H5::DataSpace filespace = dataset.getSpace();
        if (H5Tis_variable_str(tid) > 0) {
            const hsize_t nbytes = dataset.getStorageSize();
            if (nbytes == 0) {
                LOG_WARNING("InitScalar: empty variable-length string in dataset " << data_name);
            } else {
                std::vector<char> buf(nbytes + 1, '\0');
                dataset.read(buf.data(), datatype, filespace, filespace);
                ParseStringScalarToDouble(std::string(buf.data()), var, data_name);
            }
        } else {
            const size_t str_size = H5Tget_size(tid);
            std::vector<char> buf(str_size > 0 ? str_size : 1, '\0');
            dataset.read(buf.data(), datatype, filespace, filespace);
            ParseStringScalarToDouble(std::string(buf.data(), str_size), var, data_name);
        }
    } else {
        LOG_WARNING("InitScalar: unexpected HDF5 type class in dataset " << data_name);
    }

    dataset.close();
}

void H5FileInfo::Init1D(H5::H5File& file, std::string data_name, Eigen::VectorXd& var) {
    H5::DataSet dataset     = file.openDataSet(data_name);
    H5::DataSpace filespace = dataset.getSpace();
    hsize_t dims[2]         = {0, 0};
    int rank                = filespace.getSimpleExtentDims(dims);
    hsize_t n_elem          = dims[0];
    for (int d = 1; d < rank; ++d) {
        n_elem *= dims[d];
    }
    std::vector<double> buf;
    ReadNumericDatasetToDoubleBuffer(dataset, data_name, rank, dims, n_elem, buf);
    var.resize(static_cast<Eigen::Index>(n_elem));
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(n_elem); i++) {
        var[i] = buf[static_cast<size_t>(i)];
    }
    dataset.close();
}

void H5FileInfo::Init2D(H5::H5File& file, std::string data_name, Eigen::MatrixXd& var) {
    H5::DataSet dataset     = file.openDataSet(data_name);
    H5::DataSpace filespace = dataset.getSpace();
    hsize_t dims[2]         = {1, 1};
    int rank                = filespace.getSimpleExtentDims(dims);
    for (int d = rank; d < 2; ++d) {
        dims[d] = 1;
    }
    hsize_t n_elem = dims[0] * dims[1];
    std::vector<double> buf;
    ReadNumericDatasetToDoubleBuffer(dataset, data_name, rank, dims, n_elem, buf);
    var.resize(dims[0], dims[1]);
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(dims[0]); i++) {
        for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(dims[1]); j++) {
            var(i, j) = buf[static_cast<hsize_t>(i) * dims[1] + static_cast<hsize_t>(j)];
        }
    }
    dataset.close();
}

void H5FileInfo::Init3D(H5::H5File& file, std::string data_name, Eigen::Tensor<double, 3>& var) {
    H5::DataSet dataset     = file.openDataSet(data_name);
    H5::DataSpace filespace = dataset.getSpace();
    hsize_t dims[3]         = {1, 1, 1};
    int rank                = filespace.getSimpleExtentDims(dims);
    for (int d = rank; d < 3; ++d) {
        dims[d] = 1;
    }
    hsize_t n_elem = dims[0] * dims[1] * dims[2];
    std::vector<double> buf;
    ReadNumericDatasetToDoubleBuffer(dataset, data_name, rank, dims, n_elem, buf);
    var.resize(static_cast<Eigen::Index>(dims[0]), static_cast<Eigen::Index>(dims[1]), static_cast<Eigen::Index>(dims[2]));
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(dims[0]); i++) {
        for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(dims[1]); j++) {
            for (Eigen::Index k = 0; k < static_cast<Eigen::Index>(dims[2]); k++) {
                hsize_t index = static_cast<hsize_t>(k) + dims[2] * (static_cast<hsize_t>(j) + static_cast<hsize_t>(i) * dims[1]);
                var(i, j, k)  = buf[index];
            }
        }
    }
    dataset.close();
}

H5FileInfo::~H5FileInfo() {}

}  // namespace seastack::hydro_io
