/**
 * @file h5_writer.cpp
 * @brief Implementation of H5Writer and Group.
 *
 * @note Thread-safety: not thread-safe; synchronize externally if shared across threads.
 * @note Strings are written as UTF-8 via HDF5 string type charset setting.
 * @note Datasets are row-major; size must equal product(dims). Mismatches throw std::invalid_argument.
 * @note RequireGroup expects absolute POSIX-style paths (rooted at '/'); empty segments are skipped.
 * @note Dataset creation uses createDataSet and fails if a dataset already exists; choose unique names or manage overwrites.
 */

#include <seastack/hydro_io/h5_writer.h>

#include <hdf5.h>

#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <utility>

namespace {

// Prefer the C API for open-or-create navigation: some platforms ship HDF5 C++ exceptions that do
// not reliably unwind into our catch handlers when thrown from libhdf5_cpp (release binaries).
H5::Group OpenRootGroupC(const H5::H5File& file) {
    hid_t gid = H5Gopen2(file.getId(), "/", H5P_DEFAULT);
    if (gid < 0) {
        throw std::runtime_error("HDF5: could not open root group '/'");
    }
    return H5::Group(gid);
}

H5::Group OpenOrCreateChildGroupC(hid_t parent_id, const std::string& part) {
    hid_t gid = H5Gopen2(parent_id, part.c_str(), H5P_DEFAULT);
    if (gid < 0) {
        H5Eclear2(H5E_DEFAULT);
        gid = H5Gcreate2(parent_id, part.c_str(), H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
        if (gid < 0) {
            throw std::runtime_error("HDF5: failed to open or create child group '" + part + "'");
        }
    }
    return H5::Group(gid);
}

}  // namespace

namespace seastack::hydro_io {

static H5::H5File OpenFile(const std::string& path, bool overwrite) {
    try {
        if (overwrite) {
            return H5::H5File(path, H5F_ACC_TRUNC);
        }
        return H5::H5File(path, H5F_ACC_EXCL);
    } catch (const H5::FileIException& e) {
        std::ostringstream oss;
        oss << "Failed to open HDF5 file '" << path << "' with mode "
            << (overwrite ? "truncate" : "create-exclusive") << ": ";
        try {
            oss << e.getDetailMsg();
        } catch (...) {
            oss << "(no additional HDF5 details)";
        }
        throw std::runtime_error(oss.str());
    } catch (...) {
        throw std::runtime_error(
            std::string("Failed to open HDF5 file '") + path + "' with mode " +
            (overwrite ? "truncate" : "create-exclusive") +
            " (HDF5 failure; exception type could not be inspected — possible shared-library mismatch).");
    }
}

H5Writer::H5Writer(const std::string& filepath, bool overwrite)
    : file_(OpenFile(filepath, overwrite)), file_path_(filepath) {
    // Suppress global HDF5 error printing; handle failures explicitly via exceptions
    H5::Exception::dontPrint();
}


H5Writer::Group::Group(H5::Group g) : group_(std::move(g)) {}

bool H5Writer::Group::Valid() const noexcept { return group_.getId() >= 0; }

H5Writer::Group H5Writer::Group::CreateGroup(const std::string& name) const {
    if (!Valid()) {
        throw std::logic_error("CreateGroup: group handle is not valid");
    }
    if (name.find('/') != std::string::npos) {
        throw std::invalid_argument("CreateGroup: name must be a single path segment; use RequireGroup for multi-segment paths");
    }
    H5::Group created_group = group_.createGroup(name);
    return Group(created_group);
}

void H5Writer::Group::WriteAttribute(const std::string& name, const std::string& value) const {
    if (!Valid()) {
        throw std::logic_error("WriteAttribute(string): group handle is not valid");
    }
    H5::StrType str_datatype(H5::PredType::C_S1, H5T_VARIABLE);
    str_datatype.setCset(H5T_CSET_UTF8);
    H5::DataSpace data_space(H5S_SCALAR);
    H5::Attribute attribute = group_.createAttribute(name, str_datatype, data_space);
    const char* c_str = value.c_str();
    attribute.write(str_datatype, &c_str);
}

void H5Writer::Group::WriteAttribute(const std::string& name, double value) const {
    if (!Valid()) {
        throw std::logic_error("WriteAttribute(double): group handle is not valid");
    }
    H5::DataSpace data_space(H5S_SCALAR);
    H5::Attribute attribute = group_.createAttribute(name, H5::PredType::NATIVE_DOUBLE, data_space);
    attribute.write(H5::PredType::NATIVE_DOUBLE, &value);
}

void H5Writer::Group::WriteDataset(const std::string& name, const std::string& value) const {
    if (!Valid()) {
        throw std::logic_error("WriteDataset(string): group handle is not valid");
    }
    H5::StrType str_datatype(H5::PredType::C_S1, H5T_VARIABLE);
    str_datatype.setCset(H5T_CSET_UTF8);
    H5::DataSpace data_space(H5S_SCALAR);
    H5::DataSet data_set = group_.createDataSet(name, str_datatype, data_space);
    const char* c_str = value.c_str();
    data_set.write(&c_str, str_datatype);
}

void H5Writer::Group::WriteDataset(const std::string& name, const std::vector<double>& data,
                                   const std::array<hsize_t, 1>& dims) const {
    if (!Valid()) {
        throw std::logic_error("WriteDataset(1D): group handle is not valid");
    }
    if (static_cast<hsize_t>(data.size()) != dims[0]) {
        std::ostringstream oss;
        oss << "WriteDataset(1D, '" << name << "'): size mismatch. data.size()="
            << data.size() << " dims[0]=" << dims[0];
        throw std::invalid_argument(oss.str());
    }
    H5::DataSpace data_space(1, dims.data());
    H5::DataSet data_set = group_.createDataSet(name, H5::PredType::NATIVE_DOUBLE, data_space);
    data_set.write(data.data(), H5::PredType::NATIVE_DOUBLE);
}

void H5Writer::Group::WriteDataset(const std::string& name, const std::vector<double>& data,
                                   const std::array<hsize_t, 2>& dims) const {
    if (!Valid()) {
        throw std::logic_error("WriteDataset(2D): group handle is not valid");
    }
    if (static_cast<hsize_t>(data.size()) != dims[0] * dims[1]) {
        std::ostringstream oss;
        oss << "WriteDataset(2D, '" << name << "'): size mismatch. data.size()="
            << data.size() << " dims product=" << (dims[0] * dims[1]);
        throw std::invalid_argument(oss.str());
    }
    H5::DataSpace data_space(2, dims.data());
    H5::DataSet data_set = group_.createDataSet(name, H5::PredType::NATIVE_DOUBLE, data_space);
    data_set.write(data.data(), H5::PredType::NATIVE_DOUBLE);
}

// ── Helpers for chunked / compressed dataset creation ────────────────────

namespace {

H5::DSetCreatPropList MakePropList(int rank, const hsize_t* dims,
                                   int compression_level) {
    H5::DSetCreatPropList plist;
    if (compression_level > 0) {
        // Chunk shape: min(N, 1000) for first dim; keep remaining dims intact.
        // For 1D: min(N, 4000); for 2D: {min(N, 1000), cols}.
        std::vector<hsize_t> chunk(rank);
        hsize_t row_limit = (rank == 1) ? 4000 : 1000;
        chunk[0] = std::min(dims[0], row_limit);
        for (int i = 1; i < rank; ++i) chunk[i] = dims[i];
        plist.setChunk(rank, chunk.data());
        plist.setDeflate(compression_level);
    }
    return plist;
}

}  // namespace

void H5Writer::Group::WriteDataset(const std::string& name,
                                   const std::vector<double>& data,
                                   const std::array<hsize_t, 1>& dims,
                                   const WriteOptions& opts) const {
    if (!Valid()) {
        throw std::logic_error("WriteDataset(1D,opts): group handle is not valid");
    }
    if (static_cast<hsize_t>(data.size()) != dims[0]) {
        std::ostringstream oss;
        oss << "WriteDataset(1D, '" << name << "'): size mismatch. data.size()="
            << data.size() << " dims[0]=" << dims[0];
        throw std::invalid_argument(oss.str());
    }
    H5::DataSpace data_space(1, dims.data());
    auto plist = MakePropList(1, dims.data(), opts.compression_level);
    H5::DataSet ds = group_.createDataSet(name, H5::PredType::NATIVE_DOUBLE,
                                          data_space, plist);
    ds.write(data.data(), H5::PredType::NATIVE_DOUBLE);
}

void H5Writer::Group::WriteDataset(const std::string& name,
                                   const std::vector<double>& data,
                                   const std::array<hsize_t, 2>& dims,
                                   const WriteOptions& opts) const {
    if (!Valid()) {
        throw std::logic_error("WriteDataset(2D,opts): group handle is not valid");
    }
    if (static_cast<hsize_t>(data.size()) != dims[0] * dims[1]) {
        std::ostringstream oss;
        oss << "WriteDataset(2D, '" << name << "'): size mismatch. data.size()="
            << data.size() << " dims product=" << (dims[0] * dims[1]);
        throw std::invalid_argument(oss.str());
    }
    H5::DataSpace data_space(2, dims.data());
    auto plist = MakePropList(2, dims.data(), opts.compression_level);
    H5::DataSet ds = group_.createDataSet(name, H5::PredType::NATIVE_DOUBLE,
                                          data_space, plist);
    ds.write(data.data(), H5::PredType::NATIVE_DOUBLE);
}

void H5Writer::Group::WriteDatasetF32(const std::string& name,
                                      const std::vector<double>& data,
                                      const std::array<hsize_t, 1>& dims,
                                      const WriteOptions& opts) const {
    if (!Valid()) {
        throw std::logic_error("WriteDatasetF32(1D): group handle is not valid");
    }
    if (static_cast<hsize_t>(data.size()) != dims[0]) {
        std::ostringstream oss;
        oss << "WriteDatasetF32(1D, '" << name << "'): size mismatch. data.size()="
            << data.size() << " dims[0]=" << dims[0];
        throw std::invalid_argument(oss.str());
    }
    std::vector<float> f32(data.size());
    for (size_t i = 0; i < data.size(); ++i) f32[i] = static_cast<float>(data[i]);
    H5::DataSpace data_space(1, dims.data());
    auto plist = MakePropList(1, dims.data(), opts.compression_level);
    H5::DataSet ds = group_.createDataSet(name, H5::PredType::NATIVE_FLOAT,
                                          data_space, plist);
    ds.write(f32.data(), H5::PredType::NATIVE_FLOAT);
}

void H5Writer::Group::WriteDatasetF32(const std::string& name,
                                      const std::vector<double>& data,
                                      const std::array<hsize_t, 2>& dims,
                                      const WriteOptions& opts) const {
    if (!Valid()) {
        throw std::logic_error("WriteDatasetF32(2D): group handle is not valid");
    }
    if (static_cast<hsize_t>(data.size()) != dims[0] * dims[1]) {
        std::ostringstream oss;
        oss << "WriteDatasetF32(2D, '" << name << "'): size mismatch. data.size()="
            << data.size() << " dims product=" << (dims[0] * dims[1]);
        throw std::invalid_argument(oss.str());
    }
    std::vector<float> f32(data.size());
    for (size_t i = 0; i < data.size(); ++i) f32[i] = static_cast<float>(data[i]);
    H5::DataSpace data_space(2, dims.data());
    auto plist = MakePropList(2, dims.data(), opts.compression_level);
    H5::DataSet ds = group_.createDataSet(name, H5::PredType::NATIVE_FLOAT,
                                          data_space, plist);
    ds.write(f32.data(), H5::PredType::NATIVE_FLOAT);
}

void H5Writer::Group::WriteStringArray(const std::string& name, const std::vector<std::string>& values) const {
    if (!Valid()) {
        throw std::logic_error("WriteStringArray: group handle is not valid");
    }
    std::vector<const char*> c_strs(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
        c_strs[i] = values[i].c_str();
    }
    H5::StrType str_datatype(H5::PredType::C_S1, H5T_VARIABLE);
    str_datatype.setCset(H5T_CSET_UTF8);
    hsize_t dims_1d[1] = { static_cast<hsize_t>(values.size()) };
    H5::DataSpace data_space(1, dims_1d);
    H5::DataSet data_set = group_.createDataSet(name, str_datatype, data_space);
    data_set.write(c_strs.data(), str_datatype);
}

H5Writer::Group H5Writer::Root() const { return Group(OpenRootGroupC(file_)); }

static std::vector<std::string> Split(const std::string& s, char delim) {
    std::vector<std::string> parts;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        if (!item.empty()) parts.push_back(item);
    }
    return parts;
}

H5Writer::Group H5Writer::RequireGroup(const std::string& path) const {
    if (path.empty() || path.front() != '/') {
        throw std::invalid_argument("RequireGroup: path must be absolute and start with '/': '" + path + "'");
    }
    H5::Group current_group = OpenRootGroupC(file_);
    // Empty segments are skipped. One open/create per path segment (C API — see OpenOrCreateChildGroupC).
    for (const auto& part : Split(path, '/')) {
        H5::Group next = OpenOrCreateChildGroupC(current_group.getId(), part);
        current_group = std::move(next);
    }
    return Group(std::move(current_group));
}

} // namespace seastack::hydro_io