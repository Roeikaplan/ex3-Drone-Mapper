/**
 * @file PluginLibrary.cpp
 * @brief The only translation unit in the project that calls `dlopen` and `dlclose`.
 */

#include <Simulator/PluginLibrary.h>

#include <dlfcn.h>

#include <utility>

namespace simulator {

PluginLibrary::PluginLibrary(std::filesystem::path file) : file_(std::move(file)) {
    /**
     * @note `dlerror()` is only meaningful immediately after a failed `dl*` call and it latches
     *       until read. Clearing it first stops a previous failure being misreported as ours.
     */
    dlerror();

    handle_ = dlopen(file_.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle_ == nullptr) {
        const char* message = dlerror();
        error_ = message != nullptr ? message : "dlopen failed without a diagnostic";
    }
}

PluginLibrary::~PluginLibrary() {
    close();
}

PluginLibrary::PluginLibrary(PluginLibrary&& other) noexcept
    : file_(std::move(other.file_)), handle_(other.handle_), error_(std::move(other.error_)) {
    /**
     * @note Clearing the source handle is what keeps the close count at one. Without it, growing
     *       the loader's vector would `dlclose` the same library once per reallocation.
     */
    other.handle_ = nullptr;
}

PluginLibrary& PluginLibrary::operator=(PluginLibrary&& other) noexcept {
    if (this != &other) {
        close();
        file_ = std::move(other.file_);
        handle_ = other.handle_;
        error_ = std::move(other.error_);
        other.handle_ = nullptr;
    }
    return *this;
}

void PluginLibrary::close() noexcept {
    if (handle_ != nullptr) {
        /**
         * @note The return value is deliberately ignored. A failing `dlclose` leaves the library
         *       mapped, which is harmless here, and there is nothing useful to do about it during
         *       teardown - whereas throwing or aborting would violate the never-crash rule.
         */
        dlclose(handle_);
        handle_ = nullptr;
    }
}

} // namespace simulator
