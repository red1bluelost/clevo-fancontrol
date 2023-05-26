//===- clevo_fan_control.cpp - Reimplement of fan control -------*- C++ -*-===//
//
// TODO: License
//
//===----------------------------------------------------------------------===//
///
/// \file
/// Reimplements the fan control in c++.
///
//===----------------------------------------------------------------------===//

#include <fmt/chrono.h>
#include <fmt/core.h>
#include <sys/io.h>
#include <sys/syscall.h>

#include <fcntl.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <span>
#include <string_view>
#include <thread>
#include <vector>

namespace {
namespace k {
constexpr uint16_t ec_data        = 0x62;
constexpr uint16_t ec_sc          = 0x66;
constexpr uint16_t ec_sc_read_cmd = 0x80;

constexpr size_t ec_reg_size        = 0x100;
constexpr size_t ec_reg_cpu_temp    = 0x07;
constexpr size_t ec_reg_gpu_temp    = 0xCD;
constexpr size_t ec_reg_fan_duty    = 0xCE;
constexpr size_t ec_reg_fan_rpms_hi = 0xD0;
constexpr size_t ec_reg_fan_rpms_lo = 0xD1;

constexpr uint8_t ibf = 0x01;
constexpr uint8_t obf = 0x00;
} // namespace k

volatile bool global_running_flag = true;

std::errc ec_init() {
  if (auto err = ioperm(k::ec_data, 1, 1)) return std::errc(err);
  if (auto err = ioperm(k::ec_sc, 1, 1)) return std::errc(err);
  return std::errc(EXIT_SUCCESS);
}

std::errc
ec_io_wait(const uint16_t port, const uint8_t flag, const uint8_t value) {
  int repeat_count = 0;
  while (true) {
    uint8_t data = inb(port);
    if (((data >> flag) & 0x1) == value) break;
    repeat_count += 1;
    if (repeat_count >= 100) {
      fmt::print(
          "wait_ec error on port {:#02x}, data={:#02x}, flag={:#02x}, "
          "value={:#02x}\n",
          port,
          data,
          flag,
          value
      );
      return std::errc::timed_out;
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  return std::errc(EXIT_SUCCESS);
}

std::errc ec_io_do(const uint8_t cmd, const uint8_t port, const uint8_t value) {
  if (auto err = ec_io_wait(k::ec_sc, k::ibf, 0); int(err)) return err;
  outb(cmd, k::ec_sc);

  if (auto err = ec_io_wait(k::ec_sc, k::ibf, 0); int(err)) return err;
  outb(port, k::ec_data);

  if (auto err = ec_io_wait(k::ec_sc, k::ibf, 0); int(err)) return err;
  outb(value, k::ec_data);

  return ec_io_wait(k::ec_sc, k::ibf, 0);
}

uint8_t ec_io_read(const uint8_t port) {
  ec_io_wait(k::ec_sc, k::ibf, 0);
  outb(k::ec_sc_read_cmd, k::ec_sc);

  ec_io_wait(k::ec_sc, k::ibf, 0);
  outb(port, k::ec_data);

  ec_io_wait(k::ec_sc, k::obf, 1);
  return inb(k::ec_data);
}

int32_t calculate_fan_duty(int32_t raw_duty) {
  return static_cast<int32_t>(static_cast<double>(raw_duty) / 255.0 * 100.0);
}

int32_t calculate_fan_rpms(int32_t raw_rpm_high, int32_t raw_rpm_low) {
  int32_t raw_rpm = (raw_rpm_high << 8) + raw_rpm_low;
  return raw_rpm > 0 ? (2156220 / raw_rpm) : 0;
}

int32_t ec_query_cpu_temp() { return ec_io_read(k::ec_reg_cpu_temp); }

int32_t ec_query_gpu_temp() { return ec_io_read(k::ec_reg_gpu_temp); }

int32_t ec_query_fan_duty() {
  return calculate_fan_duty(ec_io_read(k::ec_reg_fan_duty));
}

int32_t ec_query_fan_rpms() {
  uint8_t raw_rpm_hi = ec_io_read(k::ec_reg_fan_rpms_hi);
  uint8_t raw_rpm_lo = ec_io_read(k::ec_reg_fan_rpms_lo);
  return calculate_fan_rpms(raw_rpm_hi, raw_rpm_lo);
}

std::errc ec_write_fan_duty(int32_t duty_percentage) {
  if (duty_percentage < 0 || duty_percentage > 100) {
    fmt::print("Wrong fan duty to write: {}\n", duty_percentage);
    return std::errc::invalid_argument;
  }
  return ec_io_do(
      0x99,
      0x01,
      static_cast<uint8_t>(static_cast<double>(duty_percentage) / 100.0 * 255.0)
  );
}

void dump_fan() {
  fmt::print("{{\n");
  fmt::print("  \"duty\": {},\n", ec_query_fan_duty());
  fmt::print("  \"rpms\": {},\n", ec_query_fan_rpms());
  fmt::print("  \"cpu_temp_cels\": {},\n", ec_query_cpu_temp());
  fmt::print("  \"gpu_temp_cels\": {},\n", ec_query_gpu_temp());
  fmt::print("}}\n");
}

int32_t identify_duty(int32_t duty) {
  std::array<int32_t, 7> allowed_duties = {0, 16, 30, 40, 65, 90, 100};
  if (auto res = std::ranges::find_if(
          allowed_duties,
          [&](int32_t d) {
            constexpr int32_t range     = 1;
            int32_t           min_right = d ? d - range : d;
            int32_t           max_right = d ? d + range : d;
            return duty >= min_right && duty <= max_right;
          }
      );
      res != allowed_duties.end())
    return *res;
  return duty;
}

int32_t
ec_auto_duty_adjust(int32_t cpu_temp, int32_t gpu_temp, int32_t fan_duty) {
  const int32_t temp_max = std::max(cpu_temp, gpu_temp);
  const int32_t duty     = identify_duty(fan_duty);
  int32_t       new_duty = temp_max >= 85 && duty < 65  ? 65
                         : temp_max >= 75 && duty < 40  ? 40
                         : temp_max >= 65 && duty < 30  ? 30
                         : temp_max >= 55 && duty < 17  ? 17
                         : temp_max <= 50               ? 0
                         : temp_max <= 60 && duty >= 17 ? 17
                         : temp_max <= 70 && duty >= 30 ? 30
                         : temp_max <= 80 && duty >= 40 ? 40
                         : temp_max <= 85 && duty >= 65 ? 65
                                                        : -1;

  if (new_duty > fan_duty) {
    int32_t new_duty_adj = duty + ((new_duty - fan_duty) / 2);
    if (new_duty - new_duty_adj > 2) {
      fmt::print("using adjusted new duty={}%\n", new_duty_adj);
      new_duty = new_duty_adj;
    }
  }

  return new_duty;
}

std::errc ec_worker() {
  setuid(0);
  system("modprobe ec_sys");

  int32_t cpu_temp = 0, gpu_temp = 0, fan_duty = 0, auto_duty_val = -1;
  while (global_running_flag) {
    int io_fd = open("/sys/kernel/debug/ec/ec0/io", O_RDONLY, 0);
    if (io_fd < 0) {
      fmt::print("unable to read EC from sysfs\n");
      return std::errc(errno);
    }

    std::array<uint8_t, k::ec_reg_size> buf{};
    ssize_t len = read(io_fd, buf.data(), buf.size());

    switch (len) {
    case -1:
      return fmt::print("unable to read EC from sysfs\n"), std::errc(errno);
    case 0x100: {
      cpu_temp = buf[k::ec_reg_cpu_temp];
      gpu_temp = buf[k::ec_reg_gpu_temp];
      fan_duty = calculate_fan_duty(buf[k::ec_reg_fan_duty]);
      break;
    }
    default: fmt::print("wrong EC size from sysfs: {}\n", len); break;
    }

    close(io_fd);

    int32_t next_duty = ec_auto_duty_adjust(cpu_temp, gpu_temp, fan_duty);
    if ((next_duty != -1 && next_duty != auto_duty_val) ||
        (next_duty == 0 && fan_duty != 0)) {
      fmt::print(
          "{:%m/%d %H:%M:%S} - CPU={}°C, GPU={}°C, auto fan duty to {}%\n",
          std::chrono::system_clock::now(),
          cpu_temp,
          gpu_temp,
          next_duty
      );
      ec_write_fan_duty(next_duty);
      auto_duty_val = next_duty;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  fmt::print("worker quit\n");
  return std::errc(EXIT_SUCCESS);
}

void print_help() {
  fmt::print(R"(
Usage: clevo-fancontrol [fan-duty-percentage|-1]

Dump/Control fan duty on Clevo laptops. Display indicator by default.

Arguments:
  [fan-duty-percentage|-1]  Target fan duty in percentage, from 40 to 100
  -h, --help                Display this help and exit

Without arguments this program will dump current fan duty and temperature in JSON
format. The binary requires running as root - either directly or with
setuid=root flag.
This program would always attempt to load kernel
module 'ec_sys', in order to query EC information from
'/sys/kernel/debug/ec/ec0/io' instead of polling EC ports for readings,
which may be more risky if interrupted or concurrently operated during the
process.

DO NOT MANIPULATE OR QUERY EC I/O PORTS WHILE THIS PROGRAM IS RUNNING.

)");
}

std::errc set_fan(int32_t duty_percentage) {
  fmt::print("Change fan duty to {}%\n", duty_percentage);
  if (auto err = ec_write_fan_duty(duty_percentage); int(err)) return err;
  fmt::print("\n");
  dump_fan();
  return std::errc(EXIT_SUCCESS);
}

void ec_on_sigterm(int signum) {
  constexpr int32_t fan_reset = 40;
  fmt::print(
      "ec on signal: {}\n, resetting to {}%\n", strsignal(signum), fan_reset
  );
  global_running_flag = false;
  set_fan(fan_reset);
}

void install_signal_handler() {
  std::signal(SIGHUP, ec_on_sigterm);
  std::signal(SIGINT, ec_on_sigterm);
  std::signal(SIGQUIT, ec_on_sigterm);
  std::signal(SIGPIPE, ec_on_sigterm);
  std::signal(SIGALRM, ec_on_sigterm);
  std::signal(SIGTERM, ec_on_sigterm);
  std::signal(SIGUSR1, ec_on_sigterm);
  std::signal(SIGUSR2, ec_on_sigterm);
}

std::error_code ec_main(std::span<std::string_view> args) {
  if (auto err = make_error_code(ec_init())) {
    fmt::print("unable to control EC: {}\n", err.message());
    return err;
  }
  if (args.size() <= 1) return dump_fan(), std::error_code();
  if (std::ranges::any_of(args.subspan(1), [](std::string_view s) {
        return s == "--help" || s == "-h";
      }))
    return print_help(), dump_fan(), std::error_code();

  int32_t val;
  if (auto err = make_error_code(
          std::from_chars(args[1].begin(), args[1].end(), val).ec
      );
      err || val < -1 || val > 100) {
    fmt::print("invalid fan duty {}!\n", args[1]);
    if (err) fmt::print("{}\n", err.message());
    return err;
  }

  if (val == -1) {
    install_signal_handler();
    if (auto err = make_error_code(ec_worker())) {
      fmt::print("worker failed: {}\n", err.message());
      return err;
    }
    return {};
  }

  if (auto err = make_error_code(set_fan(val))) {
    fmt::print("set fan failed: {}\n", err.message());
    return err;
  }
  return {};
}
} // namespace

int main(const int argc, const char* argv[]) {
  const auto arg_cnt = static_cast<std::size_t>(argc);

  std::vector<std::string_view> args;
  args.reserve(arg_cnt);
  std::copy_n(argv, arg_cnt, std::back_inserter(args));

  return ec_main(args) ? EXIT_FAILURE : EXIT_SUCCESS;
}